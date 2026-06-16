#pragma once
#include "core/os/mutex.h"
#include "core/templates/hash_map.h"
#include "core/templates/hash_set.h"
#include "meshers/blocky/voxel_blocky_library.h"
#include "meshers/blocky/voxel_mesher_blocky.h"
#include "scene/main/node.h"
#include "sub_grid_collision_builder.h"
#include "sub_grid_mesh_task.h"
#include "sub_grid_metadata.h"
#include <future>
#include <vector>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "streams/sqlite/voxel_stream_sqlite.h"
#include "core/config/project_settings.h"

// Forward declare Godot types to avoid heavy includes in header
class AnimatableBody3D;

namespace zylann::voxel {

class VoxelSubGrid;
class VoxelLodTerrain;

class SubGridManager : public Node {
	GDCLASS(SubGridManager, Node)

public:
	// -----------------------------------------------------------------------
	// Setup

	void initialize(
			VoxelLodTerrain *terrain,
			const String &saves_dir,
			Ref<VoxelMesherBlocky> mesher,
			Ref<VoxelBlockyLibrary> library
	);

	// Block mass table. call before ships are assembled or loaded.
	// Any voxel ID not listed uses default_mass.
	void set_block_mass(int voxel_id, float mass);
	void set_default_block_mass(float mass);

	// -----------------------------------------------------------------------
	// Ship lifecycle

	void register_ship_tree(VoxelSubGrid *root);
	void unregister_ship(const String &uuid_str);
	void mark_chunk_dirty(const String &uuid_str, Vector3i chunk_pos);


	String uuid_for_node(VoxelSubGrid *node) const;
	void mark_all_dirty(const String &uuid);

	// -----------------------------------------------------------------------
	// threading
	struct SaveRequest {
		std::string uuid;
		std::string saves_dir;
		Vector3i chunk_pos;
		std::shared_ptr<VoxelBuffer> buffer;
		bool close_stream = false;
	};

	// Save thread owns all streams, never accessed from main thread
	struct SaveThreadData {
		std::mutex mutex;
		std::condition_variable cv;
		std::atomic<bool> running{ false };
		std::atomic<int> items_in_flight{ 0 };
		std::vector<SaveRequest> queue;
		HashMap<String, Ref<VoxelStreamSQLite>> streams;
	} _save_thread_data;

	std::thread _save_thread;

	void push_save(const SaveRequest &req);

	void wait_save_queue();
	void _save_thread_func();

	// -----------------------------------------------------------------------
	// Persistence

	void save_all();
	void load_all();

	// -----------------------------------------------------------------------
	// Physics body interaction

	void grab_subgrid(VoxelSubGrid *sg, Vector3 grab_point_local, bool rotate = false);
	void release_subgrid(VoxelSubGrid *sg);
	void set_grab_target(VoxelSubGrid *sg, Transform3D target);
	void apply_impulse(VoxelSubGrid *sg, Vector3 impulse, Vector3 world_point);
	void apply_central_impulse(VoxelSubGrid *sg, Vector3 impulse);

protected:
	void _notification(int p_what);
	static void _bind_methods();

private:
	// -----------------------------------------------------------------------
	// Rendering data per chunk

	struct ChunkRenderData {
		RID instance_rid;
		Ref<ArrayMesh> mesh;
	};

	// -----------------------------------------------------------------------
	// Collision data per chunk. updated when collision results arrive.
	// Stored so we can recalculate ship-wide CoM incrementally.

	struct ChunkCollisionData {
		Vector<RID> shape_rids;
		Vector3 weighted_pos; // center_of_mass * mass for this chunk
		float mass = 0.f;
	};

	// -----------------------------------------------------------------------
	// Per-ship state

	enum LoadState { SLEEPING, LOADED };

	struct ShipState {
		VoxelSubGrid *node = nullptr;
		LoadState load_state = SLEEPING;
		String parent_uuid; // empty for root ships

		// Rendering
		HashSet<Vector3i> dirty_chunks;
		HashSet<Vector3i> in_flight_chunks;
		HashMap<Vector3i, int> current_lod;
		HashMap<uint64_t, ChunkRenderData> chunk_renders;

		// Physics. root ships own a RigidBody via server RID.
		// Child sub-contraptions own an AnimatableBody3D node (driven manually,
		// avoids joint constraint overhead).
		RID body_rid; // rigid body (roots only)
		AnimatableBody3D *animatable_body = nullptr; // kinematic body (children only)

		// Per-chunk collision shapes and mass data.
		// Keyed by chunk_pos (un-padded chunk grid coordinates).
		HashMap<Vector3i, ChunkCollisionData> chunk_collision;

		bool grabbed = false;
		bool grab_rotate = false;
		Vector3 grab_point_local;
		Transform3D grab_target;
	};

	// -----------------------------------------------------------------------
	// Main-thread data

	VoxelLodTerrain *_terrain = nullptr;
	String _saves_dir;
	Ref<VoxelMesherBlocky> _mesher;
	Ref<VoxelBlockyLibrary> _library;
	BlockWeightTable _weight_table;

	HashMap<String, ShipState> _ships;

	float _load_distance = 200.f;
	float _unload_distance = 250.f;

	// -----------------------------------------------------------------------
	// Threading

	static constexpr int MAX_CONCURRENT_TASKS = 8;
	std::vector<std::future<SubGridMeshTaskResult>> _pending_futures;

	// -----------------------------------------------------------------------
	// _process(). mesh, LOD, streaming, task poll

	void _process_mesh(double delta);
	void _process_streaming();
	void _process_lod_for_ship(const String &uuid, ShipState &state);
	void _submit_pending_tasks(const String &uuid, ShipState &state);
	void _poll_completed_tasks();

	void _submit_one_task(const String &uuid, ShipState &state, Vector3i chunk_pos, int lod);
	void _apply_mesh_result(const SubGridMeshTaskResult &result);

	std::shared_ptr<VoxelBuffer> _build_padded_buffer(VoxelSubGrid *node, Vector3i chunk_pos) const;

	// -----------------------------------------------------------------------
	// _physics_process(). rotation, transform sync

	void _process_physics(double delta);
	void _update_rotations(double delta);
	void _sync_all_transforms();

	void _drive_grabbed_ships(double delta);

	// -----------------------------------------------------------------------
	// Physics body management

	void _create_root_body(const String &uuid, ShipState &state);
	void _create_child_body(const String &uuid, ShipState &state);
	void _destroy_body(ShipState &state);

	void _apply_collision_result(ShipState &state, Vector3i chunk_pos, const SubGridCollisionOutput &col);
	void _remove_chunk_collision(ShipState &state, Vector3i chunk_pos);
	void _recalculate_com(const String &uuid, ShipState &state);

	// -----------------------------------------------------------------------
	// Streaming

	void _load_ship(const String &uuid, ShipState &state);
	void _unload_ship(const String &uuid, ShipState &state);
	void _free_chunk_renders(ShipState &state);

	// -----------------------------------------------------------------------
	// LOD

	static const float LOD_DISTANCES[4];
	int _compute_lod(VoxelSubGrid *node, Vector3i chunk_pos) const;
	int _total_in_flight() const;

	// -----------------------------------------------------------------------
	// Registration

	void _register_single(VoxelSubGrid *sg, const String &parent_uuid);
	void _mark_all_dirty(ShipState &state);

	// -----------------------------------------------------------------------
	// Helpers

	static uint64_t _chunk_mesh_key(Vector3i p, int lod) {
		return ((uint64_t)(uint16_t)p.x) | ((uint64_t)(uint16_t)p.y << 16) | ((uint64_t)(uint16_t)p.z << 32) |
				((uint64_t)(uint8_t)lod << 48);
	}
	static Vector3i _key_to_chunk_pos(uint64_t key) {
		return Vector3i((int16_t)(key & 0xFFFF), (int16_t)((key >> 16) & 0xFFFF), (int16_t)((key >> 32) & 0xFFFF));
	}
	static int _key_to_lod(uint64_t key) {
		return (int)((key >> 48) & 0xFF);
	}

	void _save_metadata_index(const Vector<SubGridMetadata> &metas);
	Vector<SubGridMetadata> _load_metadata_index();
	void _collect_metadata_recursive(VoxelSubGrid *sg, Vector<SubGridMetadata> &out);

	static String _uuid_to_string(const uint8_t *uuid) {
		String s;
		for (int i = 0; i < 16; i++) {
			s += String::num_int64(uuid[i] >> 4, 16);
			s += String::num_int64(uuid[i] & 0xF, 16);
		}
		return s;
	}
};

} // namespace zylann::voxel