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

#include "core/config/project_settings.h"
#include "lod/sub_grid_chunk_map.h"
#include "lod/sub_grid_lod_state.h"
#include "streams/sqlite/voxel_stream_sqlite.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

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
			Ref<VoxelBlockyLibrary> library,
			float view_distance = 512.f,
			int lod_count = 4,
			float lod_distance = 48.f,
			float secondary_lod_distance = 48.f
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

	void grab_subgrid(VoxelSubGrid *sg, Vector3 grab_point_local, bool rotate = false, float strength = 1.0f);
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
	//
	// ShipState is public (only this one struct - everything below stays private) because
	// sub_grid_lod_streaming.h needs to name SubGridManager::ShipState in its function
	// signature. See subgrid/lod/sub_grid_lod_streaming.h for the rationale.

	enum LoadState { SLEEPING, LOADED };

public:
	struct ShipState {
		VoxelSubGrid *node = nullptr;
		LoadState load_state = SLEEPING;
		String parent_uuid;

		// MIRROR of VoxelLodTerrainUpdateData::Lod, one per LOD level. Replaces the old
		// dirty_chunks_per_lod / in_flight_per_lod / pending_requeue_per_lod / active_lod_chunks
		// quartet of HashSets with a single per-LOD map + output lists.
		// See subgrid/lod/sub_grid_lod_state.h for what each field mirrors and why.
		FixedArray<ShipLod, SUBGRID_MAX_LODS> lods;

		// MIRROR of VoxelLodTerrainUpdateData::ClipboxStreamingState::paired_viewers, scoped
		// per-ship instead of per-volume.
		Vector<PairedShipViewer> paired_viewers;

		// MIRROR of VoxelLodTerrainUpdateData::ClipboxStreamingState::loaded_mesh_blocks,
		// minus the mutex (producer and consumer are both main-thread, same frame - see
		// LoadedChunkEvent's comment in sub_grid_lod_state.h).
		Vector<LoadedChunkEvent> pending_loaded_chunks;

		// Render instances keyed by (lod_space_chunk_pos, lod).
		// In the new system lod_space_chunk_pos is in LOD-space (not LOD0 space).
		HashMap<uint64_t, ChunkRenderData> chunk_renders;

		// Physics
		RID body_rid;
		AnimatableBody3D *animatable_body = nullptr;
		HashMap<Vector3i, ChunkCollisionData> chunk_collision;
		HashSet<Vector3i> collision_built_chunks; // LOD0 positions only
		int32_t min_chunk_y = INT32_MAX;

		// Count of currently-active chunks at lod > SubGridManager::_collision_safe_lod_max.
		// Maintained incrementally in _apply_lod_visibility_changes as to_activate_visuals /
		// to_deactivate_visuals get drained - cheaper than rescanning mesh_state every frame,
		// and correct as long as every active->inactive AND active->gone transition pushes a
		// matching to_deactivate_visuals entry (see unview_mesh_box's comment on this).
		int coarse_lod_active_count = 0;
		// Whether body_rid (root rigidbodies only) is currently force-slept because of the
		// above. Tracked separately so body_set_state(BODY_STATE_SLEEPING) is only called on
		// the actual 0<->positive transition, not every frame.
		bool collision_suspended = false;

		bool grabbed = false;
		bool grab_rotate = false;
		Vector3 grab_point_local;
		Transform3D grab_target;
		float grab_strength = 1.0f;
	};

private:
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

	std::shared_ptr<VoxelBuffer> _build_padded_buffer(VoxelSubGrid *node, Vector3i chunk_pos, int lod) const;

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
	// Collision layers
	static constexpr int SUBGRID_PART_LAYER_BIT = 10;
	static constexpr int TERRAIN_ANCHORED_LAYER_BIT = 11;
	static constexpr int SHIP_HULL_LAYER_BIT = 12;

	void _apply_collision_layers(ShipState &state, bool is_terrain_anchored);

	// -----------------------------------------------------------------------
	// LOD

	// Number of LOD levels actually in use, 1..SUBGRID_MAX_LODS. Set (and clamped) by initialize(). SubGridChunkMap always 
	// builds all SUBGRID_MAX_LODS levels regardless (see sub_grid_chunk_map.h), this only controls how many of them the 
	// streaming/viewer-pairing system in sub_grid_lod_streaming.cpp actually treats as real, including which one it treats as the "root" (no-parent) level.
	int _lod_count = 4;

	//Stored separately from the derived per-LOD distances below so initialize() can be called again
	// later if you ever want to support reconfiguring at runtime (not wired up yet, would need to recompute
	// _lod_distances and re-pair all ships' viewer boxes)
	float _lod_distance = 48.f;
	float _secondary_lod_distance = 48.f;

	// Per-LOD distance in world voxels, index 0..(_lod_count - 1). Computed once in initialize() by _recompute_lod_distances()
	FixedArray<float, SUBGRID_MAX_LODS> _lod_distances;

	// Recomputes _lod_distances from _lod_distance / _secondary_lod_distance / _lod_count
	// Called once from initialize()
	void _recompute_lod_distances();

	// Pairing range for sub_grid_lod_streaming.cpp's viewer pairing pass: must be at least
	// LOD_DISTANCES[SUBGRID_MAX_LODS - 1], plus margin so a viewer doesn't pair/unpair right
	// at the edge of the outermost LOD box every frame.
	float _viewer_pairing_distance = 0.f;

	// Highest LOD index at which the terrain we're physically resting on is still guaranteed
	// to have collision. LOD_DISTANCES must be tuned to coarsen the subgrid before the
	// terrain itself loses collision at the same real-world distance, so that "subgrid is at
	// lod > this" is always a safe, slightly-early signal to suspend physics, never a late one.
	// E.g. terrain has collision at LOD 0 and 1 -> this is 1.
	int _collision_safe_lod_max = 1;

	int _total_in_flight() const;

	// Returns the viewer position in world space from VoxelEngine,falling back to the node's manually assigned viewer.
	Vector3 _get_viewer_world_pos(VoxelSubGrid *node) const;

	// Drains state.lods[*].to_activate_visuals / to_deactivate_visuals / to_unload (populated
	// by process_ship_lod_streaming in sub_grid_lod_streaming.cpp) into actual
	// RenderingServer::instance_set_visible calls / freed render instances. Entries here are
	// guaranteed to have a chunk_renders entry UNLESS the chunk's mesh task hasn't completed
	// yet (e.g. unview_mesh_box's "show parent immediately" branch can reactivate a chunk
	// whose own mesh is still in flight) - in that case this is a harmless no-op, and
	// _apply_mesh_result reads the chunk's current visual_active when it eventually creates
	// the instance, so the correct visibility still gets applied, just one frame later.
	void _apply_lod_visibility_changes(ShipState &state);

	// Suspends (sleeps) or wakes state.body_rid based on state.coarse_lod_active_count vs
	// _collision_safe_lod_max. No-op for ships with no body_rid (sub-contraptions are
	// AnimatableBody3D, kinematic, never gravity-simulated, so there's nothing to suspend).
	// Grabbing a suspended ship wakes it automatically as a side effect of
	// _drive_grabbed_ships setting BODY_STATE_LINEAR_VELOCITY every physics frame - no
	// special-casing needed here for that.
	void _update_collision_suspension(ShipState &state);
	bool _center_collision_safe(const ShipState &state) const;
	// World offset of a LOD-space chunk in subgrid-local space
	static Vector3 _lod_chunk_local_offset(Vector3i lod_pos, int lod) {
		const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
		return Vector3(lod_pos * (cs << lod));
	}

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