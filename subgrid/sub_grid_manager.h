#pragma once
#include "core/os/mutex.h"
#include "core/templates/hash_map.h"
#include "core/templates/hash_set.h"
#include "meshers/blocky/voxel_blocky_library.h"
#include "meshers/blocky/voxel_mesher_blocky.h"
#include "scene/main/node.h"
#include "sub_grid_mesh_task.h"
#include "sub_grid_metadata.h"
#include "voxel_sub_grid.h"
#include <future>
#include <vector>

namespace zylann::voxel {

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

	// -----------------------------------------------------------------------
	// Ship lifecycle
	//
	// Call register_ship_tree() after assembly or load to enqueue initial meshes.
	// Call unregister_ship() before destroying the root node.

	void register_ship_tree(VoxelSubGrid *root);
	void unregister_ship(const String &uuid_str);

	// Mark a specific chunk dirty for a ship (call after editing voxels).
	void mark_chunk_dirty(const String &uuid_str, Vector3i chunk_pos);

	// -----------------------------------------------------------------------
	// Persistence (unchanged API)

	void save_all();
	void load_all();

protected:
	void _notification(int p_what);
	static void _bind_methods();

private:
	// -----------------------------------------------------------------------
	// Per-ship state, tracked for every VoxelSubGrid node (root and children).

	struct ShipState {
		VoxelSubGrid *node = nullptr;
		HashSet<Vector3i> dirty_chunks; // need mesh rebuild
		HashSet<Vector3i> in_flight_chunks; // task submitted, not yet applied
		HashMap<Vector3i, int> current_lod; // last built lod per chunk
	};

	// -----------------------------------------------------------------------
	// Main-thread data (never touched from worker threads)

	VoxelLodTerrain *_terrain = nullptr;
	String _saves_dir;
	Ref<VoxelMesherBlocky> _mesher;
	Ref<VoxelBlockyLibrary> _library;

	// uuid string -> per-ship state
	HashMap<String, ShipState> _ships;

	// -----------------------------------------------------------------------
	// Threading
	//
	// Futures are polled every _process() with zero timeout.
	// Completed results are applied immediately on the main thread.
	// MAX_CONCURRENT_TASKS prevents unbounded thread spawning.

	static constexpr int MAX_CONCURRENT_TASKS = 8;
	std::vector<std::future<SubGridMeshTaskResult>> _pending_futures;

	// -----------------------------------------------------------------------
	// _process() helpers

	void _process_all_ships();
	void _process_lod_for_ship(const String &uuid, ShipState &state);
	void _submit_pending_tasks(const String &uuid, ShipState &state);
	void _poll_completed_tasks();

	void _submit_one_task(const String &uuid, ShipState &state, Vector3i chunk_pos, int lod);
	void _apply_mesh_result(const SubGridMeshTaskResult &result);

	// Builds the padded VoxelBuffer for one chunk on the main thread.
	// Returns nullptr if the chunk buffer doesn't exist yet.
	std::shared_ptr<VoxelBuffer> _build_padded_buffer(VoxelSubGrid *node, Vector3i chunk_pos) const;

	// LOD distance thresholds copied from VoxelSubGrid (keep in sync).
	static const float LOD_DISTANCES[4];

	int _compute_lod(VoxelSubGrid *node, Vector3i chunk_pos) const;

	// -----------------------------------------------------------------------
	// Registration helpers

	void _register_single(VoxelSubGrid *sg);
	void _mark_all_dirty(ShipState &state);

	int _total_in_flight() const;

	// -----------------------------------------------------------------------
	// Persistence helpers (unchanged from original)

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