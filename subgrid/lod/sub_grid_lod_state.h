#pragma once

#include "../../engine/ids.h" // ViewerID
#include "../../util/containers/fixed_array.h"
#include "../../util/math/box3i.h"
#include "core/templates/hash_map.h"
#include "core/templates/vector.h"
#include "sub_grid_chunk_map.h" // SUBGRID_MAX_LODS

namespace zylann::voxel {

enum SubGridMeshState : uint8_t {
	MESH_NEVER_UPDATED = 0,
	MESH_UP_TO_DATE,
	MESH_NEED_UPDATE, // out of date, not yet scheduled
	MESH_UPDATE_NOT_SENT, // scheduled, sitting in pending_update, no task fired yet
	MESH_UPDATE_SENT // task fired, waiting on result
};

struct ChunkMeshBlockState {
	SubGridMeshState state = MESH_NEVER_UPDATED;
	bool visual_active = false;
	bool visual_loaded = false;
	int update_list_index = -1;
	int mesh_viewers = 0;
	bool light_dirty = true; // needs its light (re)computed at next build
	bool requires_geometry = true; // false = flood-only task, no mesh/collision rebuild
	bool pending_light_retry = false;
};

struct ShipLod {
	HashMap<Vector3i, ChunkMeshBlockState> mesh_state;

	Vector<Vector3i> pending_update;

	// Drained once per frame by SubGridManager to flip render-instance visibility / free them.
	Vector<Vector3i> to_activate_visuals;
	Vector<Vector3i> to_deactivate_visuals;
	Vector<Vector3i> to_unload;
};

struct PairedShipViewer {
	struct State {
		Vector3i local_position_voxels;
		FixedArray<Box3i, SUBGRID_MAX_LODS> mesh_box_per_lod;
	};
	ViewerID id;
	State state;
	State prev_state;
};

struct LoadedChunkEvent {
	Vector3i position; // LOD-space
	uint8_t lod_index = 0;
};

} // namespace zylann::voxel
