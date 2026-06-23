#pragma once


#include "../../util/containers/span.h"
#include "../sub_grid_manager.h" // SubGridManager::ShipState

namespace zylann::voxel {

void schedule_chunk_remesh(ShipLod &lod, Vector3i bpos);

void process_ship_lod_streaming(
		SubGridManager::ShipState &state,
		const SubGridChunkMap &chunk_map,
		Span<const float> lod_distances,
		float viewer_pairing_distance
);

} // namespace zylann::voxel