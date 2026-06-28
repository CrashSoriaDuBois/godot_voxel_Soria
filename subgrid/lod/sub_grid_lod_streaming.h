#pragma once


#include "../../util/containers/span.h"
#include "../sub_grid_manager.h" // SubGridManager::ShipState

namespace zylann::voxel {

void schedule_chunk_remesh(ShipLod &lod, Vector3i bpos);

// this is the part that handles a chunk a voxel edit just created that no viewer's box-diff has ever seen before, 
// which schedule_chunk_remesh alone can't. 
// it only re-triggers chunks already in ShipLod::mesh_state, and a brand new chunk isn't in there yet.
void notify_chunk_edited(SubGridManager::ShipState &state, Vector3i bpos, int lod_index);

void process_ship_lod_streaming(
		SubGridManager::ShipState &state,
		const SubGridChunkMap &chunk_map,
		Span<const float> lod_distances,
		float viewer_pairing_distance
);

} // namespace zylann::voxel