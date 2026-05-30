#pragma once
#include "../meshers/blocky/voxel_mesher_blocky.h"
#include "../meshers/voxel_mesher.h"
#include "../storage/voxel_buffer.h"
#include "lod/sub_grid_chunk_map.h"
#include <memory>

namespace zylann::voxel {

// Owned by a pending std::future. Built and moved on the main thread,
// read-only on the worker thread. The padded_buffer is a deep copy so the
// worker never races with main-thread chunk edits.
struct SubGridMeshTaskInput {
	String ship_uuid; // identifies which VoxelSubGrid owns this chunk
	Vector3i chunk_pos;
	int lod = 0;
	std::shared_ptr<VoxelBuffer> padded_buffer; // full copy, cs+2 on each side
	Ref<VoxelMesherBlocky> mesher; // shared, build() is stateless
};

struct SubGridMeshTaskResult {
	String ship_uuid;
	Vector3i chunk_pos;
	int lod = 0;
	VoxelMesher::Output output; // raw surfaces, transferred to main thread on completion
};

// Free function called inside std::async. Takes input by value (moved in).
SubGridMeshTaskResult run_mesh_task(SubGridMeshTaskInput input);

} // namespace zylann::voxel