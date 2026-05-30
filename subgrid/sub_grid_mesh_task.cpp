#include "sub_grid_mesh_task.h"

namespace zylann::voxel {

SubGridMeshTaskResult run_mesh_task(SubGridMeshTaskInput input) {
	SubGridMeshTaskResult result;
	result.ship_uuid = input.ship_uuid;
	result.chunk_pos = input.chunk_pos;
	result.lod = input.lod;

	// The padded buffer's origin is shifted back by 1 (the pad) so that the
	// mesher maps voxel [1,1,1] to world pos chunk_pos*chunk_size correctly.
	const int pad = 1;
	VoxelMesher::Input mesher_input{ *input.padded_buffer,
									 nullptr,
									 (input.chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2) - Vector3i(pad, pad, pad),
									 (uint8_t)input.lod,
									 false,
									 false,
									 false,
									 false };

	input.mesher->build(result.output, mesher_input);
	return result;
}

} // namespace zylann::voxel