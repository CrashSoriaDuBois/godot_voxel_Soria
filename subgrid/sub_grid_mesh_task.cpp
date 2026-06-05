#include "sub_grid_mesh_task.h"

namespace zylann::voxel {

SubGridMeshTaskResult run_mesh_task(SubGridMeshTaskInput input) {
	SubGridMeshTaskResult result;
	result.ship_uuid = input.ship_uuid;
	result.chunk_pos = input.chunk_pos;
	result.lod = input.lod;

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

	// Collision only at LOD 0. higher LODs are too coarse for physics
	if (input.build_collision && input.lod == 0) {
		const int chunk_size = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
		Vector3i chunk_voxel_origin = input.chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;

		result.collision = SubGridCollisionBuilder::build(
				*input.padded_buffer, chunk_voxel_origin, chunk_size, input.weight_table
		);
		result.has_collision = true;
	}

	return result;
}

} // namespace zylann::voxel