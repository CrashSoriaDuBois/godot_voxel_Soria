#pragma once
#include "../meshers/blocky/voxel_mesher_blocky.h"
#include "../meshers/voxel_mesher.h"
#include "../storage/voxel_buffer.h"
#include "lod/sub_grid_chunk_map.h"
#include "sub_grid_collision_builder.h"
#include <memory>

namespace zylann::voxel {
	
struct SubGridMeshTaskInput {
	String ship_uuid;
	Vector3i chunk_pos;
	int lod = 0;
	std::shared_ptr<VoxelBuffer> padded_buffer;
	std::shared_ptr<VoxelBuffer> light_padded_buffer;
	bool light_dirty = false;
	bool requires_geometry = true;
	Ref<VoxelMesherBlocky> mesher;

	// Collision is only built at lod == 0.
	// At higher LODs we skip the collision pass entirely.
	bool build_collision = true;
	BlockWeightTable weight_table;
};

struct SubGridMeshTaskResult {
	String ship_uuid;
	Vector3i chunk_pos;
	int lod = 0;
	bool light_only = false;
	VoxelMesher::Output output;

	// Only populated when input.build_collision == true && lod == 0.
	SubGridCollisionOutput collision;
	bool has_collision = false;
};

SubGridMeshTaskResult run_mesh_task(SubGridMeshTaskInput input);

} // namespace zylann::voxel
