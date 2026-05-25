#pragma once

#include "scene/main/node.h"
#include "assembly/sub_grid_assembler.h"
#include "voxel_sub_grid.h"
#include "meshers/blocky/voxel_mesher_blocky.h"
#include "meshers/blocky/voxel_blocky_library.h"

namespace zylann::voxel {

class SubGridTestSpawner : public Node {
	GDCLASS(SubGridTestSpawner, Node)

public:
	void assemble_at(Node *terrain_node, Vector3i world_pos);

protected:
	static void _bind_methods();

private:
	VoxelSubGrid *_spawn_body(
			SubGridAssembler::AssembledBody *body,
			VoxelSubGrid *parent_sg,
			const String &saves_dir,
			Ref<VoxelMesherBlocky> mesher,
			Ref<VoxelBlockyLibrary> library);
};

} // namespace zylann::voxel