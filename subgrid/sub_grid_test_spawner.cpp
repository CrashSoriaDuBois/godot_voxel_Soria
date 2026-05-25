#include "sub_grid_test_spawner.h"
#include "assembly/sub_grid_assembler.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "voxel_sub_grid.h"
#include "scene/main/viewport.h"
#include "scene/3d/camera_3d.h"

namespace zylann::voxel {

void SubGridTestSpawner::_bind_methods() {
	ClassDB::bind_method(D_METHOD("assemble_at", "terrain", "world_pos"), &SubGridTestSpawner::assemble_at);
}

void SubGridTestSpawner::assemble_at(Node *terrain_node, Vector3i world_pos) {
	VoxelLodTerrain *terrain = Object::cast_to<VoxelLodTerrain>(terrain_node);
	ERR_FAIL_COND_MSG(terrain == nullptr, "terrain_node must be VoxelLodTerrain");

	SubGridAssembler::AssemblyConfig config;
	config.max_blocks = 4096;
	config.diagonal_stick = true;
	config.bearing_voxel_id_min = 2;
	config.bearing_voxel_id_max = 7;

	String error;
	SubGridAssembler::AssembledBody *body = SubGridAssembler::assemble(terrain, world_pos, config, error);

	if (body == nullptr) {
		print_line(String("Assembly failed: ") + error);
		return;
	}

	print_line(String("Assembly succeeded. Block count: ") + itos(body->world_blocks.size()));
	print_line(String("Child body count: ") + itos(body->children.size()));

	Ref<VoxelMesherBlocky> mesher = terrain->get_mesher();
	ERR_FAIL_COND_MSG(!mesher.is_valid(), "Terrain must use VoxelMesherBlocky");
	Ref<VoxelBlockyLibrary> library = mesher->get_library();
	ERR_FAIL_COND_MSG(!library.is_valid(), "Mesher must have a VoxelBlockyLibrary");

	String saves_dir = "user://subgrid_saves";

	// Spawn recursively
	VoxelSubGrid *root_sg = memnew(VoxelSubGrid);
	get_parent()->add_child(root_sg); // add to tree FIRST

	// Now initialize
	SubGridMetadata meta;
	SubGridAssembler::generate_uuid_v4(meta.uuid);
	meta.is_root = true;
	meta.world_position =
			Vector3(body->local_origin_in_world.x, body->local_origin_in_world.y, body->local_origin_in_world.z);
	meta.world_rotation = Quaternion();

	root_sg->initialize_root(meta, std::move(body->chunks), saves_dir, mesher, library);

	// Spawn children recursively
	for (SubGridAssembler::AssembledBody *child_body : body->children) {
		_spawn_body(child_body, root_sg, saves_dir, mesher, library);
	}

	Node3D *cam = Object::cast_to<Node3D>(get_viewport()->get_camera_3d());
	if (cam != nullptr) {
		root_sg->set_viewer(cam);
	}

	SubGridAssembler::_free_tree(body);
	print_line("VoxelSubGrid tree spawned successfully");
}

VoxelSubGrid *SubGridTestSpawner::_spawn_body(
		SubGridAssembler::AssembledBody *body,
		VoxelSubGrid *parent_sg,
		const String &saves_dir,
		Ref<VoxelMesherBlocky> mesher,
		Ref<VoxelBlockyLibrary> library
) {
	VoxelSubGrid *sg = memnew(VoxelSubGrid);
	parent_sg->add_child(sg); // add to tree before initialize_child

	SubGridMetadata meta = body->metadata;
	// keep uuid from metadata, already set during assembly

	sg->initialize_child(meta, std::move(body->chunks), saves_dir);

	// Recurse
	for (SubGridAssembler::AssembledBody *child_body : body->children) {
		_spawn_body(child_body, sg, saves_dir, mesher, library);
	}

	return sg;
}

} // namespace zylann::voxel