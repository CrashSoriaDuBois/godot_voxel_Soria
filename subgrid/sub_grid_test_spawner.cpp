#include "sub_grid_test_spawner.h"
#include "assembly/sub_grid_assembler.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "voxel_sub_grid.h"
#include "scene/main/viewport.h"
#include "scene/3d/camera_3d.h"
#include "sub_grid_manager.h"

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

	const VoxelFormat root_format = body->chunks.get_format();
	root_sg->initialize_root(meta, std::move(body->chunks), saves_dir, mesher, library, root_format);

	// Spawn children recursively
	for (SubGridAssembler::AssembledBody *child_body : body->children) {
		if (child_body->terrain_anchored) {
			// Spawn as a root, anchored to terrain at the bearing position
			_spawn_terrain_anchored(child_body, terrain, saves_dir, mesher, library);
		} else {
			_spawn_body(child_body, root_sg, saves_dir, mesher, library);
		}
	}

	Node3D *cam = Object::cast_to<Node3D>(get_viewport()->get_camera_3d());
	if (cam != nullptr) {
		root_sg->set_viewer(cam);
	}

	// Register full tree with SubGridManager now that all children exist.
	SubGridManager *mgr = Object::cast_to<SubGridManager>(get_parent()->get_node_or_null(String("SubGridManager")));
	if (mgr != nullptr) {
		mgr->register_ship_tree(root_sg);
		print_line("Registered ship tree with SubGridManager");
	} else {
		print_line("WARNING: SubGridManager not found - mesh will not generate");
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

	const VoxelFormat child_format = body->chunks.get_format();
	sg->initialize_child(meta, std::move(body->chunks), saves_dir, /*async_stream=*/true, child_format);

	// Recurse
	for (SubGridAssembler::AssembledBody *child_body : body->children) {
		_spawn_body(child_body, sg, saves_dir, mesher, library);
	}

	return sg;
}

VoxelSubGrid *SubGridTestSpawner::_spawn_terrain_anchored(
		SubGridAssembler::AssembledBody *body,
		VoxelLodTerrain *terrain,
		const String &saves_dir,
		Ref<VoxelMesherBlocky> mesher,
		Ref<VoxelBlockyLibrary> library
) {
	// Terrain-anchored sub-contraption: spawns as a root subgrid
	// but its pivot is relative to the terrain, not another subgrid.
	VoxelSubGrid *sg = memnew(VoxelSubGrid);
	get_parent()->add_child(sg);

	SubGridMetadata meta = body->metadata;
	meta.is_root = true; // anchored to terrain = acts as its own root
	// World position is the local origin in world space from assembly
	meta.world_position =
			Vector3(body->local_origin_in_world.x, body->local_origin_in_world.y, body->local_origin_in_world.z);
	meta.world_rotation = Quaternion();

	const VoxelFormat anchored_format = body->chunks.get_format(); // capture before move
	sg->initialize_root(meta, std::move(body->chunks), saves_dir, mesher, library, anchored_format);

	// Recurse: children of a terrain-anchored body spawn normally
	for (SubGridAssembler::AssembledBody *child_body : body->children) {
		_spawn_body(child_body, sg, saves_dir, mesher, library);
	}

	Node3D *cam = Object::cast_to<Node3D>(get_viewport()->get_camera_3d());
	if (cam != nullptr) {
		sg->set_viewer(cam);
	}

	SubGridManager *mgr = Object::cast_to<SubGridManager>(get_parent()->get_node_or_null(String("SubGridManager")));
	if (mgr != nullptr) {
		mgr->register_ship_tree(sg);
	}

	return sg;
}
} // namespace zylann::voxel