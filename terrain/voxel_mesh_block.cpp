#include "voxel_mesh_block.h"
#include "../constants/voxel_string_names.h"
#include "../util/godot/classes/collision_shape_3d.h"
#include "../util/godot/classes/concave_polygon_shape_3d.h"
#include "../util/godot/classes/node_3d.h"
#include "../util/macros.h"
#include "../util/profiling.h"
#include "free_mesh_task.h"
#include "../util/godot/classes/navigation_server_3d.h"
#include "../util/godot/classes/navigation_mesh.h"

namespace zylann::voxel {

VoxelMeshBlock::VoxelMeshBlock(Vector3i bpos) {
	position = bpos;
}

VoxelMeshBlock::~VoxelMeshBlock() {
	FreeMeshTask::try_add_and_destroy(_mesh_instance);
	drop_navmesh(); // ADD - frees the NavigationServer3D region
}

void VoxelMeshBlock::set_world(Ref<World3D> p_world) {
	if (_world != p_world) {
		_world = p_world;

		// To update world. I replaced visibility by presence in world because Godot 3 culling performance is horrible
		_set_visible(_visible && _parent_visible);

		if (_static_body.is_valid()) {
			_static_body.set_world(*p_world);
		}
	}
}

void VoxelMeshBlock::set_gi_mode(GeometryInstance3D::GIMode mode) {
	if (_mesh_instance.is_valid()) {
		_mesh_instance.set_gi_mode(mode);
	}
}

void VoxelMeshBlock::set_shadow_casting(RenderingServer::ShadowCastingSetting setting) {
	if (_mesh_instance.is_valid()) {
		_mesh_instance.set_cast_shadows_setting(setting);
	}
}

void VoxelMeshBlock::set_render_layers_mask(int mask) {
	if (_mesh_instance.is_valid()) {
		_mesh_instance.set_render_layers_mask(mask);
	}
}

void VoxelMeshBlock::set_mesh(
		Ref<Mesh> mesh,
		GeometryInstance3D::GIMode gi_mode,
		RenderingServer::ShadowCastingSetting shadow_setting,
		int render_layers_mask
) {
	// TODO Don't add mesh instance to the world if it's not visible.
	// I suspect Godot is trying to include invisible mesh instances into the culling process,
	// which is killing performance when LOD is used (i.e many meshes are in pool but hidden)
	// This needs investigation.

	if (mesh.is_valid()) {
		if (!_mesh_instance.is_valid()) {
			// Create instance if it doesn't exist
			_mesh_instance.create();
			_mesh_instance.set_interpolated(false);
			_mesh_instance.set_gi_mode(gi_mode);
			_mesh_instance.set_cast_shadows_setting(shadow_setting);
			_mesh_instance.set_render_layers_mask(render_layers_mask);
			set_mesh_instance_visible(_mesh_instance, _visible && _parent_visible);
		}

		_mesh_instance.set_mesh(mesh);

#ifdef VOXEL_DEBUG_LOD_MATERIALS
		_mesh_instance.set_material_override(_debug_material);
#endif

	} else {
		if (_mesh_instance.is_valid()) {
			// Delete instance if it exists
			_mesh_instance.destroy();
		}
	}
}

Ref<Mesh> VoxelMeshBlock::get_mesh() const {
	if (_mesh_instance.is_valid()) {
		return _mesh_instance.get_mesh();
	}
	return Ref<Mesh>();
}

bool VoxelMeshBlock::has_mesh() const {
	return _mesh_instance.get_mesh().is_valid();
}

void VoxelMeshBlock::drop_mesh() {
	if (_mesh_instance.is_valid()) {
		_mesh_instance.destroy();
	}
}

void VoxelMeshBlock::set_visible(bool visible) {
	if (_visible == visible) {
		return;
	}
	_visible = visible;
	_set_visible(_visible && _parent_visible);
}

bool VoxelMeshBlock::is_visible() const {
	return _visible;
}

void VoxelMeshBlock::_set_visible(bool visible) {
	if (_mesh_instance.is_valid()) {
		set_mesh_instance_visible(_mesh_instance, visible);
	}
}

void VoxelMeshBlock::set_parent_visible(bool parent_visible) {
	if (_parent_visible && parent_visible) {
		return;
	}
	_parent_visible = parent_visible;
	_set_visible(_visible && _parent_visible);
}

void VoxelMeshBlock::set_parent_transform(const Transform3D &parent_transform) {
	ZN_PROFILE_SCOPE();

	if (_mesh_instance.is_valid() || _static_body.is_valid()) {
		const Transform3D local_transform(Basis(), _position_in_voxels);
		const Transform3D world_transform = parent_transform * local_transform;

		if (_mesh_instance.is_valid()) {
			_mesh_instance.set_transform(world_transform);
		}

		if (_static_body.is_valid()) {
			_static_body.set_transform(world_transform);
		}
	}
}

void VoxelMeshBlock::set_collision_shape(Ref<Shape3D> shape, bool debug_collision, const Node3D *node, float margin) {
	ERR_FAIL_COND(node == nullptr);
	ERR_FAIL_COND_MSG(node->get_world_3d() != _world, "Physics body and attached node must be from the same world");

	if (shape.is_null()) {
		drop_collision();
		return;
	}

	if (!_static_body.is_valid()) {
		_static_body.create();
		_static_body.set_world(*_world);
		// This allows collision signals to provide the terrain node in the `collider` field
		_static_body.set_attached_object(node);

	} else {
		_static_body.remove_shape(0);
	}

	shape->set_margin(margin);

	_static_body.add_shape(shape);
	_static_body.set_debug(debug_collision, *_world);
	_static_body.set_shape_enabled(0, _collision_enabled);
}

bool VoxelMeshBlock::has_collision_shape() const {
	return _static_body.is_valid();
}

void VoxelMeshBlock::set_collision_layer(int layer) {
	if (_static_body.is_valid()) {
		_static_body.set_collision_layer(layer);
	}
}

void VoxelMeshBlock::set_collision_mask(int mask) {
	if (_static_body.is_valid()) {
		_static_body.set_collision_mask(mask);
	}
}

void VoxelMeshBlock::set_collision_margin(float margin) {
	if (_static_body.is_valid()) {
		Ref<Shape3D> shape = _static_body.get_shape(0);
		if (shape.is_valid()) {
			shape->set_margin(margin);
		}
	}
}

void VoxelMeshBlock::drop_collision() {
	if (_static_body.is_valid()) {
		_static_body.destroy();
	}
}

void VoxelMeshBlock::set_collision_enabled(bool enable) {
	if (_collision_enabled == enable) {
		return;
	}
	if (_static_body.is_valid()) {
		_static_body.set_shape_enabled(0, enable);
	}
	_collision_enabled = enable;
}

bool VoxelMeshBlock::is_collision_enabled() const {
	return _collision_enabled;
}


bool VoxelMeshBlock::has_navmesh() const {
	return _nav_region.is_valid();
}

void VoxelMeshBlock::drop_navmesh() {
	if (_nav_region.is_valid()) {
		NavigationServer3D::get_singleton()->free_rid(_nav_region);
		_nav_region = RID();
	}
}

void VoxelMeshBlock::update_navmesh(const PackedVector3Array &vertices, const Transform3D &terrain_transform) {
	if (vertices.is_empty()) {
		drop_navmesh();
		return;
	}

	NavigationServer3D *nav = NavigationServer3D::get_singleton();

    if (!_nav_region.is_valid()) {
		_nav_region = nav->region_create();
		nav->region_set_enabled(_nav_region, true);
		if (_world.is_valid()) {
			nav->region_set_map(_nav_region, _world->get_navigation_map());
		}
	}

	// _position_in_voxels is Vector3i - must cast to Vector3 explicitly
	const Vector3 local_pos((float)_position_in_voxels.x, (float)_position_in_voxels.y, (float)_position_in_voxels.z);
	const Transform3D local_transform(Basis(), local_pos);
	nav->region_set_transform(_nav_region, terrain_transform * local_transform);

	Ref<NavigationMesh> nav_mesh;
	nav_mesh.instantiate();
	nav_mesh->set_cell_size(0.5f);
	nav_mesh->set_cell_height(0.25f);
	nav_mesh->set_vertices(vertices);

	const int tri_count = vertices.size() / 3;
	for (int i = 0; i < tri_count; ++i) {
		PackedInt32Array tri;
		tri.resize(3);
		int32_t *w = tri.ptrw();
		w[0] = i * 3;
		w[1] = i * 3 + 1;
		w[2] = i * 3 + 2;
		nav_mesh->add_polygon(tri);
	}

	nav->region_set_navigation_mesh(_nav_region, nav_mesh);
}

Ref<ConcavePolygonShape3D> make_collision_shape_from_mesher_output(
		const VoxelMesher::Output &mesher_output,
		const VoxelMesher &mesher
) {
	using namespace zylann::godot;

	Ref<ConcavePolygonShape3D> shape;

	if (mesher.is_generating_collision_surface()) {
		if (mesher_output.collision_surface.submesh_vertex_end != -1) {
			// Use a sub-region of the render mesh
			if (mesher_output.surfaces.size() > 0) {
				shape = create_concave_polygon_shape(
						mesher_output.surfaces[0].arrays,
						mesher_output.collision_surface.submesh_vertex_end,
						mesher_output.collision_surface.submesh_index_end
				);
			}

		} else {
			// Use specialized collision mesh
			shape = create_concave_polygon_shape(
					to_span(mesher_output.collision_surface.positions), to_span(mesher_output.collision_surface.indices)
			);
		}

	} else {
		// Use render mesh
		static const unsigned int MAX_STACK_SURFACES = 8;

		if (mesher_output.surfaces.size() <= MAX_STACK_SURFACES) {
			// Use stack
			std::array<Array, MAX_STACK_SURFACES> render_surfaces_s;
			for (unsigned int i = 0; i < mesher_output.surfaces.size(); ++i) {
				render_surfaces_s[i] = mesher_output.surfaces[i].arrays;
			}
			Span<const Array> render_surfaces(render_surfaces_s.data(), mesher_output.surfaces.size());
			shape = create_concave_polygon_shape(render_surfaces);

		} else {
			// Use heap
			StdVector<Array> render_surfaces_h;
			render_surfaces_h.reserve(mesher_output.surfaces.size());
			for (const VoxelMesher::Output::Surface &surface : mesher_output.surfaces) {
				render_surfaces_h.push_back(surface.arrays);
			}
			shape = create_concave_polygon_shape(to_span(render_surfaces_h));
		}
	}

	return shape;
}


enum FaceDirection {
	FACE_TOP, // +Y
	FACE_BOTTOM, // -Y
	FACE_NORTH, // +Z
	FACE_SOUTH, // -Z
	FACE_EAST, // +X
	FACE_WEST, // -X
	FACE_COUNT
};

FaceDirection classify_normal(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    // Raw cross product, no normalize needed for axis-aligned geometry
    const Vector3 n = (c - a).cross(b - a);
    float ax = Math::abs(n.x);
    float ay = Math::abs(n.y);
    float az = Math::abs(n.z);
    if (ay >= ax && ay >= az) {
        return n.y > 0 ? FACE_TOP : FACE_BOTTOM;
    } else if (ax >= az) {
        return n.x > 0 ? FACE_EAST : FACE_WEST;
    } else {
        return n.z > 0 ? FACE_NORTH : FACE_SOUTH;
    }
}

// Returns an array of 6 PackedVector3Arrays, one per direction
FixedArray<PackedVector3Array, FACE_COUNT> split_mesh_by_face_direction(const PackedVector3Array &flat_vertices) {
    FixedArray<PackedVector3Array, FACE_COUNT> buckets;

    const Vector3 *src = flat_vertices.ptr();

    const int tri_count = flat_vertices.size() / 3;

    for (int i = 0; i < tri_count; ++i) {
        const Vector3 &a = src[i * 3 + 0];
        const Vector3 &b = src[i * 3 + 1];
        const Vector3 &c = src[i * 3 + 2];

        const FaceDirection dir = classify_normal_fast(a, b, c);  // changed
        buckets[dir].push_back(a);
        buckets[dir].push_back(b);
        buckets[dir].push_back(c);
    }

	return buckets;
}

PackedVector3Array make_navmesh_vertices_from_mesher_output(
		const VoxelMesher::Output &mesher_output,
		const VoxelMesher &mesher
) {
	PackedVector3Array vertices;

	if (mesher.is_generating_collision_surface()) {
		if (mesher_output.collision_surface.submesh_vertex_end != -1) {
			// The collision geometry is a sub-region of surface 0 of the render mesh.
			// Extract only up to the vertex count the collision surface uses.
			if (mesher_output.surfaces.size() > 0) {
				const Array &arrays = mesher_output.surfaces[0].arrays;
				const PackedVector3Array all_verts = arrays[Mesh::ARRAY_VERTEX];
				const PackedInt32Array all_indices = arrays[Mesh::ARRAY_INDEX];

				const int vert_end = mesher_output.collision_surface.submesh_vertex_end;
				const int idx_end = mesher_output.collision_surface.submesh_index_end;

				// Unpack indexed triangles into a flat vertex list (3 verts per triangle)
				// NavigationMesh polygons need consistent winding so we respect the index buffer
				for (int i = 0; i + 2 < idx_end; i += 3) {
					const int i0 = all_indices[i];
					const int i1 = all_indices[i + 1];
					const int i2 = all_indices[i + 2];
					if (i0 < vert_end && i1 < vert_end && i2 < vert_end) {
						vertices.push_back(all_verts[i0]);
						vertices.push_back(all_verts[i1]);
						vertices.push_back(all_verts[i2]);
					}
				}
			}

		} else {
			// Specialized collision surface - positions and indices are already separate
			const auto &positions = mesher_output.collision_surface.positions;
			const auto &indices = mesher_output.collision_surface.indices;

			for (int i = 0; i + 2 < (int)indices.size(); i += 3) {
				vertices.push_back(zylann::to_vec3(positions[indices[i]]));
				vertices.push_back(zylann::to_vec3(positions[indices[i + 1]]));
				vertices.push_back(zylann::to_vec3(positions[indices[i + 2]]));
			}
		}

	} else {
		// No specialized collision surface - use render mesh surface 0
		if (mesher_output.surfaces.size() > 0) {
			const Array &arrays = mesher_output.surfaces[0].arrays;
			const PackedVector3Array all_verts = arrays[Mesh::ARRAY_VERTEX];
			const PackedInt32Array all_indices = arrays[Mesh::ARRAY_INDEX];

			if (all_indices.size() > 0) {
				// Indexed mesh - unpack triangles
				for (int i = 0; i + 2 < all_indices.size(); i += 3) {
					vertices.push_back(all_verts[all_indices[i]]);
					vertices.push_back(all_verts[all_indices[i + 1]]);
					vertices.push_back(all_verts[all_indices[i + 2]]);
				}
			} else {
				// Non-indexed mesh - already flat triangles
				vertices = all_verts;
			}
		}
	}

	auto buckets = split_mesh_by_face_direction(vertices);
	PackedVector3Array top_faces = buckets[FACE_TOP];

	return top_faces;
}

} // namespace zylann::voxel
