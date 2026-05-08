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

void VoxelMeshBlock::update_navmesh(const PackedVector3Array &vertices, const Transform3D &terrain_transform, RID navigation_map) {
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
		PackedInt32Array poly;
		poly.resize(3);
		int32_t *w = poly.ptrw();
		w[0] = i * 3;
		w[1] = i * 3 + 1;
		w[2] = i * 3 + 2;
		nav_mesh->add_polygon(poly);
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

 //REMEMBER if Instead of growing arrays dynamically if we reserve capacity upfront it will make
 //two passes over the data but eliminates all heap reallocations

//tris to quads
static PackedVector3Array merge_triangles_to_quads(const PackedVector3Array &flat_tris) {
	PackedVector3Array quads;
	const int tri_count = flat_tris.size() / 3;
	const Vector3 *src = flat_tris.ptr();

	for (int i = 0; i + 1 < tri_count; i += 2) {
		const Vector3 *ta[3] = { &src[i * 3], &src[i * 3 + 1], &src[i * 3 + 2] };
		const Vector3 *tb[3] = { &src[(i + 1) * 3], &src[(i + 1) * 3 + 1], &src[(i + 1) * 3 + 2] };

		int shared_a[2] = {}, shared_b[2] = {}, unique_a = -1, unique_b = -1, ns = 0;
		for (int ia = 0; ia < 3; ++ia) {
			bool found = false;
			for (int ib = 0; ib < 3; ++ib) {
				if (ta[ia]->is_equal_approx(*tb[ib])) {
					shared_a[ns] = ia;
					shared_b[ns] = ib;
					++ns;
					found = true;
					break;
				}
			}
			if (!found)
				unique_a = ia;
		}
		for (int ib = 0; ib < 3; ++ib) {
			if (ib != shared_b[0] && ib != shared_b[1]) {
				unique_b = ib;
				break;
			}
		}

		if (ns == 2 && unique_a != -1 && unique_b != -1) {
			int a_next = (unique_a + 1) % 3;
			int a_next2 = (unique_a + 2) % 3;
			bool b_after_is_first = tb[(unique_b + 1) % 3]->is_equal_approx(*ta[a_next]);
			quads.push_back(*ta[unique_a]);
			quads.push_back(b_after_is_first ? *tb[unique_b] : *ta[a_next]);
			quads.push_back(b_after_is_first ? *ta[a_next] : *tb[unique_b]);
			quads.push_back(*ta[a_next2]);
		} else {
			quads.push_back(*ta[0]);
			quads.push_back(*ta[1]);
			quads.push_back(*ta[2]);
			quads.push_back(*ta[2]);
		}
	}

	if (tri_count % 2 != 0) {
		const Vector3 *t = &src[(tri_count - 1) * 3];
		quads.push_back(t[0]);
		quads.push_back(t[1]);
		quads.push_back(t[2]);
		quads.push_back(t[2]);
	}

	return quads;
}

//split by face direction
enum FaceDirection {
	FACE_TOP, // +Y
	FACE_BOTTOM, // -Y
	FACE_NORTH, // +Z
	FACE_SOUTH, // -Z
	FACE_EAST, // +X
	FACE_WEST, // -X
	FACE_COUNT
};

static FaceDirection classify_normal(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
	const Vector3 n = (c - a).cross(b - a);
	float ax = Math::abs(n.x);
	float ay = Math::abs(n.y);
	float az = Math::abs(n.z);
	if (ay >= ax && ay >= az)
		return n.y > 0 ? FACE_TOP : FACE_BOTTOM;
	if (ax >= az)
		return n.x > 0 ? FACE_EAST : FACE_WEST;


	return n.z > 0 ? FACE_NORTH : FACE_SOUTH;



}

// Returns an array of 6 PackedVector3Arrays, one per direction
static FixedArray<PackedVector3Array, FACE_COUNT> split_mesh_by_face_direction(const PackedVector3Array &quads) {
	FixedArray<PackedVector3Array, FACE_COUNT> buckets;
	const Vector3 *src = quads.ptr();
	const int quad_count = quads.size() / 4;

	for (int i = 0; i < quad_count; ++i) {
		const Vector3 &a = src[i * 4 + 0];
		const Vector3 &b = src[i * 4 + 1];
		const Vector3 &c = src[i * 4 + 2];
		const Vector3 &d = src[i * 4 + 3];
		FaceDirection dir = classify_normal(a, b, c);
		buckets[dir].push_back(a);
		buckets[dir].push_back(b);
		buckets[dir].push_back(c);
		buckets[dir].push_back(d);
	}

	return buckets;
}

//make ramps

static uint64_t encode_edge(const Vector3 &va, const Vector3 &vb) {
	Vector2i a2(Math::round(va.x), Math::round(va.z));
	Vector2i b2(Math::round(vb.x), Math::round(vb.z));
	if (b2 < a2) {
		Vector2i tmp = a2;
		a2 = b2;
		b2 = tmp;
	}
	return ((uint64_t)(uint16_t)a2.x) | ((uint64_t)(uint16_t)a2.y << 16) | ((uint64_t)(uint16_t)b2.x << 32) |
			((uint64_t)(uint16_t)b2.y << 48);
}

static PackedVector3Array apply_step_ramps(const PackedVector3Array &top_quads, float voxel_size = 1.0f) {
    const int vert_count = top_quads.size();
    if (vert_count == 0)
        return top_quads;

    const Vector3 *src = top_quads.ptr();
    HashSet<Vector3i> occupied;

    for (int i = 0; i < vert_count; ++i) {
        Vector3i key(
            Math::round(src[i].x / voxel_size),
            Math::round(src[i].y / voxel_size),
            Math::round(src[i].z / voxel_size)
        );
        occupied.insert(key);
    }

    PackedVector3Array result = top_quads;
    Vector3 *dst = result.ptrw();

    for (int i = 0; i < vert_count; ++i) {
        Vector3i above(
            Math::round(dst[i].x / voxel_size),
            Math::round(dst[i].y / voxel_size) + 1,
            Math::round(dst[i].z / voxel_size)
        );
        if (occupied.has(above)) {
            dst[i].y += voxel_size;
        }
    }

    return result;
}

//navmesh
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

	PackedVector3Array quads = merge_triangles_to_quads(vertices);
	auto buckets = split_mesh_by_face_direction(quads);
	//PackedVector3Array top_faces = buckets[FACE_TOP];
	PackedVector3Array ramped_top = apply_step_ramps(buckets[FACE_TOP]);

	return vertices;
}

} // namespace zylann::voxel
