#include "voxel_box_mover.h"
#include "../../meshers/blocky/voxel_mesher_blocky.h"
#include "../../meshers/cubes/voxel_mesher_cubes.h"
#include "../../storage/voxel_buffer.h"
#include "../../storage/voxel_data.h"
#include "../../util/containers/std_vector.h"
#include "../../util/profiling.h"
#include "../variable_lod/voxel_lod_terrain.h"
#include "voxel_terrain.h"

#include "../../storage/metadata/voxel_grid_metadata.h"
#include "../../util/thread/spatial_lock_3d.h"

namespace zylann::voxel {

namespace {

AABB expand_with_vector(AABB box, Vector3 v) {
	if (v.x > 0) {
		box.size.x += v.x;
	} else if (v.x < 0) {
		box.position.x += v.x;
		box.size.x -= v.x;
	}

	if (v.y > 0) {
		box.size.y += v.y;
	} else if (v.y < 0) {
		box.position.y += v.y;
		box.size.y -= v.y;
	}

	if (v.z > 0) {
		box.size.z += v.z;
	} else if (v.z < 0) {
		box.position.z += v.z;
		box.size.z -= v.z;
	}

	return box;
}

real_t calculate_i_offset(const AABB &box, AABB other, real_t motion, int i, int j, int k) {
	const real_t EPSILON = 0.001;

	const Vector3 other_end = other.position + other.size;
	const Vector3 box_end = box.position + box.size;

	if (other_end[k] <= box.position[k] || other.position[k] >= box_end[k]) {
		return motion;
	}

	if (other_end[j] <= box.position[j] || other.position[j] >= box_end[j]) {
		return motion;
	}

	if (motion > 0.0 && other_end[i] <= box.position[i]) {
		const real_t off = box.position[i] - other_end[i] - EPSILON;
		if (off < motion) {
			motion = off;
		}
	}

	if (motion < 0.0 && other.position[i] >= box_end[i]) {
		const real_t off = box_end[i] - other.position[i] + EPSILON;
		if (off > motion) {
			motion = off;
		}
	}

	return motion;
}

// Gets the transformed vector for moving a box and slide.
// This algorithm is free from tunnelling for axis-aligned movement,
// except in some high-speed diagonal cases or huge size differences:
// For example, if a box is fast enough to have a diagonal motion jumping from A to B,
// it will pass through C if that other box is the only other one:
//
//  o---o
//  | A |
//  o---o
//          o---o
//          | C |
//          o---o
//                  o---o
//                  | B |
//                  o---o
//
// TODO one way to fix this would be to try a "hot side" projection instead
//
Vector3 get_motion(AABB box, Vector3 motion, Span<const AABB> environment_boxes) {
	// The bounding box is expanded to include it's estimated version at next update.
	// This also makes the algorithm tunnelling-free
	const AABB expanded_box = expand_with_vector(box, motion);

	StdVector<AABB> colliding_boxes;
	for (size_t i = 0; i < environment_boxes.size(); ++i) {
		const AABB &other = environment_boxes[i];
		if (expanded_box.intersects(other)) {
			colliding_boxes.push_back(other);
		}
	}

	if (colliding_boxes.size() == 0) {
		return motion;
	}

	// print("Colliding: ", colliding_boxes.size())

	Vector3 new_motion = motion;

	for (unsigned int i = 0; i < colliding_boxes.size(); ++i) {
		new_motion.y = calculate_i_offset(colliding_boxes[i], box, new_motion.y, 1, 0, 2);
	}
	box.position.y += new_motion.y;

	for (unsigned int i = 0; i < colliding_boxes.size(); ++i) {
		new_motion.x = calculate_i_offset(colliding_boxes[i], box, new_motion.x, 0, 1, 2);
	}
	box.position.x += new_motion.x;

	for (unsigned int i = 0; i < colliding_boxes.size(); ++i) {
		new_motion.z = calculate_i_offset(colliding_boxes[i], box, new_motion.z, 2, 1, 0);
	}
	box.position.z += new_motion.z;

	return new_motion;
}

// bool raycast_down(Span<const AABB> aabbs, Vector3 ray_origin, real_t &out_hit_y) {
// 	if (aabbs.size() == 0) {
// 		return false;
// 	}
// 	bool hit = false;
// 	real_t max_y = -9999999;
// 	for (unsigned int i = 0; i < aabbs.size(); ++i) {
// 		const AABB &aabb = aabbs[i];
// 		if ( //
// 				ray_origin.x >= aabb.position.x && //
// 				ray_origin.z >= aabb.position.z && //
// 				ray_origin.x < aabb.position.x + aabb.size.x && //
// 				ray_origin.z < aabb.position.z + aabb.size.z) {
// 			//
// 			const real_t box_top = aabb.position.y + aabb.size.y;
// 			if (hit) {
// 				max_y = math::max(box_top, max_y);
// 			} else {
// 				max_y = box_top;
// 			}
// 			hit = true;
// 		}
// 	}
// 	out_hit_y = max_y;
// 	return hit;
// }

inline Vector2 get_xz(Vector3 v) {
	return Vector2(v.x, v.z);
}

bool boxcast_down(Span<const AABB> aabbs, Vector2 box_pos, Vector2 box_size, real_t &out_hit_y) {
	if (aabbs.size() == 0) {
		return false;
	}
	bool hit = false;
	real_t max_y = -9999999;
	for (unsigned int i = 0; i < aabbs.size(); ++i) {
		const AABB &aabb = aabbs[i];
		if (Rect2(box_pos, box_size).intersects(Rect2(get_xz(aabb.position), get_xz(aabb.size)))) {
			const real_t box_top = aabb.position.y + aabb.size.y;
			if (hit) {
				max_y = math::max(box_top, max_y);
			} else {
				max_y = box_top;
			}
			hit = true;
		}
	}
	out_hit_y = max_y;
	return hit;
}

bool intersects(Span<const AABB> aabbs, const AABB &box) {
	for (unsigned int i = 0; i < aabbs.size(); ++i) {
		if (aabbs[i].intersects(box)) {
			return true;
		}
	}
	return false;
}


// voxelgrid methods
inline bool is_full_cube_collider(const blocky::BakedModel &m) {
	return m.box_collision_aabbs.size() == 1 && m.box_collision_aabbs[0].position.is_equal_approx(Vector3()) &&
			m.box_collision_aabbs[0].size.is_equal_approx(Vector3(1, 1, 1));
}

void build_grid_collision_boxes(
		const VoxelGridMetadata &g,
		const blocky::BakedLibrary &lib,
		const uint32_t collision_mask,
		StdVector<AABB> &out
) {
	const int res = g.res;
	const float inv = 1.f / float(res);
	uint64_t used = 0;

	auto id_at = [&](int x, int y, int z) { return g.cells[g.index(x, y, z)]; };
	auto is_free = [&](int x, int y, int z) { return (used & (uint64_t(1) << g.index(x, y, z))) == 0; };
	auto mergeable = [&](int x, int y, int z) {
		const uint16_t id = id_at(x, y, z);
		if (id == 0 || !lib.has_model(id)) {
			return false;
		}
		const blocky::BakedModel &m = lib.models[id];
		return (m.box_collision_mask & collision_mask) != 0 && is_full_cube_collider(m);
	};

	for (int z = 0; z < res; ++z) {
		for (int x = 0; x < res; ++x) {
			for (int y = 0; y < res; ++y) {
				if (!is_free(x, y, z)) {
					continue;
				}
				const uint16_t id = id_at(x, y, z);
				if (id == 0 || !lib.has_model(id)) {
					continue;
				}
				const blocky::BakedModel &m = lib.models[id];
				if ((m.box_collision_mask & collision_mask) == 0) {
					continue;
				}

				if (!is_full_cube_collider(m)) {
					// Partial model (slab, fence...): scale its own boxes into the cell
					for (const AABB &b : m.box_collision_aabbs) {
						out.push_back(AABB((b.position + Vector3(x, y, z)) * inv, b.size * inv));
					}
					continue;
				}

				// Grow a box: along y, then x, then z
				int y1 = y + 1;
				while (y1 < res && is_free(x, y1, z) && mergeable(x, y1, z)) {
					++y1;
				}
				int x1 = x + 1;
				for (; x1 < res; ++x1) {
					bool ok = true;
					for (int yy = y; yy < y1 && ok; ++yy) {
						ok = is_free(x1, yy, z) && mergeable(x1, yy, z);
					}
					if (!ok) {
						break;
					}
				}
				int z1 = z + 1;
				for (; z1 < res; ++z1) {
					bool ok = true;
					for (int xx = x; xx < x1 && ok; ++xx) {
						for (int yy = y; yy < y1 && ok; ++yy) {
							ok = is_free(xx, yy, z1) && mergeable(xx, yy, z1);
						}
					}
					if (!ok) {
						break;
					}
				}
				for (int zz = z; zz < z1; ++zz) {
					for (int xx = x; xx < x1; ++xx) {
						for (int yy = y; yy < y1; ++yy) {
							used |= uint64_t(1) << g.index(xx, yy, zz);
						}
					}
				}
				out.push_back(AABB(Vector3(x, y, z) * inv, Vector3(x1 - x, y1 - y, z1 - z) * inv));
			}
		}
	}
}
void append_grid_boxes(
		const VoxelGridMetadata &g,
		const blocky::BakedLibrary &lib,
		const uint32_t mask,
		const Vector3i vpos,
		StdVector<AABB> &out
) {
	MutexLock lock(g.collision_mutex);
	if (!g.collision_cache_valid || g.collision_cache_mask != mask) {
		g.collision_boxes.clear();
		build_grid_collision_boxes(g, lib, mask, g.collision_boxes);
		g.collision_cache_mask = mask;
		g.collision_cache_valid = true;
	}
	for (const AABB &b : g.collision_boxes) {
		AABB w = b;
		w.position += vpos;
		out.push_back(w);
	}
}

// One pass over the sparse metadata of the blocks overlapping the box: one lock per block, cost proportional
// to the number of grids, not the number of voxels. Appends their boxes and remembers which voxels were grids.
void collect_grid_boxes(
		const VoxelData &voxels,
		const blocky::BakedLibrary &lib,
		const Vector3i minp,
		const Vector3i maxp,
		const uint32_t mask,
		StdVector<AABB> &out_boxes,
		StdVector<Vector3i> &out_grid_positions
) {
	const int bs = voxels.get_block_size();
	const Box3i block_box = Box3i::from_min_max(minp, maxp).downscaled(bs);

	// Gather first, lock afterwards (gathering takes the same lock internally, don't nest it)
	StdVector<std::shared_ptr<VoxelBuffer>> blocks(Vector3iUtil::get_volume_u64(block_box.size));
	voxels.get_blocks_with_voxel_data(block_box, 0, to_span(blocks));

	SpatialLock3D::Read srlock(voxels.get_spatial_lock(0), BoxBounds3i(block_box));

	unsigned int index = 0;
	block_box.for_each_cell_zxy([&](const Vector3i bpos) {
		const std::shared_ptr<VoxelBuffer> &vb = blocks[index++];
		if (vb == nullptr) {
			return;
		}
		const Vector3i origin = bpos * bs;
		vb->for_each_voxel_metadata_in_area(
				Box3i(minp - origin, maxp - minp),
				[&](const Vector3i rpos, const VoxelMetadata &meta) {
					if (meta.get_type() != VoxelGridMetadata::TYPE_INDEX) {
						return;
					}
					const Vector3i vpos = origin + rpos;
					out_grid_positions.push_back(vpos);
					append_grid_boxes(
							static_cast<const VoxelGridMetadata &>(meta.get_custom()), lib, mask, vpos, out_boxes
					);
				}
		);
	});
}

void collect_boxes_blocky(
		const VoxelData &voxels,
		const VoxelMesherBlocky &mesher,
		const Vector3i minp,
		const Vector3i maxp,
		const uint32_t collision_mask,
		StdVector<AABB> &potential_boxes
) {
	Ref<VoxelBlockyLibraryBase> library_ref = mesher.get_library();
	ERR_FAIL_COND_MSG(library_ref.is_null(), "VoxelMesherBlocky has no library assigned");
	const int channel = VoxelBuffer::CHANNEL_TYPE;
	VoxelSingleValue defval;
	defval.i = 0;

	const blocky::BakedLibrary &baked_data = library_ref->get_baked_data();

	Vector3i i = minp;

	static thread_local StdVector<Vector3i> tls_grid_positions;
	tls_grid_positions.clear();
	collect_grid_boxes(voxels, baked_data, minp, maxp, collision_mask, potential_boxes, tls_grid_positions);

	// TODO Optimization: read the whole box of voxels at once, querying individually is slower
	for (i.z = minp.z; i.z < maxp.z; ++i.z) {
		for (i.y = minp.y; i.y < maxp.y; ++i.y) {
			for (i.x = minp.x; i.x < maxp.x; ++i.x) {
				const int type_id = voxels.get_voxel(i, channel, defval).i;

				if (!tls_grid_positions.empty()) {
					bool is_grid = false;
					for (const Vector3i &gp : tls_grid_positions) {
						if (gp == i) {
							is_grid = true;
							break;
						}
					}
					if (is_grid) {
						continue; // precise boxes already added, ignore the fallback block's boxes
					}
				}

				if (baked_data.has_model(type_id)) {
					const blocky::BakedModel &model = baked_data.models[type_id];

					if ((model.box_collision_mask & collision_mask) == 0) {
						continue;
					}

					for (const AABB &aabb : model.box_collision_aabbs) {
						AABB world_box = aabb;
						world_box.position += i;
						potential_boxes.push_back(world_box);
					}
				}
			}
		}
	}
}

void collect_boxes_cubes(
		const VoxelData &voxels,
		const Vector3i minp,
		const Vector3i maxp,
		StdVector<AABB> &potential_boxes
) {
	const int channel = VoxelBuffer::CHANNEL_COLOR;
	VoxelSingleValue defval;
	defval.i = 0;

	Vector3i i = minp;

	// TODO Optimization: read the whole box of voxels at once, querying individually is slower
	for (i.z = minp.z; i.z < maxp.z; ++i.z) {
		for (i.y = minp.y; i.y < maxp.y; ++i.y) {
			for (i.x = minp.x; i.x < maxp.x; ++i.x) {
				const int color_data = voxels.get_voxel(i, channel, defval).i;
				if (color_data != 0) {
					potential_boxes.push_back(AABB(i, Vector3(1, 1, 1)));
				}
			}
		}
	}
}

void collect_boxes(
		const VoxelData &voxels,
		const VoxelMesher &mesher,
		const AABB query_box,
		const uint32_t collision_nask,
		StdVector<AABB> &potential_boxes
) {
	ZN_PROFILE_SCOPE();

	const Vector3i minp = math::floor_to_int(query_box.position);
	const Vector3i maxp = math::ceil_to_int(query_box.position + query_box.size);

	const VoxelMesherBlocky *mesher_blocky = Object::cast_to<VoxelMesherBlocky>(&mesher);
	if (mesher_blocky != nullptr) {
		collect_boxes_blocky(voxels, *mesher_blocky, minp, maxp, collision_nask, potential_boxes);
		return;
	}

	const VoxelMesherCubes *mesher_cubes = Object::cast_to<VoxelMesherCubes>(&mesher);
	if (mesher_cubes != nullptr) {
		collect_boxes_cubes(voxels, minp, maxp, potential_boxes);
		return;
	}
}

} // namespace

Vector3 VoxelBoxMover::get_motion(
		const Vector3 pos_world,
		const Vector3 motion_world,
		const AABB aabb_world,
		const VoxelData &terrain_data,
		const Transform3D &terrain_transform,
		const VoxelMesher &mesher
) {
	ZN_PROFILE_SCOPE();

	// Transform to local in case the volume is transformed
	const Transform3D to_world = terrain_transform;
	const Transform3D to_local = to_world.affine_inverse();
	const Vector3 pos = to_local.xform(pos_world);
	const Vector3 motion = to_local.basis.xform(motion_world);
	const AABB aabb = Transform3D(to_local.basis, Vector3()).xform(aabb_world);

	const AABB box(aabb.position + pos, aabb.size);
	const AABB expanded_box = expand_with_vector(box, motion);

	// TODO Candidate for temp allocator
	static thread_local StdVector<AABB> s_colliding_boxes;
	StdVector<AABB> &potential_boxes = s_colliding_boxes;
	potential_boxes.clear();

	// Collect potential collisions with the terrain (broad phase)
	// TODO If motion is really big, we may want something more optimal or reject it
	collect_boxes(terrain_data, mesher, expanded_box, _collision_mask, potential_boxes);

	// Calculate collisions (narrow phase)
	Vector3 slided_motion = zylann::voxel::get_motion(box, motion, to_span(potential_boxes));

	// Minecraft-style stair climbing:
	// If we were moving, changed horizontal direction due to collision, and resulting motion is about horizontal
	_has_stepped_up = false;
	if (_step_climbing_enabled &&
		// Movement is horizontal?
		Math::abs(slided_motion.y) < 0.001 && Vector2(motion.x, motion.z).length_squared() > 0.0001 &&
		// Motor movement isn't the same as resulting slided motion?
		Vector2(motion.x, motion.z).normalized().dot(Vector2(slided_motion.x, slided_motion.z).normalized()) < 0.99) {
		// We hit an obstacle
		real_t hit_y;
		// Find out the height of the step
		if (boxcast_down(to_span(potential_boxes), get_xz(expanded_box.position), get_xz(expanded_box.size), hit_y)) {
			// If the step is up and not too high
			if (hit_y > box.position.y && (hit_y - box.position.y) <= _max_step_height) {
				// Check if we would fit if we move the box above the step.
				// Raise it slightly higher to avoid precision issues. Even if the final motion would move the box
				// exactly on top of the stair, gameplay code could do some additional calculations with that motion
				// (converting it to velocity?) which may induce precision errors causing the box to fall through.
				const real_t epsilon = 0.0001f;
				const AABB hyp_box(
						Vector3(box.position.x + motion.x, hit_y + epsilon, box.position.z + motion.z), box.size
				);

				potential_boxes.clear();
				collect_boxes(terrain_data, mesher, hyp_box, _collision_mask, potential_boxes);

				// If the box fits on top of the step
				if (!zylann::voxel::intersects(to_span(potential_boxes), hyp_box)) {
					// Change motion so that it brings the box on top of the step
					slided_motion = hyp_box.position - box.position;
					_has_stepped_up = true;
				}
			}
		}
	}

	// Switch back to world
	const Vector3 world_slided_motion = to_world.basis.xform(slided_motion);

	return world_slided_motion;
}

void VoxelBoxMover::set_collision_mask(uint32_t mask) {
	_collision_mask = mask;
}

void VoxelBoxMover::set_step_climbing_enabled(bool enable) {
	_step_climbing_enabled = enable;
}

bool VoxelBoxMover::is_step_climbing_enabled() const {
	return _step_climbing_enabled;
}

bool VoxelBoxMover::has_stepped_up() const {
	return _has_stepped_up;
}

void VoxelBoxMover::set_max_step_height(float height) {
	_max_step_height = height;
}

float VoxelBoxMover::get_max_step_height() const {
	return _max_step_height;
}

bool VoxelBoxMover::intersects(
		const AABB aabb_world,
		const VoxelData &terrain_data,
		const Transform3D &terrain_transform,
		const VoxelMesher &mesher
) const {
	// Transform to local in case the volume is transformed
	const Transform3D to_world = terrain_transform;
	const Transform3D to_local = to_world.affine_inverse();
	const AABB aabb = to_local.xform(aabb_world);

	// TODO Candidate for temp allocator
	static thread_local StdVector<AABB> s_colliding_boxes;
	StdVector<AABB> &potential_boxes = s_colliding_boxes;
	potential_boxes.clear();

	// Collect potential collisions with the terrain (broad phase)
	collect_boxes(terrain_data, mesher, aabb, _collision_mask, potential_boxes);

	return zylann::voxel::intersects(to_span(potential_boxes), aabb);
}

#if defined(ZN_GODOT)
Vector3 VoxelBoxMover::_b_get_motion(Vector3 pos, Vector3 motion, AABB aabb, Node *terrain_node) {
#elif defined(ZN_GODOT_EXTENSION)
Vector3 VoxelBoxMover::_b_get_motion(Vector3 pos, Vector3 motion, AABB aabb, Object *terrain_node) {
#endif
	ERR_FAIL_COND_V(terrain_node == nullptr, Vector3());
	VoxelNode *terrain = Object::cast_to<VoxelNode>(terrain_node);
	ERR_FAIL_COND_V(terrain == nullptr, Vector3());

	// The mesher is required to know how collisions should be processed
	Ref<VoxelMesher> mesher = terrain->get_mesher();
	ERR_FAIL_COND_V(mesher.is_null(), Vector3());

	return get_motion(pos, motion, aabb, terrain->get_storage(), terrain->get_global_transform(), **mesher);
}

bool VoxelBoxMover::_b_intersects(AABB p_aabb, Object *p_terrain_node) const {
	ERR_FAIL_COND_V(p_terrain_node == nullptr, false);
	VoxelNode *terrain = Object::cast_to<VoxelNode>(p_terrain_node);
	ERR_FAIL_COND_V(terrain == nullptr, false);

	// The mesher is required to know how collisions should be processed
	Ref<VoxelMesher> mesher = terrain->get_mesher();
	ERR_FAIL_COND_V(mesher.is_null(), false);

	return intersects(p_aabb, terrain->get_storage(), terrain->get_global_transform(), **mesher);
}

void VoxelBoxMover::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_motion", "pos", "motion", "aabb", "terrain"), &VoxelBoxMover::_b_get_motion);
	ClassDB::bind_method(D_METHOD("intersects", "aabb", "terrain"), &VoxelBoxMover::_b_intersects);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &VoxelBoxMover::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &VoxelBoxMover::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_step_climbing_enabled", "enabled"), &VoxelBoxMover::set_step_climbing_enabled);
	ClassDB::bind_method(D_METHOD("is_step_climbing_enabled"), &VoxelBoxMover::is_step_climbing_enabled);

	ClassDB::bind_method(D_METHOD("set_max_step_height", "height"), &VoxelBoxMover::set_max_step_height);
	ClassDB::bind_method(D_METHOD("get_max_step_height"), &VoxelBoxMover::get_max_step_height);

	ClassDB::bind_method(D_METHOD("has_stepped_up"), &VoxelBoxMover::has_stepped_up);
}

} // namespace zylann::voxel
