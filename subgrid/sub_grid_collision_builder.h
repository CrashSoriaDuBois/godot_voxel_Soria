#pragma once
#include "../storage/voxel_buffer.h"
#include "../util/containers/span.h"
#include "core/math/aabb.h"
#include "core/math/vector3.h"
#include "core/templates/hash_map.h"
#include "core/templates/vector.h"
#include <memory>

namespace zylann::voxel {

// Result of one chunk's collision build pass.
// Produced on a worker thread, consumed on the main thread.
struct SubGridCollisionOutput {
	// Greedy-merged axis-aligned boxes in chunk-local voxel space.
	// Each box covers one or more solid external voxels.
	// These become BoxShape3D collision shapes on the physics body.
	Vector<AABB> boxes;

	// Center of mass in chunk-local voxel space, weighted by block density.
	Vector3 center_of_mass;

	// Total mass of all solid voxels in this chunk.
	float total_mass = 0.f;
};

// Block weight table: maps voxel type ID to mass per voxel.
// Missing IDs fall back to default_mass.
struct BlockWeightTable {
	HashMap<uint32_t, float> weights;
	float default_mass = 1.f;

	float get(uint32_t voxel_id) const {
		const float *w = weights.getptr(voxel_id);
		return w ? *w : default_mass;
	}
};

class SubGridCollisionBuilder {
public:
	// Build greedy collision boxes and center of mass for one chunk.
	//
	// `padded_buffer` must be the same padded buffer used for meshing
	// (1 voxel of padding on all sides, same as CHUNK_SIZE_PO2 chunks).
	// `chunk_voxel_origin` is the world-local position of voxel (0,0,0)
	// of the un-padded chunk, used to offset output boxes into subgrid space.
	//
	// Safe to call from a worker thread.
	static SubGridCollisionOutput build(
			const VoxelBuffer &padded_buffer,
			Vector3i chunk_voxel_origin,
			int chunk_size, // 1 << CHUNK_SIZE_PO2
			const BlockWeightTable &weights
	);

private:
	// Returns true if the voxel at `pos` (in un-padded chunk space) is solid
	// and has at least one air-exposed face. i.e. is a candidate for collision.
	static bool is_external_solid(
			const VoxelBuffer &buf,
			int x,
			int y,
			int z,
			int pad // = 1
	);

	// Greedy 3D box merging on a binary solid grid.
	// `solid[x + size*y + size*size*z]` = true if voxel is external solid.
	static void greedy_boxes(const Vector<bool> &solid, int size, Vector3i origin, Vector<AABB> &out_boxes);
};

} // namespace zylann::voxel