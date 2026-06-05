#include "sub_grid_collision_builder.h"

namespace zylann::voxel {

// ____________________________________________________________________________
// External-solid check
//
// A voxel is "external" if it is solid AND at least one cardinal neighbor
// is air. We only want surface voxels in the collision shape, not buried ones.
// This mirrors the face-visibility test in VoxelMesherBlocky::generate_mesh.

bool SubGridCollisionBuilder::is_external_solid(const VoxelBuffer &buf, int x, int y, int z, int pad) {
	// All coordinates are in un-padded chunk space (0..chunk_size-1).
	// Translate to padded buffer space by adding `pad`.
	const int px = x + pad;
	const int py = y + pad;
	const int pz = z + pad;

	const uint32_t v = buf.get_voxel(px, py, pz, VoxelBuffer::CHANNEL_TYPE);
	if (v == 0) {
		return false;
	}

	// Check 6 cardinal neighbors in padded space
	const int dx[6] = { 1, -1, 0, 0, 0, 0 };
	const int dy[6] = { 0, 0, 1, -1, 0, 0 };
	const int dz[6] = { 0, 0, 0, 0, 1, -1 };

	for (int i = 0; i < 6; i++) {
		const uint32_t n = buf.get_voxel(px + dx[i], py + dy[i], pz + dz[i], VoxelBuffer::CHANNEL_TYPE);
		if (n == 0) {
			return true; // at least one air neighbor → external
		}
	}
	return false; // fully buried
}

// ____________________________________________________________________________
// Greedy 3D box merging
//
// Classic algorithm: sweep over Z slabs, within each slab sweep Y rows to
// find maximal rectangles, then extend those rectangles in Z as far as
// identical solid coverage allows.
//
// Complexity: O(size^3) time, O(size^2) space.
// For a 16^3 chunk this is 4096 iterations. negligible on a worker thread.

void SubGridCollisionBuilder::greedy_boxes(
		const Vector<bool> &solid,
		int size,
		Vector3i origin,
		Vector<AABB> &out_boxes
) {
	// `merged[x + size*y + size*size*z]` = true means already consumed.
	Vector<bool> merged;
	merged.resize(size * size * size);

	for (int i = 0; i < merged.size(); ++i) {
		merged.write[i] = false;
	}

	auto idx = [&](int x, int y, int z) { return x + size * y + size * size * z; };
	auto is_solid_free = [&](int x, int y, int z) -> bool {
		if (x < 0 || y < 0 || z < 0 || x >= size || y >= size || z >= size) {
			return false;
		}
		return solid[idx(x, y, z)] && !merged[idx(x, y, z)];
	};

	for (int z = 0; z < size; z++) {
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				if (!is_solid_free(x, y, z)) {
					continue;
				}

				// --- Extend in X ---
				int w = 1;
				while (x + w < size && is_solid_free(x + w, y, z)) {
					w++;
				}

				// --- Extend in Y: find max height where all X columns are solid/free ---
				int h = 1;
				while (y + h < size) {
					bool row_ok = true;
					for (int dx = 0; dx < w; dx++) {
						if (!is_solid_free(x + dx, y + h, z)) {
							row_ok = false;
							break;
						}
					}
					if (!row_ok) {
						break;
					}
					h++;
				}

				// --- Extend in Z: find max depth where all X*Y slab is solid/free ---
				int d = 1;
				while (z + d < size) {
					bool slab_ok = true;
					for (int dy = 0; dy < h && slab_ok; dy++) {
						for (int dx = 0; dx < w && slab_ok; dx++) {
							if (!is_solid_free(x + dx, y + dy, z + d)) {
								slab_ok = false;
							}
						}
					}
					if (!slab_ok) {
						break;
					}
					d++;
				}

				// Mark consumed
				for (int dz = 0; dz < d; dz++) {
					for (int dy = 0; dy < h; dy++) {
						for (int dx = 0; dx < w; dx++) {
							merged.write[idx(x + dx, y + dy, z + dz)] = true;
						}
					}
				}

				// Emit box in subgrid-local voxel space
				AABB box(Vector3(origin.x + x, origin.y + y, origin.z + z), Vector3(w, h, d));
				out_boxes.push_back(box);
			}
		}
	}
}

// ____________________________________________________________________________
// Main entry point

SubGridCollisionOutput SubGridCollisionBuilder::build(
		const VoxelBuffer &padded_buffer,
		Vector3i chunk_voxel_origin,
		int chunk_size,
		const BlockWeightTable &weights
) {
	SubGridCollisionOutput output;

	const int pad = 1;

	// --- Pass 1: per-voxel center of mass and total mass ---
	//
	// We do this before greedy merging because different block types
	// may have different masses. After merging we can't distinguish them.
	// CoM = sum(pos_i * mass_i) / total_mass

	Vector3 weighted_pos_sum;
	float total_mass = 0.f;

	for (int z = 0; z < chunk_size; z++) {
		for (int y = 0; y < chunk_size; y++) {
			for (int x = 0; x < chunk_size; x++) {
				const uint32_t v = padded_buffer.get_voxel(x + pad, y + pad, z + pad, VoxelBuffer::CHANNEL_TYPE);
				if (v == 0) {
					continue;
				}
				const float mass = weights.get(v);
				// Voxel center in subgrid-local space
				const Vector3 voxel_center(
						chunk_voxel_origin.x + x + 0.5f,
						chunk_voxel_origin.y + y + 0.5f,
						chunk_voxel_origin.z + z + 0.5f
				);
				weighted_pos_sum += voxel_center * mass;
				total_mass += mass;
			}
		}
	}

	output.total_mass = total_mass;
	if (total_mass > 0.f) {
		output.center_of_mass = weighted_pos_sum / total_mass;
	}

	// --- Pass 2: build external-solid grid ---
	//
	// Only surface voxels need collision shapes. Buried voxels contribute
	// to mass but not to the physics shape.

	Vector<bool> external_solid;
	external_solid.resize(chunk_size * chunk_size * chunk_size);

	for (int i = 0; i < external_solid.size(); ++i) {
		external_solid.write[i] = false;
	}

	for (int z = 0; z < chunk_size; z++) {
		for (int y = 0; y < chunk_size; y++) {
			for (int x = 0; x < chunk_size; x++) {
				external_solid.write[x + chunk_size * y + chunk_size * chunk_size * z] =
						is_external_solid(padded_buffer, x, y, z, pad);
			}
		}
	}

	// --- Pass 3: greedy box merging ---

	greedy_boxes(external_solid, chunk_size, chunk_voxel_origin, output.boxes);

	return output;
}

} // namespace zylann::voxel