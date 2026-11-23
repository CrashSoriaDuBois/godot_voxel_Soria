#ifndef VOXEL_BLOCKY_TILE_ORIENTATION_H
#define VOXEL_BLOCKY_TILE_ORIENTATION_H

#include "../../constants/cube_tables.h"
#include "../../util/math/ortho_basis.h"
#include "../../util/math/vector3i.h"

#include <array>
#include <cstdint>

namespace zylann::voxel::blocky {

// Orientation of the texture of a tile on each side of a (non-rotated) voxel, in the frame of the voxel.
// `u` is where the tile's X axis points, `v` where its Y axis points (tiles are Y-down). `normal` is the direction of
// the side.
// This is what the mesher assumes when it looks for neighbors of connected tiles or extends tiles, and it comes from
// the UVs of cube sides defined in the cube tables.
struct SideTileAxes {
	Vector3i normal;
	Vector3i u;
	Vector3i v;
};

const SideTileAxes &get_side_tile_axes(const unsigned int side);

// When a model is rotated, the texture of its faces rotates with them, so on the side where a face ends up, the tile
// may no longer have the orientation the mesher assumes for that side. This tells, for each side of a model rotated
// with `basis`, how many quarter turns the orientation of the tile is rotated compared to the standard one:
// - 0: (u, v)
// - 1: (v, -u)
// - 2: (-u, -v)
// - 3: (-v, u)
// where (u, v) are the standard axes of that side (see SideTileAxes).
void get_side_tile_rotations(const math::OrthoBasis &basis, std::array<uint8_t, Cube::SIDE_COUNT> &out_rotations);

} // namespace zylann::voxel::blocky

#endif // VOXEL_BLOCKY_TILE_ORIENTATION_H
