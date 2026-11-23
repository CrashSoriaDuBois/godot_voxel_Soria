#include "blocky_tile_orientation.h"
#include "../../util/errors.h"

namespace zylann::voxel::blocky {

const SideTileAxes &get_side_tile_axes(const unsigned int side) {
	// Must match the tile jumps of the mesher (which depend on cube side UVs)
	static const SideTileAxes axes[Cube::SIDE_COUNT] = {
		// Cube::SIDE_POSITIVE_X
		{ Vector3i(1, 0, 0), Vector3i(0, 0, -1), Vector3i(0, -1, 0) },
		// Cube::SIDE_NEGATIVE_X
		{ Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, -1, 0) },
		// Cube::SIDE_NEGATIVE_Y
		{ Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(0, 0, -1) },
		// Cube::SIDE_POSITIVE_Y
		{ Vector3i(0, 1, 0), Vector3i(1, 0, 0), Vector3i(0, 0, 1) },
		// Cube::SIDE_NEGATIVE_Z
		{ Vector3i(0, 0, -1), Vector3i(-1, 0, 0), Vector3i(0, -1, 0) },
		// Cube::SIDE_POSITIVE_Z
		{ Vector3i(0, 0, 1), Vector3i(1, 0, 0), Vector3i(0, -1, 0) },
	};
	ZN_ASSERT_RETURN_V(side < Cube::SIDE_COUNT, axes[0]);
	return axes[side];
}

void get_side_tile_rotations(const math::OrthoBasis &basis, std::array<uint8_t, Cube::SIDE_COUNT> &out_rotations) {
	out_rotations.fill(0);

	for (unsigned int src_side = 0; src_side < Cube::SIDE_COUNT; ++src_side) {
		const SideTileAxes &src = get_side_tile_axes(src_side);

		ZN_ASSERT_CONTINUE_MSG(
				src.normal == Cube::g_side_normals[src_side], "Tile axes don't match the normals of cube sides"
		);

		// Side where the face ends up
		const unsigned int dst_side = Cube::dir_to_side(basis.xform(src.normal));
		const SideTileAxes &dst = get_side_tile_axes(dst_side);

		// Where the X axis of the tile points after rotation, compared to the standard axes of the destination side
		const Vector3i u = basis.xform(src.u);
		uint8_t quarter_turns = 0;
		if (u == dst.u) {
			quarter_turns = 0;
		} else if (u == dst.v) {
			quarter_turns = 1;
		} else if (u == -dst.u) {
			quarter_turns = 2;
		} else if (u == -dst.v) {
			quarter_turns = 3;
		}
		out_rotations[dst_side] = quarter_turns;
	}
}

} // namespace zylann::voxel::blocky
