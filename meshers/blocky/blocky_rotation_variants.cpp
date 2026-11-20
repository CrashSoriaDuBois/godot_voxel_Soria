#include "blocky_rotation_variants.h"
#include "../../util/math/ortho_basis.h"
#include "../../util/math/vector3i.h"

namespace zylann::voxel::blocky {

namespace {

// Directions where the up (+Y) axis of the model points, for the 6 faces mode
const Vector3i *get_face_directions() {
	static const Vector3i dirs[6] = { Vector3i(0, 1, 0),  Vector3i(0, -1, 0), Vector3i(1, 0, 0),
									  Vector3i(-1, 0, 0), Vector3i(0, 0, 1),  Vector3i(0, 0, -1) };
	return dirs;
}

struct VariantTables {
	uint8_t faces[6];
	uint8_t axes[3];

	VariantTables() {
		const Vector3i *dirs = get_face_directions();
		for (unsigned int d = 0; d < 6; ++d) {
			faces[d] = math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
			// Lowest rotation that sends +Y to the direction. The Y axis of the basis is where +Y ends up.
			for (int i = 0; i < math::ORTHOGONAL_BASIS_COUNT; ++i) {
				if (math::get_ortho_basis_from_index(i).y == dirs[d]) {
					faces[d] = i;
					break;
				}
			}
		}
		axes[0] = faces[0]; // +Y
		axes[1] = faces[2]; // +X
		axes[2] = faces[4]; // +Z
	}
};

const VariantTables &get_tables() {
	static const VariantTables tables;
	return tables;
}

} // namespace

unsigned int get_rotation_variant_count(const RotationMode mode) {
	switch (mode) {
		case ROTATION_MODE_AXES_3:
			return 3;
		case ROTATION_MODE_FACES_6:
			return 6;
		case ROTATION_MODE_ORTHO_24:
			return math::ORTHOGONAL_BASIS_COUNT;
		default:
			return 1;
	}
}

uint8_t get_rotation_variant_ortho_index(const RotationMode mode, const unsigned int variant) {
	switch (mode) {
		case ROTATION_MODE_AXES_3:
			return variant < 3 ? get_tables().axes[variant] : math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
		case ROTATION_MODE_FACES_6:
			return variant < 6 ? get_tables().faces[variant] : math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
		case ROTATION_MODE_ORTHO_24:
			return variant < math::ORTHOGONAL_BASIS_COUNT ? variant : math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
		default:
			return math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
	}
}

unsigned int get_rotation_variant_from_ortho_index(const RotationMode mode, const unsigned int ortho_index) {
	if (ortho_index >= math::ORTHOGONAL_BASIS_COUNT) {
		return 0;
	}
	switch (mode) {
		case ROTATION_MODE_ORTHO_24:
			return ortho_index;

		case ROTATION_MODE_FACES_6: {
			const Vector3i up = math::get_ortho_basis_from_index(ortho_index).y;
			const Vector3i *dirs = get_face_directions();
			for (unsigned int d = 0; d < 6; ++d) {
				if (dirs[d] == up) {
					return d;
				}
			}
			return 0;
		}

		case ROTATION_MODE_AXES_3: {
			const Vector3i up = math::get_ortho_basis_from_index(ortho_index).y;
			if (up.y != 0) {
				return 0;
			}
			if (up.x != 0) {
				return 1;
			}
			return 2;
		}

		default:
			return 0;
	}
}

const char *get_rotation_mode_name(const RotationMode mode) {
	switch (mode) {
		case ROTATION_MODE_AXES_3:
			return "3 axes";
		case ROTATION_MODE_FACES_6:
			return "6 faces";
		case ROTATION_MODE_ORTHO_24:
			return "24 rotations";
		default:
			return "none";
	}
}

} // namespace zylann::voxel::blocky
