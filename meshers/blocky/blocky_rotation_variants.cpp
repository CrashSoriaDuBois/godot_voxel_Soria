#include "blocky_rotation_variants.h"
#include "../../util/math/ortho_basis.h"
#include "../../util/math/vector3i.h"

namespace zylann::voxel::blocky {

namespace {

// Orthogonal rotations (math::OrthoRotationID) of each variant, in order.
// Y is up and -Z is forward.
// 6 faces: forward, look up (X 90), turn around (Y 180), look down (X 270), turn right (Y 270), turn left (Y 90)
const uint8_t g_faces_ortho_indices[6] = { 0, 4, 10, 12, 16, 22};
// 3 axes: forward, look down, turn right
const uint8_t g_axes_ortho_indices[3] = { 0, 4, 16 };

// Direction where the forward (-Z) axis of the model points after a rotation. The Z axis of the basis is where +Z
// ends up.
Vector3i get_forward_after(const unsigned int ortho_index) {
	const Vector3i z = math::get_ortho_basis_from_index(ortho_index).z;
	return Vector3i(-z.x, -z.y, -z.z);
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
			return variant < 3 ? g_axes_ortho_indices[variant] : math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
		case ROTATION_MODE_FACES_6:
			return variant < 6 ? g_faces_ortho_indices[variant] : math::ORTHOGONAL_BASIS_IDENTITY_INDEX;
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
			// Variant where the forward direction ends up at the same place
			const Vector3i forward = get_forward_after(ortho_index);
			for (unsigned int v = 0; v < 6; ++v) {
				if (get_forward_after(g_faces_ortho_indices[v]) == forward) {
					return v;
				}
			}
			return 0;
		}

		case ROTATION_MODE_AXES_3: {
			// Variant where the forward direction ends up on the same axis (the sign is ignored)
			const Vector3i forward = get_forward_after(ortho_index);
			for (unsigned int v = 0; v < 3; ++v) {
				const Vector3i other = get_forward_after(g_axes_ortho_indices[v]);
				if ((forward.x != 0) == (other.x != 0) && (forward.y != 0) == (other.y != 0) &&
					(forward.z != 0) == (other.z != 0)) {
					return v;
				}
			}
			return 0;
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

uint32_t rotate_voxel_id(const BakedLibrary &lib, const uint32_t id, const math::OrthoBasis &rotation) {
	if (!lib.has_model(id)) {
		return id;
	}
	const BakedModel &m = lib.models[id];
	if (m.rotation_mode == ROTATION_MODE_NONE) {
		return id;
	}
	const unsigned int count = get_rotation_variant_count(m.rotation_mode);
	if (m.rotation_variant >= count || id < m.rotation_variant) {
		return id;
	}
	const uint32_t base = id - m.rotation_variant;
	// The group must be laid out as the header describes, otherwise leave the voxel alone
	if (!lib.has_model(base + count - 1) || lib.models[base].rotation_mode != m.rotation_mode ||
		lib.models[base].rotation_variant != 0) {
		return id;
	}

	const math::OrthoBasis current =
			math::get_ortho_basis_from_index(get_rotation_variant_ortho_index(m.rotation_mode, m.rotation_variant));
	const math::OrthoBasis rotated = rotation * current;
	const int rotated_index = math::get_index_from_ortho_basis(rotated);
	if (rotated_index < 0) {
		return id; // not a proper rotation (mirrored basis): don't guess
	}
	return base + get_rotation_variant_from_ortho_index(m.rotation_mode, static_cast<unsigned int>(rotated_index));
}

} // namespace zylann::voxel::blocky
