#ifndef VOXEL_BLOCKY_ROTATION_VARIANTS_H
#define VOXEL_BLOCKY_ROTATION_VARIANTS_H

#include "blocky_baked_library.h"
#include "../../util/math/ortho_basis.h"

namespace zylann::voxel::blocky {

// A "rotation group" is a model followed by rotated copies of itself, occupying consecutive indices in a library.
// Variants always have the same order, so the index of a variant can be deduced from the first index of the group.
// Variant 0 is always the identity (the model itself). Rotations are given as math::OrthoRotationID, with Y up and -Z
// being the forward direction of the model:
//
// - ROTATION_MODE_ORTHO_24: variant N is the orthogonal rotation N.
// - ROTATION_MODE_FACES_6: ortho rotations 0, 16, 10, 22, 4, 12 (forward,look up ,turn around ,lookdown ,turn right ,turn left). The forward direction of the model points at a different face for each.
// - ROTATION_MODE_AXES_3: ortho rotations 0, 4, 16 (one per axis: forward, look down, turn right).

// Number of models a rotation group occupies in a library.
unsigned int get_rotation_variant_count(const RotationMode mode);

// Orthogonal rotation (math::OrthoRotationID) applied to the model for the given variant.
uint8_t get_rotation_variant_ortho_index(const RotationMode mode, const unsigned int variant);

// Variant of a group that best matches an orthogonal rotation. With the 3 and 6 variants modes, only the direction
// where the forward (-Z) axis of the model ends up is considered (the 3 variants mode ignores the sign too).
unsigned int get_rotation_variant_from_ortho_index(const RotationMode mode, const unsigned int ortho_index);

const char *get_rotation_mode_name(const RotationMode mode);

uint32_t rotate_voxel_id(const BakedLibrary &lib, uint32_t voxel_id, const math::OrthoBasis &rotation);

} // namespace zylann::voxel::blocky

#endif // VOXEL_BLOCKY_ROTATION_VARIANTS_H
