#ifndef VOXEL_BLOCKY_ROTATION_VARIANTS_H
#define VOXEL_BLOCKY_ROTATION_VARIANTS_H

#include "blocky_baked_library.h"

namespace zylann::voxel::blocky {

// A "rotation group" is a model followed by rotated copies of itself, occupying consecutive indices in a library.
// Variants always have the same order, so the index of a variant can be deduced from the first index of the group:
//
// - ROTATION_MODE_ORTHO_24: variant N is the orthogonal rotation N (see math::OrthoRotationID). 0 is the identity.
// - ROTATION_MODE_FACES_6: the up (+Y) axis of the model points to +Y, -Y, +X, -X, +Z, -Z (in this order).
// - ROTATION_MODE_AXES_3: the up (+Y) axis of the model points to +Y, +X, +Z (in this order).
//
// For the last two, the spin around the up axis is the one of the lowest orthogonal rotation index that gives the
// direction. In every mode, variant 0 is the identity (the model itself).

// Number of models a rotation group occupies in a library.
unsigned int get_rotation_variant_count(const RotationMode mode);

// Orthogonal rotation (math::OrthoRotationID) applied to the model for the given variant.
uint8_t get_rotation_variant_ortho_index(const RotationMode mode, const unsigned int variant);

// Variant of a group that best matches an orthogonal rotation. With the 3 and 6 variants modes, only the direction
// where the up axis of the model ends up is considered.
unsigned int get_rotation_variant_from_ortho_index(const RotationMode mode, const unsigned int ortho_index);

const char *get_rotation_mode_name(const RotationMode mode);

} // namespace zylann::voxel::blocky

#endif // VOXEL_BLOCKY_ROTATION_VARIANTS_H
