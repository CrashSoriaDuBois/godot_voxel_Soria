#pragma once
#include "core/io/resource.h"
#include "core/math/quaternion.h"
#include "core/math/vector3.h"
#include "core/math/vector3i.h"
#include "core/templates/vector.h"
#include "core/variant/variant.h"

namespace zylann::voxel {

struct SubGridMetadata {
	enum LockMode : uint8_t { LOCKED_ALWAYS = 0, LOCKED_DEFAULT = 1, UNLOCKED_DEFAULT = 2, UNLOCKED_ALWAYS = 3 };

	uint8_t uuid[16] = {};
	uint8_t parent_uuid[16] = {};
	Vector3i pivot_in_parent_local;
	Vector3i rotation_axis = Vector3i(0, 1, 0);
	double target_angle_rad = 0.0;
	bool is_root = false;
	bool is_terrain_anchored = false;
	LockMode lock_mode = LOCKED_DEFAULT;
	Vector3 world_position;
	Quaternion world_rotation;
	Vector<Vector3i> chunk_positions;

	Vector3 promoted_pivot_world; // only meaningful for terrain-anchored roots

	PackedByteArray serialize() const;
	static SubGridMetadata deserialize(const PackedByteArray &bytes);
};

} // namespace zylann::voxel