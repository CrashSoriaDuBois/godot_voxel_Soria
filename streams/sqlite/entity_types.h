#ifndef VOXEL_STREAM_SQLITE_ENTITY_TYPES_H
#define VOXEL_STREAM_SQLITE_ENTITY_TYPES_H

#include "../../util/containers/std_vector.h"
#include "../../util/math/vector3i.h"

namespace zylann::voxel::sqlite {

// Entities are keyed by the voxel's index inside its 16^3 chunk (12 bits).
constexpr int ENTITY_CHUNK_PO2 = 4;
constexpr int ENTITY_CHUNK_MASK = (1 << ENTITY_CHUNK_PO2) - 1;

inline int entity_index(const Vector3i v) {
	return (v.x & ENTITY_CHUNK_MASK) | ((v.y & ENTITY_CHUNK_MASK) << ENTITY_CHUNK_PO2) |
			((v.z & ENTITY_CHUNK_MASK) << (2 * ENTITY_CHUNK_PO2));
}

inline Vector3i entity_chunk(const Vector3i v) {
	return v >> ENTITY_CHUNK_PO2;
}

inline Vector3i entity_voxel_from(const Vector3i chunk, const int index) {
	return (chunk << ENTITY_CHUNK_PO2) +
			Vector3i(
					index & ENTITY_CHUNK_MASK,
					(index >> ENTITY_CHUNK_PO2) & ENTITY_CHUNK_MASK,
					(index >> (2 * ENTITY_CHUNK_PO2)) & ENTITY_CHUNK_MASK
			);
}

// Value stored in CHANNEL_DATA6: the action type in bits 12..15. Parent/child links live only in the database.
inline uint32_t entity_channel_value(const int action_type) {
	return static_cast<uint32_t>(action_type) << 12;
}

struct EntityRow {
	bool exists = false;
	int action_type = 0;
	bool is_child = false;
	Vector3i root; // own voxel when it is a root, the parent's voxel when it is a child
	StdVector<uint8_t> data; // effective payload (children get their root's payload)
};

struct EntityGroup {
	Vector3i root;
	StdVector<Vector3i> children;
	int action_type = 0;
	StdVector<uint8_t> data;
};

struct EntityPromotion {
	Vector3i voxel; // voxel that became a root
	int action_type = 0;
};

struct EntityMove {
	Vector3i src;
	Vector3i dst;
};

struct EntityChannelWrite {
	Vector3i voxel;
	uint32_t value;
};

} // namespace zylann::voxel::sqlite

#endif
