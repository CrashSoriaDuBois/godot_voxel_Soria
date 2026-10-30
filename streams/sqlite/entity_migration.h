#ifndef VOXEL_STREAM_SQLITE_ENTITY_MIGRATION_H
#define VOXEL_STREAM_SQLITE_ENTITY_MIGRATION_H

#include "../../util/containers/std_unordered_map.h"
#include "../../util/containers/std_vector.h"
#include "entity_types.h"
#include "voxel_stream_sqlite.h"

namespace zylann::voxel::sqlite {

// `src` and `dst` must be different databases.
// Not atomic across two files: the destination is written first, so a failure leaves a duplicate, never a loss.
// On success:
// - out_dst_values: the CHANNEL_DATA6 value to write at every destination voxel that now holds an entity.
// - out_src_promotions: source voxels that became roots because their parent moved away. If the source world
//   stays alive, rewrite their channel value with entity_channel_value(action_type, false).
inline bool migrate_entities(
		Ref<VoxelStreamSQLite> src,
		Ref<VoxelStreamSQLite> dst,
		Span<const EntityMove> moves,
		StdVector<EntityChannelWrite> &out_dst_values,
		StdVector<EntityPromotion> &out_src_promotions
) {
	out_dst_values.clear();
	out_src_promotions.clear();
	if (moves.size() == 0) {
		return true;
	}
	if (src.is_null() || dst.is_null()) {
		return false;
	}

	// 1. Read the source rows.
	StdVector<Vector3i> src_voxels;
	for (size_t i = 0; i < moves.size(); ++i) {
		src_voxels.push_back(moves[i].src);
	}
	StdVector<EntityRow> rows;
	if (!src->read_entity_rows(Span<const Vector3i>(src_voxels.data(), src_voxels.size()), rows)) {
		return false;
	}

	// 2. Group the moved voxels by their source root.
	struct Group {
		int action_type = 0;
		StdVector<uint8_t> data;
		StdVector<size_t> members; // indexes into `moves`
		bool has_root_member = false;
	};
	StdVector<Group> groups;
	StdUnorderedMap<Vector3i, size_t> group_index;

	for (size_t i = 0; i < moves.size(); ++i) {
		if (!rows[i].exists) {
			continue;
		}
		auto it = group_index.find(rows[i].root);
		if (it == group_index.end()) {
			Group g;
			g.action_type = rows[i].action_type;
			g.data = rows[i].data; // effective payload, children report their root's
			groups.push_back(std::move(g));
			it = group_index.emplace(rows[i].root, groups.size() - 1).first;
		}
		Group &g = groups[it->second];
		g.members.push_back(i);
		if (!rows[i].is_child) {
			g.has_root_member = true;
		}
	}
	if (groups.empty()) {
		return true;
	}

	// 3. Build the destination groups. If the source root didn't move, the first moved voxel becomes the new root.
	StdVector<EntityGroup> dst_groups;
	StdVector<Vector3i> dst_voxels;
	StdVector<EntityChannelWrite> dst_values;
	for (const Group &g : groups) {
		size_t root_member = g.members[0];
		if (g.has_root_member) {
			for (size_t m : g.members) {
				if (!rows[m].is_child) {
					root_member = m;
					break;
				}
			}
		}
		EntityGroup eg;
		eg.root = moves[root_member].dst;
		eg.action_type = g.action_type;
		eg.data = g.data;
		dst_voxels.push_back(eg.root);
		dst_values.push_back({ eg.root, entity_channel_value(g.action_type) });
		for (size_t m : g.members) {
			if (m == root_member) {
				continue;
			}
			eg.children.push_back(moves[m].dst);
			dst_voxels.push_back(moves[m].dst);
			dst_values.push_back({ moves[m].dst, entity_channel_value(g.action_type) });
		}
		dst_groups.push_back(std::move(eg));
	}

	// 4. The destination voxels receive moved blocks, so any row still sitting there is stale. Clear it first.
	{
		StdVector<EntityPromotion> ignored;
		dst->remove_entity_voxels(Span<const Vector3i>(dst_voxels.data(), dst_voxels.size()), ignored);
	}

	// 5. Write the destination first.
	if (!dst->add_entity_groups(Span<const EntityGroup>(dst_groups.data(), dst_groups.size()))) {
		return false; // nothing was written anywhere
	}

	// 6. Then remove from the source. If that fails, take the copies back out of the destination.
	StdVector<Vector3i> to_remove;
	for (size_t i = 0; i < moves.size(); ++i) {
		if (rows[i].exists) {
			to_remove.push_back(moves[i].src);
		}
	}
	if (!src->remove_entity_voxels(Span<const Vector3i>(to_remove.data(), to_remove.size()), out_src_promotions)) {
		StdVector<EntityPromotion> ignored;
		dst->remove_entity_voxels(Span<const Vector3i>(dst_voxels.data(), dst_voxels.size()), ignored);
		out_src_promotions.clear();
		return false;
	}

	out_dst_values = std::move(dst_values);
	return true;
}

} // namespace zylann::voxel::sqlite

#endif
