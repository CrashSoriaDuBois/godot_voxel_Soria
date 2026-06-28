#include "sub_grid_lod_streaming.h"
#include "../../engine/voxel_engine.h"
#include "../../util/containers/std_vector.h" // StdVector, for Box3i::difference_to_vec
#include "../../util/math/conv.h" // math::floor_to_int
#include "../../util/math/vector3i.h" // math::floordiv / math::ceildiv on Vector3i
#include "../voxel_sub_grid.h" // full VoxelSubGrid definition - sub_grid_manager.h only forward-declares it

namespace zylann::voxel {

namespace {

Box3i get_base_box_in_chunks(
		Vector3i viewer_position_voxels,
		Vector3i distance_voxels,
		int chunk_size,
		bool make_even
) {
	Vector3i minp = viewer_position_voxels - distance_voxels;
	Vector3i maxp = viewer_position_voxels + distance_voxels + Vector3iUtil::create(1);

	minp = math::floordiv(minp, chunk_size);
	maxp = math::ceildiv(maxp, chunk_size);

	if (make_even) {
		minp = math::floordiv(minp, 2) * 2;
		maxp = math::ceildiv(maxp, 2) * 2;
	}

	return Box3i::from_min_max(minp, maxp);
}

Box3i get_minimal_box_for_parent_lod(Box3i child_lod_box, bool make_even) {
	const int min_pad = 1;
	Box3i min_box = Box3i(child_lod_box.position >> 1, child_lod_box.size >> 1).padded(min_pad);

	if (make_even) {
		min_box = min_box.downscaled(2).scaled(2);
	}

	return min_box;
}

Box3i enforce_neighboring_rule(Box3i box, const Box3i &child_lod_box, bool make_even) {
	const Box3i min_box = get_minimal_box_for_parent_lod(child_lod_box, make_even);
	box.merge_with(min_box);
	return box;
}

inline Vector3i get_child_position(Vector3i parent_position, unsigned int child_index) {
	return parent_position * 2 + Vector3i((child_index & 1), ((child_index & 2) >> 1), ((child_index & 4) >> 2));
}

void remove_from_pending(ShipLod &lod, int index) {
	if (index < 0 || index >= lod.pending_update.size()) {
		return;
	}
	const int last_index = lod.pending_update.size() - 1;
	const Vector3i moved = lod.pending_update[last_index];
	lod.pending_update.write[index] = moved;
	lod.pending_update.resize(last_index);

	if (index < lod.pending_update.size()) {
		ChunkMeshBlockState *moved_block = lod.mesh_state.getptr(moved);
		if (moved_block != nullptr) {
			moved_block->update_list_index = index;
		}
	}
}

} // anonymous namespace

void schedule_chunk_remesh(ShipLod &lod, Vector3i bpos) {
	ChunkMeshBlockState *block = lod.mesh_state.getptr(bpos);
	if (block == nullptr) {
		// Not currently viewed by anyone - see this function's header comment.
		return;
	}
	if (block->update_list_index != -1) {
		// Already queued this frame (or pending from a previous one).
		return;
	}
	block->update_list_index = lod.pending_update.size();
	lod.pending_update.push_back(bpos);
	block->state = MESH_UPDATE_NOT_SENT;
}

namespace {

void view_one_chunk(Vector3i bpos, ShipLod &lod) {
	ChunkMeshBlockState &block = lod.mesh_state[bpos]; // get-or-default-insert

	const bool first_viewer = (block.mesh_viewers == 0);
	block.mesh_viewers += 1;

	if (first_viewer) {
		schedule_chunk_remesh(lod, bpos);
	}
}

void view_mesh_box(const Box3i box_to_add, ShipLod &lod, const SubGridChunkMap &chunk_map, int lod_index) {
	const HashSet<Vector3i> &existing = chunk_map.get_lod_chunk_positions(lod_index);

	box_to_add.for_each_cell([&lod, &existing](Vector3i bpos) {
		if (!existing.has(bpos)) {
			// Sparse ship: nothing here to mesh.
			return;
		}
		view_one_chunk(bpos, lod);
	});
}

void unview_mesh_box(const Box3i out_of_range_box, SubGridManager::ShipState &ship, int lod_index, int lod_count) {
	ShipLod &lod = ship.lods[lod_index];

	out_of_range_box.for_each_cell([&lod](Vector3i bpos) {
		ChunkMeshBlockState *block = lod.mesh_state.getptr(bpos);
		if (block == nullptr) {
			return;
		}

		block->mesh_viewers -= 1;
		if (block->mesh_viewers > 0) {
			// Still needed by another viewer.
			return;
		}

		if (block->update_list_index != -1) {
			remove_from_pending(lod, block->update_list_index);
		}
		// If this chunk was active, pair it with a deactivate event before erasing, anything
		// counting activate/deactivate pairs (render visibility toggling, the collision-LOD safety counter) needs every active chunk to eventually get a matching deactivate,
		// even when it goes straight from active to gone instead of via the normal update_mesh_block_load() path.
		if (block->visual_active) {
			lod.to_deactivate_visuals.push_back(bpos);
		}
		// `block` is invalidated by this erase, don't touch it after this line.
		lod.mesh_state.erase(bpos);
		lod.to_unload.push_back(bpos);
	});

	//half of the subdivision rule in unview_mesh_box(). The other half, hide the parent only once
	// children are actually loaded, runs separately, in update_mesh_block_load() below,triggered from process_loaded_chunk_events().
	const int parent_lod_index = lod_index + 1;
	if (parent_lod_index >= lod_count) {
		return;
	}

	const Box3i parent_box(out_of_range_box.position >> 1, out_of_range_box.size >> 1);
	ShipLod &parent_lod = ship.lods[parent_lod_index];

	parent_box.for_each_cell([&lod, &parent_lod](Vector3i bpos) {
		ChunkMeshBlockState *parent_block = parent_lod.mesh_state.getptr(bpos);
		if (parent_block == nullptr || parent_block->visual_active) {
			return;
		}

		// Only re-show the parent if children were ACTUALLY removed (refcount hit zero), not
		// just because the viewer that triggered this unview no longer needs them while a
		// different viewer still does.
		const Vector3i child0 = bpos << 1;
		const ChunkMeshBlockState *child_block = lod.mesh_state.getptr(child0);
		if (child_block == nullptr || child_block->mesh_viewers == 0) {
			parent_block->visual_active = true;
			parent_lod.to_activate_visuals.push_back(bpos);
		}
	});
}

void update_mesh_block_load(
		SubGridManager::ShipState &ship,
		const SubGridChunkMap &chunk_map,
		Vector3i bpos,
		int lod_index,
		int lod_count
) {
	ShipLod &lod = ship.lods[lod_index];
	ChunkMeshBlockState *block = lod.mesh_state.getptr(bpos);
	if (block == nullptr || !block->visual_loaded) {
		return;
	}

	const int parent_lod_index = lod_index + 1;
	if (parent_lod_index >= lod_count) {
		// Root LOD: no parent to coordinate with.
		if (!block->visual_active) {
			block->visual_active = true;
			lod.to_activate_visuals.push_back(bpos);
		}

		if (lod_index > 0) {
			for (unsigned int c = 0; c < 8; ++c) {
				update_mesh_block_load(ship, chunk_map, get_child_position(bpos, c), lod_index - 1, lod_count);
			}
		}
		return;
	}

	const Vector3i parent_bpos = bpos >> 1;
	ShipLod &parent_lod = ship.lods[parent_lod_index];
	ChunkMeshBlockState *parent_block = parent_lod.mesh_state.getptr(parent_bpos);

	if (parent_block == nullptr) {
		// DEPARTURE from upstream - see function comment.
		if (!block->visual_active) {
			block->visual_active = true;
			lod.to_activate_visuals.push_back(bpos);
		}

		if (lod_index > 0) {
			for (unsigned int c = 0; c < 8; ++c) {
				update_mesh_block_load(ship, chunk_map, get_child_position(bpos, c), lod_index - 1, lod_count);
			}
		}
		return;
	}

	if (!parent_block->visual_active) {
		// Children already won this spot.
		return;
	}

	bool all_siblings_ready = true;
	for (unsigned int c = 0; c < 8; ++c) {
		const Vector3i sibling_bpos = get_child_position(parent_bpos, c);
		const ChunkMeshBlockState *sibling = lod.mesh_state.getptr(sibling_bpos);

		if (sibling != nullptr && sibling->visual_loaded) {
			continue;
		}
		if (sibling == nullptr && !chunk_map.get_lod_chunk_positions(lod_index).has(sibling_bpos)) {
			// DEPARTURE from upstream - see function comment: no geometry there, count as ready.
			continue;
		}
		all_siblings_ready = false;
		break;
	}
	if (!all_siblings_ready) {
		return;
	}

	parent_block->visual_active = false;
	parent_lod.to_deactivate_visuals.push_back(parent_bpos);

	for (unsigned int c = 0; c < 8; ++c) {
		const Vector3i sibling_bpos = get_child_position(parent_bpos, c);
		ChunkMeshBlockState *sibling = lod.mesh_state.getptr(sibling_bpos);
		if (sibling == nullptr) {
			// Sparse: nothing here.
			continue;
		}

		if (!sibling->visual_active) {
			sibling->visual_active = true;
			lod.to_activate_visuals.push_back(sibling_bpos);
		}

		if (lod_index > 0) {
			for (unsigned int cc = 0; cc < 8; ++cc) {
				update_mesh_block_load(ship, chunk_map, get_child_position(sibling_bpos, cc), lod_index - 1, lod_count);
			}
		}
	}
}

bool get_engine_viewer(ViewerID id, VoxelEngine::Viewer &out_viewer) {
	bool found = false;
	VoxelEngine::Viewer result;
	VoxelEngine::get_singleton().for_each_viewer([&](ViewerID vid, const VoxelEngine::Viewer &viewer) {
		if (vid == id) {
			result = viewer;
			found = true;
		}
	});
	if (found) {
		out_viewer = result;
	}
	return found;
}

bool find_paired_viewer(const Vector<PairedShipViewer> &viewers, ViewerID id, int &out_index) {
	for (int i = 0; i < viewers.size(); ++i) {
		if (viewers[i].id == id) {
			out_index = i;
			return true;
		}
	}
	return false;
}
//Two passes over engine viewers (destroyed/out-of-range, then new-or-existing), 
// exactly like upstream, for the same reason: removal has to be interpreted
// as "box went empty" so the box-diff step below correctly unviews everything, before the
// viewer slot itself disappears in remove_unpaired_viewers().
void pair_ship_viewers(SubGridManager::ShipState &state, Span<const float> lod_distances, float pairing_distance) {
	const int lod_count = static_cast<int>(lod_distances.size());
	ERR_FAIL_COND_MSG(
			lod_count < 1 || lod_count > SUBGRID_MAX_LODS, "lod_distances size must be between 1 and SUBGRID_MAX_LODS"
	);
	ERR_FAIL_COND(state.node == nullptr);

	const Vector3 ship_world_pos = state.node->get_global_position();

	// --- Pass 1: viewers that fell out of range. MIRROR of the "Destroyed viewers" loop,
	// generalized from "engine viewer no longer exists" to "engine viewer no longer in range".
	for (int i = 0; i < state.paired_viewers.size(); ++i) {
		PairedShipViewer &pv = state.paired_viewers.write[i];

		VoxelEngine::Viewer viewer;
		bool still_in_range = false;
		if (get_engine_viewer(pv.id, viewer)) {
			still_in_range = ship_world_pos.distance_to(viewer.world_position) < pairing_distance;
		}

		if (!still_in_range) {
			pv.prev_state = pv.state;
			for (int lod = 0; lod < lod_count; ++lod) {
				pv.state.mesh_box_per_lod[lod] = Box3i();
			}
		}
	}

	// --- Pass 2: new viewers entering range. MIRROR of the "New viewer" branch.
	VoxelEngine::get_singleton().for_each_viewer([&](ViewerID id, const VoxelEngine::Viewer &viewer) {
		const float dist = ship_world_pos.distance_to(viewer.world_position);
		if (dist >= pairing_distance) {
			return;
		}
		int existing_index;
		if (!find_paired_viewer(state.paired_viewers, id, existing_index)) {
			PairedShipViewer pv;
			pv.id = id;
			state.paired_viewers.push_back(pv);
		}
	});

	// --- Pass 3: refresh box per surviving/new viewer. This is what replaces
	// _compute_desired_lod_chunks's single-distance-band-for-the-whole-ship logic.
	const Transform3D world_to_local = state.node->get_global_transform().affine_inverse();
	const int chunk_size = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;

	for (int i = 0; i < state.paired_viewers.size(); ++i) {
		PairedShipViewer &pv = state.paired_viewers.write[i];

		VoxelEngine::Viewer viewer;
		if (!get_engine_viewer(pv.id, viewer)) {
			// About to be dropped by remove_unpaired_viewers() after this function returns.
			continue;
		}

		pv.prev_state = pv.state;
		pv.state.local_position_voxels = math::floor_to_int(world_to_local.xform(viewer.world_position));

		for (int lod = 0; lod < lod_count; ++lod) {
			// MIRROR: root LOD needn't be even (no parent to subdivide into).
			const bool make_even = (lod != lod_count - 1);
			const int lod_chunk_size = chunk_size << lod;
			const int distance_voxels = (int)lod_distances[lod];

			Box3i box = get_base_box_in_chunks(
					pv.state.local_position_voxels, Vector3iUtil::create(distance_voxels), lod_chunk_size, make_even
			);

			if (lod > 0) {
				box = enforce_neighboring_rule(box, pv.state.mesh_box_per_lod[lod - 1], make_even);
			}

			pv.state.mesh_box_per_lod[lod] = box;
		}
	}
}

//Run after the box diff, for the ordering reason noted
// in pair_ship_viewers()'s comment.
void remove_unpaired_viewers(SubGridManager::ShipState &state) {
	for (int i = state.paired_viewers.size() - 1; i >= 0; --i) {
		VoxelEngine::Viewer viewer;
		if (!get_engine_viewer(state.paired_viewers[i].id, viewer)) {
			const int last_index = state.paired_viewers.size() - 1;
			state.paired_viewers.write[i] = state.paired_viewers[last_index];
			state.paired_viewers.resize(last_index);
		}
	}
}

void process_mesh_boxes(SubGridManager::ShipState &state, const SubGridChunkMap &chunk_map, int lod_count) {
	for (int vi = 0; vi < state.paired_viewers.size(); ++vi) {
		const PairedShipViewer &pv = state.paired_viewers[vi];

		// Iterating from big to small LOD, same order upstream uses, so a future early-exit
		// on "box doesn't intersect ship bounds at all" can be added the same way upstream
		// does it, without reordering anything else.
		for (int lod_index = lod_count - 1; lod_index >= 0; --lod_index) {
			const Box3i &new_box = pv.state.mesh_box_per_lod[lod_index];
			const Box3i &prev_box = pv.prev_state.mesh_box_per_lod[lod_index];
			if (new_box == prev_box) {
				continue;
			}

			ShipLod &lod = state.lods[lod_index];

			StdVector<Box3i> added;
			new_box.difference_to_vec(prev_box, added);
			for (const Box3i &b : added) {
				view_mesh_box(b, lod, chunk_map, lod_index);
			}

			StdVector<Box3i> removed;
			prev_box.difference_to_vec(new_box, removed);
			for (const Box3i &b : removed) {
				unview_mesh_box(b, state, lod_index, lod_count);
			}
		}
	}
}

// MIRROR of process_loaded_mesh_blocks_trigger_visibility_changes(), simplified: no mutex
// (see LoadedChunkEvent's comment), and only a "visual" event kind exists here (no separate
// collision event - see ChunkMeshBlockState's comment on why).
void process_loaded_chunk_events(SubGridManager::ShipState &state, const SubGridChunkMap &chunk_map, int lod_count) {
	for (int i = 0; i < state.pending_loaded_chunks.size(); ++i) {
		const LoadedChunkEvent &ev = state.pending_loaded_chunks[i];
		update_mesh_block_load(state, chunk_map, ev.position, ev.lod_index, lod_count);
	}
	state.pending_loaded_chunks.clear();
}

} // anonymous namespace

// MIRROR of nothing upstream - this closes a gap specific to sparse, mutable ships that
// VLT's dense, append-only-at-the-edges terrain never has: a voxel edit can create a brand
// new chunk inside a region a viewer's box already covers, without the box itself changing.
// process_mesh_boxes only discovers chunks via box.difference_to_vec(prev_box) - if the box
// is identical to last frame, nothing gets re-scanned, so a freshly created chunk would
// otherwise never get a ChunkMeshBlockState at all until something perturbs some viewer's
// box enough to re-diff over that exact cell (which is what made this look like it could be
// "fixed" by changing LOD and going back - that's just incidentally re-running the box diff
// over the same area). Call this from SubGridManager::mark_chunk_dirty for every LOD0/LOD1+
// position a voxel edit touches, in addition to (not instead of) schedule_chunk_remesh.
void notify_chunk_edited(SubGridManager::ShipState &state, Vector3i bpos, int lod_index) {
	ShipLod &lod = state.lods[lod_index];

	if (lod.mesh_state.getptr(bpos) == nullptr) {
		// Not tracked yet - check whether any currently-paired viewer's box already covers
		// this position (true for essentially every real edit, since editing requires being
		// near the chunk in the first place) and, if so, view it exactly as if the box-diff
		// had discovered it normally - including the proper per-viewer refcounting, so a
		// later unview from any one of them doesn't erase it while another still needs it.
		for (int i = 0; i < state.paired_viewers.size(); ++i) {
			const PairedShipViewer &pv = state.paired_viewers[i];
			if (pv.state.mesh_box_per_lod[lod_index].contains(bpos)) {
				view_one_chunk(bpos, lod);
			}
		}
	}

	// Either this just got tracked above (view_one_chunk already scheduled it on first view),
	// or it was already tracked from before (e.g. re-editing an existing chunk) and needs an
	// explicit remesh trigger here. schedule_chunk_remesh's update_list_index guard makes
	// calling it after view_one_chunk in the first case a harmless no-op, not a double-queue.
	schedule_chunk_remesh(lod, bpos);
}

void process_ship_lod_streaming(
		SubGridManager::ShipState &state,
		const SubGridChunkMap &chunk_map,
		Span<const float> lod_distances,
		float viewer_pairing_distance
) {
	if (state.node == nullptr) {
		return;
	}

	// lod_distances.size() IS the effective LOD count for this call, see this function's header comment in sub_grid_lod_streaming.h.
	const int lod_count = static_cast<int>(lod_distances.size());

	// pair viewers -> diff boxes -> drop unpaired viewers -> trigger visibility from already-loaded chunks. The
	// one upstream step genuinely absent here is data-block load/unload (process_data_blocks_sliding_box), 
	// ships have no separate data-streaming layer to drive, since SubGridChunkMap is always fully resident once LOADED.
	pair_ship_viewers(state, lod_distances, viewer_pairing_distance);
	process_mesh_boxes(state, chunk_map, lod_count);
	remove_unpaired_viewers(state);
	process_loaded_chunk_events(state, chunk_map, lod_count);
}

} // namespace zylann::voxel