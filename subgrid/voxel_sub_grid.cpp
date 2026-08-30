#include "voxel_sub_grid.h"
#include "core/io/dir_access.h"
#include "core/math/math_defs.h"
#include "core/object/object.h"
#include "edition/voxel_tool.h"
#include "sub_grid_manager.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "voxel_tool_sub_grid.h"

namespace zylann::voxel {

static constexpr double ZN_TAU = 6.28318530717958647692;

//___________________________________________________________________________
// Godot hooks

void VoxelSubGrid::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_viewer", "viewer"), &VoxelSubGrid::set_viewer);
	ClassDB::bind_method(D_METHOD("destroy"), &VoxelSubGrid::destroy);
	ClassDB::bind_method(D_METHOD("try_disassemble", "terrain"), &VoxelSubGrid::try_disassemble);
	//ClassDB::bind_method(D_METHOD("align_and_disassemble", "rpm", "terrain"), &VoxelSubGrid::align_and_disassemble);

	ClassDB::bind_method(D_METHOD("flush_dirty_chunks"), &VoxelSubGrid::flush_dirty_chunks);
	ClassDB::bind_method(D_METHOD("set_angular_speed_rpm", "rpm"), &VoxelSubGrid::set_angular_speed_rpm);
	ClassDB::bind_method(D_METHOD("set_rotation_axis", "axis"), &VoxelSubGrid::set_rotation_axis);
	ClassDB::bind_method(D_METHOD("get_rotation_axis"), &VoxelSubGrid::get_rotation_axis);
	ClassDB::bind_method(D_METHOD("set_spin_axis", "axis"), &VoxelSubGrid::set_spin_axis);
	ClassDB::bind_method(D_METHOD("get_spin_axis"), &VoxelSubGrid::get_spin_axis);
	ClassDB::bind_method(
			D_METHOD("align_and_disassemble", "rpm", "terrain", "upright_only", "snap speed"),
			&VoxelSubGrid::align_and_disassemble
	);

	ClassDB::bind_method(D_METHOD("is_root"), &VoxelSubGrid::is_root);
	ClassDB::bind_method(D_METHOD("is_tree_grid_aligned", "tolerance_degrees"), &VoxelSubGrid::is_tree_grid_aligned);

	ClassDB::bind_method(D_METHOD("get_target_angle_rad"), &VoxelSubGrid::get_target_angle_rad);

	ClassDB::bind_method(D_METHOD("get_voxel_tool"), &VoxelSubGrid::get_voxel_tool);

	ClassDB::bind_method(D_METHOD("grab", "grab_point_local", "strength"), &VoxelSubGrid::grab, DEFVAL(1.0f));
	ClassDB::bind_method(
			D_METHOD("grab_with_rotation", "grab_point_local", "strength"),
			&VoxelSubGrid::grab_with_rotation,
			DEFVAL(1.0f)
	);
	ClassDB::bind_method(D_METHOD("release"), &VoxelSubGrid::release);
	ClassDB::bind_method(D_METHOD("set_grab_target", "target"), &VoxelSubGrid::set_grab_target);
	ClassDB::bind_method(D_METHOD("apply_impulse", "impulse", "world_point"), &VoxelSubGrid::apply_impulse);
	ClassDB::bind_method(D_METHOD("apply_central_impulse", "impulse"), &VoxelSubGrid::apply_central_impulse);
}

void VoxelSubGrid::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			set_process(true);
			break;

		case NOTIFICATION_EXIT_TREE:
			flush_dirty_chunks();
			_close_stream();
			break;
	}
}

//___________________________________________________________________________
// Lifecycle

void VoxelSubGrid::initialize_root(
		const SubGridMetadata &meta,
		SubGridChunkMap &&chunks,
		const String &saves_dir,
		Ref<VoxelMesherBlocky> mesher,
		Ref<VoxelBlockyLibrary> library,
		const VoxelFormat &format
) {
	_meta = meta;
	_chunks = std::move(chunks);
	_chunks.set_format(format);
	_saves_dir = saves_dir;
	_mesher = mesher;
	_library = library;

	_chunks.rebuild_all_lods();

	String saves_dir_copy = saves_dir;
	uint8_t uuid_copy[16];
	memcpy(uuid_copy, meta.uuid, 16);

	_needs_initial_save = true;
	// chunks are in memory, saved on next natural save cycle

	set_global_position(_meta.world_position);
	set_global_basis(Basis(_meta.world_rotation));
}

void VoxelSubGrid::initialize_root_from_disk(
		const SubGridMetadata &meta,
		const String &saves_dir,
		Ref<VoxelMesherBlocky> mesher,
		Ref<VoxelBlockyLibrary> library,
		const VoxelFormat &format
) {
	_meta = meta;
	_saves_dir = saves_dir;
	_mesher = mesher;
	_library = library;

	// Must happen before load_chunks_from_stream(), since rebuild_all_lods() inside it
	// creates every LOD1-3 buffer using whatever _chunks._format holds right now.
	_chunks.set_format(format);

	_promoted_pivot_world = meta.promoted_pivot_world;
	_is_world_anchored = meta.is_terrain_anchored;

	load_chunks_from_stream();

	print_line(String("initialize_root_from_disk:"));
	print_line(String("  world_position = ") + String(meta.world_position));
	print_line(
			String("  world_rotation = (") + rtos(meta.world_rotation.x) + ", " + rtos(meta.world_rotation.y) + ", " +
			rtos(meta.world_rotation.z) + ", " + rtos(meta.world_rotation.w) + ")"
	);
	print_line(String("  is_terrain_anchored = ") + (meta.is_terrain_anchored ? "true" : "false"));
	print_line(String("  is_root = ") + (meta.is_root ? "true" : "false"));
	print_line(String("  promoted_pivot_world = ") + String(meta.promoted_pivot_world));

	Quaternion q = _meta.world_rotation;
	print_line(String("  quat length_squared = ") + rtos(q.length_squared()));

	if (q.length_squared() < 0.0001f) {
		q = Quaternion();
	} else {
		q = q.normalized();
	}
	set_global_position(_meta.world_position);
	set_global_basis(Basis(q));
}

void VoxelSubGrid::initialize_child(
		const SubGridMetadata &meta,
		SubGridChunkMap &&chunks,
		const String &saves_dir,
		bool async_stream,
		const VoxelFormat &format
) {
	_meta = meta;
	_chunks = std::move(chunks);
	_saves_dir = saves_dir;

	_chunks.set_format(format);

	VoxelSubGrid *parent_sg = Object::cast_to<VoxelSubGrid>(get_parent());
	if (parent_sg != nullptr) {
		memcpy(_meta.parent_uuid, parent_sg->get_metadata().uuid, 16);
		parent_sg->_children.push_back(get_instance_id());
	}

	_chunks.rebuild_all_lods();

	VoxelSubGrid *root = get_root();
	_mesher = root->_mesher;
	_library = root->_library;

	if (async_stream) {
		// New spawn, open async to avoid main thread stutter
		_needs_initial_save = true;
		String saves_dir_copy = saves_dir;
		uint8_t uuid_copy[16];
		memcpy(uuid_copy, meta.uuid, 16);

	}
}

VoxelSubGrid *VoxelSubGrid::get_root() {
	Node *p = get_parent();
	while (p != nullptr) {
		VoxelSubGrid *vsg = Object::cast_to<VoxelSubGrid>(p);
		if (vsg != nullptr) {
			if (vsg->is_root()) {
				return vsg;
			}
			p = vsg->get_parent();
		} else {
			break;
		}
	}
	return this;
}

//___________________________________________________________________________
// Editing

uint32_t VoxelSubGrid::get_voxel(Vector3i local_pos, int channel) const {
	return _chunks.get_voxel(local_pos, channel);
}

void VoxelSubGrid::set_voxel(uint32_t value, Vector3i local_pos, int channel) {
	_chunks.set_voxel(value, local_pos, channel);
}

void VoxelSubGrid::set_redstone_signal(bool powered) {
	_redstone_signal = powered;
}

static String uuid_to_string(const uint8_t *uuid) {
	String s;
	for (int i = 0; i < 16; i++) {
		s += String::num_int64(uuid[i] >> 4, 16);
		s += String::num_int64(uuid[i] & 0xF, 16);
	}
	return s;
}

void VoxelSubGrid::grab(Vector3 grab_point_local, float strength) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->grab_subgrid(this, grab_point_local, false, strength);
}

void VoxelSubGrid::grab_with_rotation(Vector3 grab_point_local, float strength) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->grab_subgrid(this, grab_point_local, true, strength);
}

void VoxelSubGrid::release() {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->release_subgrid(this);
}

void VoxelSubGrid::set_grab_target(Transform3D target) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->set_grab_target(this, target);
}

void VoxelSubGrid::apply_impulse(Vector3 impulse, Vector3 world_point) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->apply_impulse(this, impulse, world_point);
}

void VoxelSubGrid::apply_central_impulse(Vector3 impulse) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->apply_central_impulse(this, impulse);
}
//

void VoxelSubGrid::set_manager(SubGridManager *manager) {
	_manager = manager;
}

Ref<VoxelToolSubGrid> VoxelSubGrid::get_voxel_tool() {
	ERR_FAIL_COND_V_MSG(_manager == nullptr, Ref<VoxelToolSubGrid>(), "No manager set on this VoxelSubGrid.");
	Ref<VoxelToolSubGrid> tool;
	tool.instantiate();
	tool->init(this, _manager);
	return tool;
}

//___________________________________________________________________________
// Persistence

void VoxelSubGrid::flush_dirty_chunks() {
	Vector<Vector3i> dirty = _chunks.get_dirty_chunks();
	for (Vector3i chunk_pos : dirty) {
		_save_chunk(chunk_pos);
		_chunks.mark_chunk_clean(chunk_pos);
		if (!_meta.chunk_positions.has(chunk_pos)) {
			_meta.chunk_positions.push_back(chunk_pos);
		}
	}
	// Sync chunk_positions with all known chunks in memory.
	// Chunks added via set_block_buffer are never dirty but must still be tracked in metadata so load_chunks_from_stream finds them.
	for (const Vector3i &pos : _chunks.get_all_chunk_positions()) {
		if (!_meta.chunk_positions.has(pos)) {
			_meta.chunk_positions.push_back(pos);
		}
	}
}

void VoxelSubGrid::load_chunks_from_stream() {
	print_line(String("load_chunks_from_stream: _saves_dir=") + _saves_dir);
	print_line(
			String("load_chunks_from_stream: globalized=") +
			ProjectSettings::get_singleton()->globalize_path(_saves_dir)
	);
	// Open a temporary stream just for loading, on main thread
	String saves_dir_abs = ProjectSettings::get_singleton()->globalize_path(_saves_dir);
	String uuid_str = uuid_to_string(_meta.uuid);
	String db_path = saves_dir_abs.path_join("ships").path_join(uuid_str + ".sqlite");

	Ref<VoxelStreamSQLite> stream;
	stream.instantiate();
	stream->set_database_path(db_path);
	int loaded_count = 0;

	print_line(String("load_chunks_from_stream: meta chunk positions=") + itos(_meta.chunk_positions.size()));
	for (Vector3i chunk_pos : _meta.chunk_positions) {
		std::shared_ptr<VoxelBuffer> buf = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_POOL);
		int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
		buf->create(cs, cs, cs);
		VoxelStream::VoxelQueryData q{ *buf, chunk_pos, 0, VoxelStream::RESULT_BLOCK_NOT_FOUND };
		stream->load_voxel_block(q);
		if (q.result == VoxelStream::RESULT_BLOCK_FOUND) {
			_chunks.set_block_buffer(chunk_pos, buf);
			print_line(String("load_chunks_from_stream: loaded chunk ") + String(chunk_pos));
		}
	}
	print_line(String("load_chunks_from_stream: chunk_map size after=") + itos(_chunks.get_chunk_count()));
	_chunks.rebuild_all_lods();
	print_line(
			String("load_chunks_from_stream: all_chunk_positions size=") +
			itos(_chunks.get_all_chunk_positions().size())
	);
}

void VoxelSubGrid::_close_stream() {
	if (_manager == nullptr)
		return;
	SubGridManager::SaveRequest req;
	req.uuid = uuid_to_string(_meta.uuid).utf8().get_data();
	req.saves_dir = _saves_dir.utf8().get_data();
	req.close_stream = true;
	_manager->push_save(req);
}

void VoxelSubGrid::save_and_close() {
	flush_dirty_chunks();
	_close_stream();
}

void VoxelSubGrid::destroy() {
	flush_dirty_chunks();
	_close_stream();
	// File deletion must happen after save thread closes the stream.
	// For now, push close and delete file after a brief wait, or
	// add a delete_after_close flag to SaveRequest.
	if (_manager != nullptr) {
		_manager->wait_save_queue();
	}
	String saves_dir_abs = ProjectSettings::get_singleton()->globalize_path(_saves_dir);
	String uuid_str = uuid_to_string(_meta.uuid);
	String db_path = saves_dir_abs.path_join("ships").path_join(uuid_str + ".sqlite");
	DirAccess::remove_absolute(db_path);
	queue_free();
}

void VoxelSubGrid::_save_chunk(Vector3i chunk_pos) {
	if (_manager == nullptr)
		return;
	std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);

	String saves_dir_abs = ProjectSettings::get_singleton()->globalize_path(_saves_dir);
	print_line(String("_save_chunk: saves_dir_abs=") + saves_dir_abs);

	// Ensure ships dir exists (main thread only)
	String ships_dir = saves_dir_abs.path_join("ships");
	if (!DirAccess::dir_exists_absolute(ships_dir)) {
		DirAccess::make_dir_recursive_absolute(ships_dir);
	}

	if (!buf)
		return;

	SubGridManager::SaveRequest req;
	req.uuid = uuid_to_string(_meta.uuid).utf8().get_data();
	// Globalize on main thread, not save thread
	req.saves_dir = ProjectSettings::get_singleton()->globalize_path(_saves_dir).utf8().get_data();
	req.chunk_pos = chunk_pos;
	req.buffer = buf;
	_manager->push_save(req);
}


//___________________________________________________________________________
// Rendering


//___________________________________________________________________________
// LOD


//___________________________________________________________________________
// Sub-contraption rotation


bool VoxelSubGrid::_should_lock() const {
	if (_lock_mode == LOCKED_ALWAYS)
		return true;
	if (_lock_mode == UNLOCKED_ALWAYS)
		return false;
	return _redstone_signal != (_lock_mode == LOCKED_DEFAULT);
}

void VoxelSubGrid::set_spin_axis(Vector3i axis) {
	ERR_FAIL_COND_MSG(
			(Math::abs(axis.x) + Math::abs(axis.y) + Math::abs(axis.z)) != 1, "spin axis must be a signed unit vector"
	);

	if (axis == -_spin_axis) {
		// Pure direction reversal on the same rotation plane: Basis(-axis, θ) == Basis(axis, -θ),
		// so negate the current angle to keep the visible orientation continuous at the moment
		// of the switch. Without this, flipping the axis sign causes an instantaneous jump of
		// 2*θ in the rendered transform instead of a clean reversal of future spin direction.
		_target_angle_rad = -_target_angle_rad;
		_target_angle_rad = Math::fmod(_target_angle_rad, ZN_TAU);
		_meta.target_angle_rad = _target_angle_rad;
	}

	_spin_axis = axis;
}

//___________________________________________________________________________
// Disassembly
bool VoxelSubGrid::is_rotation_grid_aligned(float tolerance_degrees) const {
	if (is_root() && !_meta.is_terrain_anchored) {
		return _is_full_orientation_grid_aligned(tolerance_degrees, /*upright_only=*/false);
	}
	// Project current angle onto nearest 90-degree increment
	double angle_deg = Math::rad_to_deg(_target_angle_rad);
	double nearest_90 = Math::round(angle_deg / 90.0) * 90.0;
	double diff = Math::abs(angle_deg - nearest_90);
	diff = Math::fmod(diff, 90.0);
	if (diff > 45.0)
		diff = 90.0 - diff;
	return diff <= tolerance_degrees;
}

// Recursive check - all children must be aligned
bool VoxelSubGrid::is_tree_grid_aligned(float tolerance_degrees) const {
	if (!is_rotation_grid_aligned(tolerance_degrees))
		return false;
	for (ObjectID id : _children) {
		Object *obj = ObjectDB::get_instance(id);
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(obj);
		if (child == nullptr)
			continue;
		if (!child->is_inside_tree())
			continue;
		if (!child->is_tree_grid_aligned(tolerance_degrees))
			return false;
	}
	return true;
}

void VoxelSubGrid::try_disassemble(VoxelLodTerrain *terrain) {
	try_disassemble_at(terrain, get_global_transform());
}
void VoxelSubGrid::try_disassemble_at(VoxelLodTerrain *terrain, const Transform3D &placement_t) {
	ERR_FAIL_COND_MSG(!is_inside_tree(), "Cannot disassemble: node not in tree");
	//ERR_FAIL_COND_MSG(!is_tree_grid_aligned(5.0f), "Cannot disassemble: not grid-aligned");
	if (!(is_root() && !_meta.is_terrain_anchored)) {
		//ERR_FAIL_COND_MSG(!is_tree_grid_aligned(5.0f), "Cannot disassemble: not grid-aligned");
	}
	if (is_root() || _is_world_anchored) {
		_disassemble_root_to_terrain(terrain, placement_t);
		return;
	}
	_disassemble_child_to_parent();
}

void VoxelSubGrid::_disassemble_root_to_terrain(VoxelLodTerrain *terrain, const Transform3D &placement_t) {
	// Only used to compute children's offsets RELATIVE to the root - never used directly for
	// final placement. placement_t is the sole source of truth for where anything ends up.
	Transform3D live_world_t = get_global_transform();

	Vector<VoxelSubGrid *> children_snapshot;
	for (ObjectID id : _children) {
		Object *obj = ObjectDB::get_instance(id);
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(obj);
		if (child == nullptr)
			continue;
		child->_meta.is_root = true;
		child->_meta.is_terrain_anchored = true;
		memset(child->_meta.parent_uuid, 0, 16);
		if (child->is_inside_tree()) {
			children_snapshot.push_back(child);
		}
	}

	Node *scene_parent = get_parent();

	for (VoxelSubGrid *child : children_snapshot) {
		// Preserve the child's actual relative offset from the root, then re-anchor that
		// offset onto placement_t instead of onto the (possibly stale) live root transform.
		Transform3D child_relative_to_root = live_world_t.affine_inverse() * child->get_global_transform();
		Transform3D child_world_t = placement_t * child_relative_to_root;

		Vector3 pivot_world =
				placement_t.xform(Vector3(child->get_metadata().pivot_in_parent_local) + Vector3(0.5f, 0.5f, 0.5f));

		if (_manager != nullptr) {
			_manager->unregister_ship(_manager->uuid_for_node(child));
		}

		child->_promoted_world_transform = child_world_t;
		child->_promoted_pivot_world = pivot_world;
		child->_is_world_anchored = true;
		child->_meta.is_root = true;
		memset(child->_meta.parent_uuid, 0, 16);
		child->_meta.promoted_pivot_world = pivot_world;
		child->_meta.world_position = child_world_t.origin;
		child->_meta.world_rotation = child_world_t.basis.get_rotation_quaternion();

		child->reparent(scene_parent, false);
		child->set_global_transform(child_world_t);

		if (_manager != nullptr) {
			_manager->register_ship_tree(child);
			_manager->mark_all_dirty(_manager->uuid_for_node(child));
		}
	}
	_children.clear();

	Ref<VoxelTool> tool = terrain->get_voxel_tool();
	tool->set_mode(VoxelTool::MODE_SET);
	_paste_rotated_chunks_to_terrain(tool.ptr(), placement_t);

	if (_manager != nullptr) {
		_manager->unregister_ship(uuid_to_string(_meta.uuid));
	}
	_close_stream();
	queue_free();
}

void VoxelSubGrid::_disassemble_child_to_parent() {
	ERR_FAIL_COND_MSG(!is_inside_tree(), "Cannot disassemble child: not in tree");
	ERR_FAIL_COND_MSG(!is_tree_grid_aligned(5.0f), "Cannot disassemble child: tree not grid-aligned");

	Node *p = get_parent();
	VoxelSubGrid *parent_sg = Object::cast_to<VoxelSubGrid>(p);
	ERR_FAIL_COND_MSG(parent_sg == nullptr, "Cannot disassemble child: parent is not VoxelSubGrid");

	// Snapshot transforms before touching anything
	Transform3D my_world_t = get_global_transform();
	Transform3D parent_world_t = parent_sg->get_global_transform();
	Transform3D to_parent = parent_world_t.affine_inverse() * my_world_t;

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;

	// Track which parent chunks get written so we can mark them dirty
	HashSet<Vector3i> dirty_parent_chunks;
	const int parent_cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;

	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);
		if (!buf)
			return;
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		for (int z = 0; z < cs; z++)
			for (int x = 0; x < cs; x++)
				for (int y = 0; y < cs; y++) {
					uint32_t type_v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
					if (type_v == 0)
						continue;
					Vector3 local_center = Vector3(block_origin + Vector3i(x, y, z)) + Vector3(0.5f, 0.5f, 0.5f);
					Vector3 parent_center = to_parent.xform(local_center);
					// Subtract 0.5 to get corner, then floor
					Vector3i parent_i = Vector3i(
							Math::floor(parent_center.x), Math::floor(parent_center.y), Math::floor(parent_center.z)
					);

					// Carry every tracked channel over, not just TYPE, so color/data5 survive
					// merging back into the parent contraption.
					for (int c = 0; c < SubGridChunkMap::SUBGRID_CHANNEL_COUNT; c++) {
						const VoxelBuffer::ChannelId channel = SubGridChunkMap::SUBGRID_CHANNELS[c];
						const uint32_t v =
								(channel == VoxelBuffer::CHANNEL_TYPE) ? type_v : buf->get_voxel(x, y, z, channel);
						parent_sg->set_voxel(v, parent_i, channel);
					}

					// Track which chunk this falls in
					Vector3i parent_chunk = Vector3i(
							Math::floor((float)parent_i.x / parent_cs),
							Math::floor((float)parent_i.y / parent_cs),
							Math::floor((float)parent_i.z / parent_cs)
					);
					dirty_parent_chunks.insert(parent_chunk);
				}
	});

	// Mark affected parent chunks dirty
	if (_manager != nullptr) {
		String parent_uuid = _manager->uuid_for_node(parent_sg);
		for (const Vector3i &chunk_pos : dirty_parent_chunks) {
			_manager->mark_chunk_dirty(parent_uuid, chunk_pos);
		}
	}

	// Remove self from parent's _children list
	parent_sg->_children.erase(get_instance_id());

	// Unregister and free
	if (_manager != nullptr) {
		_manager->unregister_ship(uuid_to_string(_meta.uuid));
	}
	_close_stream();
	queue_free();
}

void VoxelSubGrid::align_and_disassemble(float rpm, VoxelLodTerrain *terrain, bool upright_only, float snap_speed) {
	if (is_root() && !_meta.is_terrain_anchored) {
		// Rigidbody path: compute nearest permitted orientation, verify clearance, hand off to
		// SubGridManager's homing driver. Does NOT touch _target_angle_rad/_auto_align_* at all -
		// that machinery is for bearing-driven children only.
		ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");

		Basis current = get_global_basis().orthonormalized();
		Basis nearest = _nearest_cardinal_basis(current, upright_only);

		if (upright_only) {
			// "Upright" means the hull's own up-vector matches world-up, regardless of yaw (yaw is
			// always freely re-snappable to any of the 4 cardinal headings without tilting anything).
			// Comparing against the whole yaw-snapped basis was wrong: it folded "how far the current
			// yaw is from the nearest 90-degree bucket" into the tilt measurement, which is a
			// completely different question and explains the constant ~7 degree floor seen even at
			// pure-yaw-only rotations.
			Vector3 hull_up = current.get_column(1).normalized();
			float tilt_deg = Math::rad_to_deg(Math::acos(CLAMP(hull_up.dot(Vector3(0, 1, 0)), -1.f, 1.f)));

			ERR_FAIL_COND_MSG(
					tilt_deg > 44.0f,
					String("align_and_disassemble: cannot reach upright orientation for uuid ") +
							uuid_to_string(_meta.uuid) + " - hull's up vector is " + rtos(tilt_deg) +
							" degrees from world up (hull may be significantly tilted). Aborting."
			);
		}

		Vector3 raw_origin = get_global_position();
		Vector3i rounded_origin_i(
				(int)Math::round(raw_origin.x), (int)Math::round(raw_origin.y), (int)Math::round(raw_origin.z)
		);
		Transform3D candidate_t(nearest, Vector3(rounded_origin_i));

		ERR_FAIL_COND_MSG(
				!_check_terrain_clear_for_transform(terrain, candidate_t),
				"Cannot align and disassemble: candidate position/orientation overlaps existing terrain"
		);

		_manager->begin_homing_disassemble(this, candidate_t, terrain, snap_speed);
		return;
	}

	// Animatable-body path, unchanged shape
	_start_auto_align(rpm, upright_only);
	_auto_align_is_disassemble_root = true;
	_auto_align_terrain = terrain;

	for (ObjectID id : _children) {
		Object *obj = ObjectDB::get_instance(id);
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(obj);
		if (child != nullptr && child->is_inside_tree()) {
			child->_start_auto_align(rpm, upright_only);
		}
	}
}

void VoxelSubGrid::_start_auto_align(float rpm, bool upright_only) {
	_angular_speed_rpm = 0.f;

	if (upright_only) {
		// "Upright" = the as-assembled angle, which is always 0 per SubGridAssembler's
		// flood_fill: it never sets metadata.target_angle_rad for a newly built bearing
		// child, so it inherits SubGridMetadata's default of 0.
		_auto_align_target_rad = 0.0;
	} else {
		const double angle_deg = Math::rad_to_deg(_target_angle_rad);
		const double nearest_90 = Math::round(angle_deg / 90.0) * 90.0;
		_auto_align_target_rad = Math::deg_to_rad(nearest_90);
	}

	_auto_align_rpm = Math::abs(rpm);
	_auto_align_active = true;
}

bool VoxelSubGrid::_update_auto_align(double delta) {
	if (!_auto_align_active) {
		return true;
	}

	const double rad_per_sec = _auto_align_rpm * ZN_TAU / 60.0;
	const double remaining = _auto_align_target_rad - _target_angle_rad;

	// Shortest-path wrap: e.g. "270 the long way" becomes "-90 the short way"
	double shortest_remaining = Math::fmod(remaining, ZN_TAU);
	if (shortest_remaining > Math::PI) {
		shortest_remaining -= ZN_TAU;
	} else if (shortest_remaining < -Math::PI) {
		shortest_remaining += ZN_TAU;
	}

	if (rad_per_sec <= 0.0 || Math::abs(shortest_remaining) <= rad_per_sec * delta) {
		_target_angle_rad = _auto_align_target_rad;
		_meta.target_angle_rad = _target_angle_rad;
		_auto_align_active = false;
		return true;
	}

	const double step = rad_per_sec * delta;
	_target_angle_rad += (shortest_remaining > 0.0 ? step : -step);
	_target_angle_rad = Math::fmod(_target_angle_rad, ZN_TAU);
	_meta.target_angle_rad = _target_angle_rad;
	return false;
}

void VoxelSubGrid::clear_chunk_buffers() {
	// Cancel any in-flight saves by flushing first, then wipe RAM.
	// chunk_positions in _meta is untouched.
	_chunks.clear_buffers();
}

double VoxelSubGrid::advance_rotation(double delta) {
	if (_auto_align_active) {
		_update_auto_align(delta);
		return _target_angle_rad;
	}
	if (_should_lock()) {
		float rad_per_sec = _angular_speed_rpm * (float)ZN_TAU / 60.0f;
		if (_spin_axis.x < 0 || _spin_axis.y < 0 || _spin_axis.z < 0) {
			rad_per_sec *= -1.0f;
		}
		_target_angle_rad += rad_per_sec * delta;
		_target_angle_rad = Math::fmod(_target_angle_rad, ZN_TAU);
	}
	_meta.target_angle_rad = _target_angle_rad;
	return _target_angle_rad;
}

bool VoxelSubGrid::check_auto_align_disassemble_ready() {
	if (!_auto_align_is_disassemble_root || _auto_align_active) {
		return false;
	}
	if (!is_tree_grid_aligned(1.0f)) {
		return false;
	}

	if (is_root() || _is_world_anchored) {
		if (_auto_align_terrain != nullptr &&
			!_check_terrain_clear_for_transform(_auto_align_terrain, get_global_transform())) {
			WARN_PRINT("Auto-align disassembly blocked: target position overlaps existing terrain.");
			_auto_align_is_disassemble_root = false; // cancel rather than loop forever
			return false;
		}
	} else {
		VoxelSubGrid *parent_sg = Object::cast_to<VoxelSubGrid>(get_parent());
		if (parent_sg != nullptr) {
			Transform3D to_parent = parent_sg->get_global_transform().affine_inverse() * get_global_transform();
			if (!_check_subgrid_clear_for_transform(parent_sg->get_chunk_map_mut(), to_parent)) {
				WARN_PRINT("Auto-align disassembly blocked: target position overlaps parent contraption.");
				_auto_align_is_disassemble_root = false;
				return false;
			}
		}
	}

	_auto_align_is_disassemble_root = false;
	_auto_align_pending_disassemble = true;
	return true;
}

Transform3D VoxelSubGrid::compute_local_transform() const {
	// This is the same math that was in _apply_transform_from_angle,now returning a Transform3D instead of calling
	// set_transform() SubGridManager calls this and applies the result to AnimatableBody3D and the VoxelSubGrid Node3D
	Vector3 facing = Vector3(_meta.rotation_axis).normalized(); // mounting geometry - unchanged
	Vector3 pivot_in_child_local = Vector3(0.5f, 0.5f, 0.5f) - facing * 0.5f;
	Basis rotation_basis = Basis(Vector3(_spin_axis).normalized(), (real_t)_target_angle_rad); // spin direction
	Vector3 bearing_face_center = Vector3(_meta.pivot_in_parent_local) + Vector3(0.5f, 0.5f, 0.5f) + facing * 0.5f;

	Transform3D t;
	t.basis = rotation_basis;
	t.origin = bearing_face_center - rotation_basis.xform(pivot_in_child_local);
	return t;
}

Basis VoxelSubGrid::_nearest_cardinal_basis(const Basis &b_in, bool upright_only) {
	Basis b = b_in.orthonormalized();

	auto snap_to_cardinal = [](Vector3 v) -> Vector3 {
		v = v.normalized();
		const float ax = Math::abs(v.x), ay = Math::abs(v.y), az = Math::abs(v.z);
		if (ax >= ay && ax >= az)
			return Vector3(v.x < 0 ? -1.f : 1.f, 0, 0);
		if (ay >= ax && ay >= az)
			return Vector3(0, v.y < 0 ? -1.f : 1.f, 0);
		return Vector3(0, 0, v.z < 0 ? -1.f : 1.f);
	};

	if (upright_only) {
		// Up column locked to world up - no tilt allowed at all. Only yaw (around Y) is free,
		// snapped to the nearest of the 4 cardinal headings.
		Vector3 up(0, 1, 0);
		Vector3 x = snap_to_cardinal(Vector3(b.get_column(0).x, 0, b.get_column(0).z));
		Vector3 z = x.cross(up); // corrected: was up.cross(x), which produced a left-handed/reflected basis
		Basis result;
		result.set_column(0, x);
		result.set_column(1, up);
		result.set_column(2, z);
		return result;
	}

	// Full cube-orientation snap: round each basis axis to the nearest cardinal direction,
	// then re-derive the third axis via cross product to guarantee a valid orthonormal basis
	// (one of the 24 proper rotations of a cube).
	Vector3 x = snap_to_cardinal(b.get_column(0));
	Vector3 y = snap_to_cardinal(b.get_column(1));
	if (x == y || x == -y) {
		// Degenerate snap (both columns collapsed to the same axis) - fall back to deriving y
		// from whichever cardinal is not already used by x.
		y = snap_to_cardinal(b.get_column(2)).cross(x);
	}
	Vector3 z = x.cross(y);

	Basis result;
	result.set_column(0, x);
	result.set_column(1, y);
	result.set_column(2, z);
	return result;
}

bool VoxelSubGrid::_is_full_orientation_grid_aligned(float tolerance_degrees, bool upright_only) const {
	Basis current = get_global_basis().orthonormalized();
	Basis nearest = _nearest_cardinal_basis(current, upright_only);

	Basis diff = nearest.inverse() * current;
	Vector3 axis;
	real_t angle;
	diff.get_axis_angle(axis, angle);
	float angle_deg = Math::rad_to_deg(Math::abs((float)angle));
	if (angle_deg > 180.f)
		angle_deg = 360.f - angle_deg;
	return angle_deg <= tolerance_degrees;
}

namespace {
// Snaps a basis column to an exact integer unit vector. Only valid when the basis is already
// a true cardinal rotation (as produced by _nearest_cardinal_basis), not an approximately-aligned one.
Vector3i _snap_column_to_int(Vector3 col) {
	col = col.normalized();
	return Vector3i((int)Math::round(col.x), (int)Math::round(col.y), (int)Math::round(col.z));
}
} // namespace

bool VoxelSubGrid::_check_terrain_clear_for_transform(VoxelLodTerrain *terrain, const Transform3D &candidate_world_t) {
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;

	// Derive the exact integer rotation matrix, same as _paste_rotated_chunks_to_terrain -
	// no floating-point per-voxel transform at all, so there's no floor()-boundary noise to
	// disagree with the paste by a voxel.
	Basis b = candidate_world_t.basis.orthonormalized();
	const Vector3i ix = _snap_column_to_int(b.get_column(0));
	const Vector3i iy = _snap_column_to_int(b.get_column(1));
	const Vector3i iz = _snap_column_to_int(b.get_column(2));
	const int M[3][3] = {
		{ ix.x, iy.x, iz.x },
		{ ix.y, iy.y, iz.y },
		{ ix.z, iy.z, iz.z },
	};
	const Vector3i origin_i(
			(int)Math::round(candidate_world_t.origin.x),
			(int)Math::round(candidate_world_t.origin.y),
			(int)Math::round(candidate_world_t.origin.z)
	);

	AABB local_extent;
	bool has_any = false;
	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		AABB block_aabb(Vector3(block_origin), Vector3(cs, cs, cs));
		if (!has_any) {
			local_extent = block_aabb;
			has_any = true;
		} else {
			local_extent = local_extent.merge(block_aabb);
		}
	});
	if (!has_any) {
		return true;
	}

	// Probe box: same "world == placement space" assumption _paste_rotated_chunks_to_terrain
	// makes (no terrain_to_local applied to per-voxel positions - only used here to size the
	// batch fetch region, which tolerates being a bit generous).
	const Transform3D terrain_to_local = terrain->get_global_transform().affine_inverse();
	const AABB candidate_world_aabb = candidate_world_t.xform(local_extent);
	const AABB candidate_terrain_local_aabb = terrain_to_local.xform(candidate_world_aabb);
	const Box3i probe_box = Box3i::from_min_max(
			math::floor_to_int(candidate_terrain_local_aabb.position) - Vector3i(1, 1, 1),
			math::ceil_to_int(candidate_terrain_local_aabb.position + candidate_terrain_local_aabb.size) +
					Vector3i(1, 1, 1)
	);

	VoxelBuffer terrain_probe(VoxelBuffer::ALLOCATOR_POOL);
	terrain->get_storage().get_voxels_batch(probe_box, VoxelBuffer::CHANNEL_TYPE, terrain_probe);

	struct Entry {
		Vector3i terrain_voxel;
	};
	LocalVector<Entry> entries;
	HashMap<Vector2i, int32_t> column_min_y;

	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);
		if (!buf)
			return;
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		for (int z = 0; z < cs; z++) {
			for (int x = 0; x < cs; x++) {
				for (int y = 0; y < cs; y++) {
					uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
					if (v == 0)
						continue;

					// Exact integer mapping, identical formula to the paste - no float xform,
					// no floor(), so no possibility of disagreeing by a voxel due to noise.
					Vector3i local_p = block_origin + Vector3i(x, y, z);
					Vector3i rotated;
					rotated.x = M[0][0] * local_p.x + M[0][1] * local_p.y + M[0][2] * local_p.z;
					rotated.y = M[1][0] * local_p.x + M[1][1] * local_p.y + M[1][2] * local_p.z;
					rotated.z = M[2][0] * local_p.x + M[2][1] * local_p.y + M[2][2] * local_p.z;
					Vector3i terrain_voxel = origin_i + rotated;

					entries.push_back({ terrain_voxel });

					Vector2i col(terrain_voxel.x, terrain_voxel.z);
					int32_t *existing = column_min_y.getptr(col);
					if (existing == nullptr || terrain_voxel.y < *existing) {
						column_min_y[col] = terrain_voxel.y;
					}
				}
			}
		}
	});

	for (const Entry &e : entries) {
		Vector2i col(e.terrain_voxel.x, e.terrain_voxel.z);
		if (e.terrain_voxel.y <= column_min_y[col]) {
			continue;
		}
		Vector3i probe_local = e.terrain_voxel - probe_box.position;
		if (probe_local.x < 0 || probe_local.y < 0 || probe_local.z < 0 || probe_local.x >= probe_box.size.x ||
			probe_local.y >= probe_box.size.y || probe_local.z >= probe_box.size.z) {
			continue;
		}
		uint32_t terrain_v =
				terrain_probe.get_voxel(probe_local.x, probe_local.y, probe_local.z, VoxelBuffer::CHANNEL_TYPE);
		if (terrain_v != 0) {
			return false;
		}
	}

	return true;
}

void VoxelSubGrid::_paste_rotated_chunks_to_terrain(VoxelTool *tool, const Transform3D &placement_t) {
	Basis b = placement_t.basis.orthonormalized();
	const Vector3i ix = _snap_column_to_int(b.get_column(0));
	const Vector3i iy = _snap_column_to_int(b.get_column(1));
	const Vector3i iz = _snap_column_to_int(b.get_column(2));
	// M as a 3x3 of {-1,0,1}, row-major, for the offset-correction formula below.
	const int M[3][3] = {
		{ ix.x, iy.x, iz.x },
		{ ix.y, iy.y, iz.y },
		{ ix.z, iy.z, iz.z },
	};

	// Local voxel-space bounding box of the whole hull, so rotated coordinates can be kept
	// non-negative via the correct per-axis offset rather than naive matrix*index (which
	// produces negative/mirrored results whenever a column is negative - that mirroring is
	// exactly what was showing up as "wrap").
	Vector3i bbox_min(INT32_MAX, INT32_MAX, INT32_MAX);
	Vector3i bbox_max(INT32_MIN, INT32_MIN, INT32_MIN);
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		bbox_min = bbox_min.min(block_origin);
		bbox_max = bbox_max.max(block_origin + Vector3i(cs - 1, cs - 1, cs - 1));
	});
	const Vector3i size = bbox_max - bbox_min + Vector3i(1, 1, 1); // local extent, pre-rotation

	// Per-output-axis offset: whenever M's row has a -1 entry for input axis j, that term
	// ranges over [-(size[j]-1), 0], so adding (size[j]-1) shifts it back to [0, size[j]-1].
	// Without this, rotations with a negative column produce negative/overlapping coordinates.
	int offset[3] = { 0, 0, 0 };
	for (int out_axis = 0; out_axis < 3; ++out_axis) {
		for (int in_axis = 0; in_axis < 3; ++in_axis) {
			if (M[out_axis][in_axis] == -1) {
				offset[out_axis] += size[in_axis] - 1;
			}
		}
	}

	const Vector3i origin_i(
			(int)Math::round(placement_t.origin.x),
			(int)Math::round(placement_t.origin.y),
			(int)Math::round(placement_t.origin.z)
	);
	print_line(
			String("_paste_rotated_chunks_to_terrain: placement_t.origin = ") + String(placement_t.origin) +
			" rounded origin_i = " + String(origin_i)
	);


	Ref<VoxelMesherBlocky> blocky_mesher = _mesher;
	Ref<VoxelBlockyLibraryBase> lib = blocky_mesher.is_valid() ? blocky_mesher->get_library() : Ref<VoxelBlockyLibraryBase>();

	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);
		if (!buf)
			return;
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		for (int z = 0; z < cs; z++) {
			for (int x = 0; x < cs; x++) {
				for (int y = 0; y < cs; y++) {
					uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
					if (v == 0)
						continue;

					bool relevant = false;
					if (lib.is_valid()) {
						RWLockRead lock(lib->get_baked_data_rw_lock());
						const blocky::BakedLibrary &baked = lib->get_baked_data();
						relevant = baked.has_model(v) && baked.models[v].light_emission > 0;
					}

					Vector3i local_p = block_origin + Vector3i(x, y, z);
					Vector3i rotated;
					rotated.x = M[0][0] * local_p.x + M[0][1] * local_p.y + M[0][2] * local_p.z;
					rotated.y = M[1][0] * local_p.x + M[1][1] * local_p.y + M[1][2] * local_p.z;
					rotated.z = M[2][0] * local_p.x + M[2][1] * local_p.y + M[2][2] * local_p.z;
					Vector3i target_pos = origin_i + rotated;

					for (int c = 0; c < SubGridChunkMap::SUBGRID_CHANNEL_COUNT; c++) {
						const VoxelBuffer::ChannelId channel = SubGridChunkMap::SUBGRID_CHANNELS[c];
						const uint32_t cv = (channel == VoxelBuffer::CHANNEL_TYPE) ? v : buf->get_voxel(x, y, z, channel);
						tool->set_channel(channel);
						tool->set_voxel(target_pos, cv, relevant);
					}
					tool->set_channel(VoxelBuffer::CHANNEL_TYPE); // restore for next iteration's read
				}
			}
		}
	});
}

bool VoxelSubGrid::_check_subgrid_clear_for_transform(SubGridChunkMap &parent_chunks, const Transform3D &to_parent_t) {
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	bool clear = true;
	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		if (!clear)
			return;
		std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);
		if (!buf)
			return;
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		for (int z = 0; z < cs && clear; z++) {
			for (int x = 0; x < cs && clear; x++) {
				for (int y = 0; y < cs && clear; y++) {
					uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
					if (v == 0)
						continue;
					Vector3 local_center = Vector3(block_origin + Vector3i(x, y, z)) + Vector3(0.5f, 0.5f, 0.5f);
					Vector3 parent_center = to_parent_t.xform(local_center);
					Vector3i parent_i(
							(int)Math::floor(parent_center.x),
							(int)Math::floor(parent_center.y),
							(int)Math::floor(parent_center.z)
					);
					if (parent_chunks.get_voxel(parent_i, VoxelBuffer::CHANNEL_TYPE) != 0) {
						clear = false;
					}
				}
			}
		}
	});
	return clear;
}

} // namespace zylann::voxel
