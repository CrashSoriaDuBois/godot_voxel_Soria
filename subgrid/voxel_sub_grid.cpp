#include "voxel_sub_grid.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "core/math/math_defs.h"
#include "core/object/object.h"
#include "edition/voxel_tool.h"
#include "sub_grid_manager.h"
#include "voxel_tool_sub_grid.h"
#include "core/io/dir_access.h"

namespace zylann::voxel {

static constexpr double ZN_TAU = 6.28318530717958647692;

//___________________________________________________________________________
// Godot hooks

void VoxelSubGrid::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_viewer", "viewer"), &VoxelSubGrid::set_viewer);
	ClassDB::bind_method(D_METHOD("destroy"), &VoxelSubGrid::destroy);
	ClassDB::bind_method(D_METHOD("disassemble", "terrain"), &VoxelSubGrid::disassemble);
	ClassDB::bind_method(D_METHOD("flush_dirty_chunks"), &VoxelSubGrid::flush_dirty_chunks);
	ClassDB::bind_method(D_METHOD("set_angular_speed_rpm", "rpm"), &VoxelSubGrid::set_angular_speed_rpm);
	ClassDB::bind_method(D_METHOD("is_root"), &VoxelSubGrid::is_root);
	ClassDB::bind_method(D_METHOD("is_tree_grid_aligned", "tolerance_degrees"), &VoxelSubGrid::is_tree_grid_aligned);

	ClassDB::bind_method(D_METHOD("get_target_angle_rad"), &VoxelSubGrid::get_target_angle_rad);

	ClassDB::bind_method(D_METHOD("get_voxel_tool"), &VoxelSubGrid::get_voxel_tool);

	ClassDB::bind_method(D_METHOD("grab", "grab_point_local"), &VoxelSubGrid::grab);
	ClassDB::bind_method(D_METHOD("grab_with_rotation", "grab_point_local"), &VoxelSubGrid::grab_with_rotation);
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
		Ref<VoxelBlockyLibrary> library
) {
	_meta = meta;
	_chunks = std::move(chunks);
	_saves_dir = saves_dir;
	_mesher = mesher;
	_library = library;

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
		Ref<VoxelBlockyLibrary> library
) {
    _meta = meta;
    _saves_dir = saves_dir;
    _mesher = mesher;
    _library = library;
    _promoted_pivot_world = meta.promoted_pivot_world;
    _is_world_anchored = meta.is_terrain_anchored;

    load_chunks_from_stream();

    print_line(String("initialize_root_from_disk:"));
    print_line(String("  world_position = ") + String(meta.world_position));
    print_line(String("  world_rotation = (") + 
        rtos(meta.world_rotation.x) + ", " + rtos(meta.world_rotation.y) + ", " +
        rtos(meta.world_rotation.z) + ", " + rtos(meta.world_rotation.w) + ")");
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
		bool async_stream
) {
	_meta = meta;
	_chunks = std::move(chunks);
	_saves_dir = saves_dir;

	VoxelSubGrid *parent_sg = Object::cast_to<VoxelSubGrid>(get_parent());
	if (parent_sg != nullptr) {
		memcpy(_meta.parent_uuid, parent_sg->get_metadata().uuid, 16);
		parent_sg->_children.push_back(get_instance_id());
	}

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

void VoxelSubGrid::grab(Vector3 grab_point_local) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->grab_subgrid(this, grab_point_local, false);
}

void VoxelSubGrid::grab_with_rotation(Vector3 grab_point_local) {
	ERR_FAIL_COND_MSG(_manager == nullptr, "No manager set on this VoxelSubGrid.");
	_manager->grab_subgrid(this, grab_point_local, true);
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
}

void VoxelSubGrid::load_chunks_from_stream() {
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

//___________________________________________________________________________
// Disassembly
bool VoxelSubGrid::is_rotation_grid_aligned(float tolerance_degrees) const {
	if (is_root())
		return true; // root is always aligned

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

void VoxelSubGrid::disassemble(VoxelLodTerrain *terrain) {
	ERR_FAIL_COND_MSG(!is_inside_tree(), "Cannot disassemble: node not in tree");
	ERR_FAIL_COND_MSG(!is_tree_grid_aligned(5.0f), "Cannot disassemble: sub-contraptions not grid-aligned");

	if (is_root()) {
		_disassemble_root_to_terrain(terrain);
		return;
	}

	if (_is_world_anchored) {
		// Promoted child, parent is scene root, not a VoxelSubGrid
		// Disassemble directly to terrain at current world position
		_disassemble_root_to_terrain(terrain);
		return;
	}

	_disassemble_child_to_parent();
}

void VoxelSubGrid::_disassemble_root_to_terrain(VoxelLodTerrain *terrain) {
	// Snapshot OWN transform NOW before anything touches the tree
	Transform3D my_world_t = get_global_transform();

	// Promote children to roots BEFORE freeing self
	// Copy list first since reparenting modifies it
	Vector<VoxelSubGrid *> children_snapshot;
	for (ObjectID id : _children) {
		Object *obj = ObjectDB::get_instance(id);

		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(obj);

		child->_meta.is_root = true;
		child->_meta.is_terrain_anchored = true;
		memset(child->_meta.parent_uuid, 0, 16);

		if (child == nullptr)
			continue;
		if (child != nullptr && child->is_inside_tree()) {
			children_snapshot.push_back(child);
		}
	}

	Node *scene_parent = get_parent();

	for (VoxelSubGrid *child : children_snapshot) {
		Transform3D child_world_t = child->get_global_transform();
    
		Transform3D parent_world_t = get_global_transform();
		Vector3 pivot_world = parent_world_t.xform(Vector3(child->get_metadata().pivot_in_parent_local) + Vector3(0.5f, 0.5f, 0.5f));

		if (_manager != nullptr) {
			_manager->unregister_ship(_manager->uuid_for_node(child));
		}
    
		child->_promoted_world_transform = child_world_t;
		child->_promoted_pivot_world = pivot_world;
		child->_is_world_anchored = true;

		// This child is becoming an independent ship
		child->_meta.is_root = true;
		memset(child->_meta.parent_uuid, 0, 16);

		child->_meta.world_position = child_world_t.origin;
		child->_meta.world_rotation = child_world_t.basis.get_rotation_quaternion();

		// DON'T set is_root=true
		// Reparent to scene but keep as child type
		child->reparent(scene_parent, false);
		child->set_global_transform(child_world_t);

		if (_manager != nullptr) {
			_manager->register_ship_tree(child);
			_manager->mark_all_dirty(_manager->uuid_for_node(child));
		}
	}
	_children.clear();

	// Now place OWN blocks into terrain using snapshotted transform
	Ref<VoxelTool> tool = terrain->get_voxel_tool();
	tool->set_channel(VoxelBuffer::CHANNEL_TYPE);
	tool->set_mode(VoxelTool::MODE_SET);

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	_chunks.for_each_chunk([&](Vector3i chunk_pos, VoxelDataBlock &) {
		std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);
		if (!buf)
			return;
		Vector3i block_origin = chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		for (int z = 0; z < cs; z++)
			for (int x = 0; x < cs; x++)
				for (int y = 0; y < cs; y++) {
					uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
					if (v == 0)
						continue;
					Vector3 local = Vector3(block_origin + Vector3i(x, y, z)) + Vector3(0.5f, 0.5f, 0.5f);
					Vector3 world_f = my_world_t.xform(local);
					Vector3i world_i = Vector3i(Math::round(world_f.x), Math::round(world_f.y), Math::round(world_f.z));
					tool->set_voxel(world_i, v);
				}
	});

	// Unregister and free self
	if (_manager != nullptr) {
		_manager->unregister_ship(uuid_to_string(_meta.uuid));
	}
	_close_stream();
	queue_free();
}

void VoxelSubGrid::_disassemble_child_to_parent() {
	ERR_FAIL_COND_MSG(!is_inside_tree(), "Cannot disassemble child: not in tree");

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
					uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
					if (v == 0)
						continue;
					Vector3 local_center = Vector3(block_origin + Vector3i(x, y, z)) + Vector3(0.5f, 0.5f, 0.5f);
					Vector3 parent_center = to_parent.xform(local_center);
					// Subtract 0.5 to get corner, then floor
					Vector3i parent_i = Vector3i(
							Math::floor(parent_center.x), Math::floor(parent_center.y), Math::floor(parent_center.z)
					);
					parent_sg->set_voxel(v, parent_i, VoxelBuffer::CHANNEL_TYPE);

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

void VoxelSubGrid::clear_chunk_buffers() {
	// Cancel any in-flight saves by flushing first, then wipe RAM.
	// chunk_positions in _meta is untouched.
	_chunks.clear_buffers();
}

double VoxelSubGrid::advance_rotation(double delta) {
	if (_should_lock()) {
		float rad_per_sec = _angular_speed_rpm * (float)ZN_TAU / 60.0f;
		if (_meta.rotation_axis.x < 0 || _meta.rotation_axis.y < 0 || _meta.rotation_axis.z < 0) {
			rad_per_sec *= -1.0f;
		}
		_target_angle_rad += rad_per_sec * delta;
		_target_angle_rad = Math::fmod(_target_angle_rad, ZN_TAU);
	}
	_meta.target_angle_rad = _target_angle_rad;
	return _target_angle_rad;
}

Transform3D VoxelSubGrid::compute_local_transform() const {
	// This is the same math that was in _apply_transform_from_angle,now returning a Transform3D instead of calling set_transform()
	// SubGridManager calls this and applies the result to AnimatableBody3D and the VoxelSubGrid Node3D

	Vector3 facing = Vector3(_meta.rotation_axis).normalized();
	Vector3 pivot_in_child_local = Vector3(0.5f, 0.5f, 0.5f) - facing * 0.5f;
	Basis rotation_basis = Basis(facing, (real_t)_target_angle_rad);
	Vector3 bearing_face_center = Vector3(_meta.pivot_in_parent_local) + Vector3(0.5f, 0.5f, 0.5f) + facing * 0.5f;

	Transform3D t;
	t.basis = rotation_basis;
	t.origin = bearing_face_center - rotation_basis.xform(pivot_in_child_local);
	return t;
}

} // namespace zylann::voxel