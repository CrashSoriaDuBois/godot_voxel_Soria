#include "voxel_sub_grid.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "core/math/math_defs.h"
#include "edition/voxel_tool.h"
#include "sub_grid_manager.h"
#include "voxel_tool_sub_grid.h"

namespace zylann::voxel {

static constexpr double ZN_TAU = 6.28318530717958647692;

//___________________________________________________________________________
// Godot hooks

void VoxelSubGrid::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_viewer", "viewer"), &VoxelSubGrid::set_viewer);
	ClassDB::bind_method(D_METHOD("destroy"), &VoxelSubGrid::destroy);
	ClassDB::bind_method(D_METHOD("disassemble_to_terrain", "terrain"), &VoxelSubGrid::disassemble_to_terrain);
	ClassDB::bind_method(D_METHOD("flush_dirty_chunks"), &VoxelSubGrid::flush_dirty_chunks);
	ClassDB::bind_method(D_METHOD("set_angular_speed_rpm", "rpm"), &VoxelSubGrid::set_angular_speed_rpm);
	ClassDB::bind_method(D_METHOD("is_root"), &VoxelSubGrid::is_root);
	ClassDB::bind_method(D_METHOD("is_tree_grid_aligned", "tolerance_degrees"), &VoxelSubGrid::is_tree_grid_aligned);

	ClassDB::bind_method(D_METHOD("get_target_angle_rad"), &VoxelSubGrid::get_target_angle_rad);

	ClassDB::bind_method(D_METHOD("get_voxel_tool"), &VoxelSubGrid::get_voxel_tool);
}

void VoxelSubGrid::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			set_process(true);
			break;

		case NOTIFICATION_EXIT_TREE:
			flush_dirty_chunks();
			SubGridStreamHelper::close(_stream);
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

	_stream = SubGridStreamHelper::open(saves_dir, meta.uuid);
	flush_dirty_chunks();

	set_global_position(_meta.world_position);
	set_global_basis(Basis(_meta.world_rotation));
}

void VoxelSubGrid::initialize_root_from_disk(
        const SubGridMetadata &meta,
        const String &saves_dir,
        Ref<VoxelMesherBlocky> mesher,
        Ref<VoxelBlockyLibrary> library) {
    _meta = meta;
    _saves_dir = saves_dir;
    _mesher = mesher;
    _library = library;

    _stream = SubGridStreamHelper::open(saves_dir, meta.uuid);
	load_chunks_from_stream(); // correct location

    set_global_position(_meta.world_position);
    set_global_basis(Basis(_meta.world_rotation));
}

void VoxelSubGrid::initialize_child(const SubGridMetadata &meta, SubGridChunkMap &&chunks, const String &saves_dir) {
	_meta = meta;
	_chunks = std::move(chunks);
	_saves_dir = saves_dir;

	// Ensure parent_uuid is set from actual parent node
	VoxelSubGrid *parent_sg = Object::cast_to<VoxelSubGrid>(get_parent());
	if (parent_sg != nullptr) {
		memcpy(_meta.parent_uuid, parent_sg->get_metadata().uuid, 16);
		parent_sg->_children.push_back(this);
	}

	VoxelSubGrid *root = get_root();
	_mesher = root->_mesher;
	_library = root->_library;

	_stream = SubGridStreamHelper::open(saves_dir, meta.uuid);
	flush_dirty_chunks();
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

		// Keep chunk list in metadata up to date
		if (!_meta.chunk_positions.has(chunk_pos)) {
			_meta.chunk_positions.push_back(chunk_pos);
		}
	}
}

void VoxelSubGrid::load_chunks_from_stream() {
	for (Vector3i chunk_pos : _meta.chunk_positions) {
		// Allocate a buffer for this chunk
		std::shared_ptr<VoxelBuffer> buf = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_POOL);
		int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
		buf->create(cs, cs, cs);

		VoxelStream::VoxelQueryData q{ *buf, chunk_pos, 0, VoxelStream::RESULT_BLOCK_NOT_FOUND };
		_stream->load_voxel_block(q);

		if (q.result == VoxelStream::RESULT_BLOCK_FOUND) {
			_chunks.set_block_buffer(chunk_pos, buf);
		} else {
			print_line(String("Warning: chunk not found in stream at ") + String(chunk_pos));
		}
	}
}

void VoxelSubGrid::save_and_close() {
	flush_dirty_chunks();
	SubGridStreamHelper::close(_stream);
}

void VoxelSubGrid::destroy() {
	flush_dirty_chunks();
	SubGridStreamHelper::close_and_delete(_stream, _saves_dir, _meta.uuid);
	queue_free();
}

void VoxelSubGrid::_save_chunk(Vector3i chunk_pos) {
	std::shared_ptr<VoxelBuffer> buf = _chunks.get_chunk_buffer(chunk_pos);
	if (!buf) {
		return;
	}
	// Dereference shared_ptr to get VoxelBuffer &
	VoxelStream::VoxelQueryData q{ *buf, chunk_pos, 0, VoxelStream::RESULT_BLOCK_NOT_FOUND };
	_stream->save_voxel_block(q);
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
	for (VoxelSubGrid *child : _children) {
		if (!child->is_tree_grid_aligned(tolerance_degrees))
			return false;
	}
	return true;
}

void VoxelSubGrid::disassemble_to_terrain(VoxelLodTerrain *terrain) {
    // Check all sub-contraptions are grid-aligned before proceeding
    if (!is_tree_grid_aligned(5.0f)) { // 5 degree tolerance
        print_line("Cannot disassemble: sub-contraptions are not grid-aligned. "
                   "Rotate them to 0/90/180/270 degrees first.");
        return;
    }
	ERR_FAIL_COND_MSG(terrain == nullptr, "terrain is null");

	// Disassemble children first - depth first, leaves first
	for (VoxelSubGrid *child : _children) {
		child->disassemble_to_terrain(terrain);
	}

	Ref<VoxelTool> tool = terrain->get_voxel_tool();
	ERR_FAIL_COND_MSG(!tool.is_valid(), "get_voxel_tool returned null");

	tool->set_channel(VoxelBuffer::CHANNEL_TYPE);
	tool->set_mode(VoxelTool::MODE_SET);

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;

	if (is_root()) {
		// Root: use integer world origin - no rotation
		Vector3i world_origin = Vector3i(
				Math::round(_meta.world_position.x),
				Math::round(_meta.world_position.y),
				Math::round(_meta.world_position.z)
		);

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
						Vector3i local = block_origin + Vector3i(x, y, z);
						tool->set_voxel(local + world_origin, v);
					}
				}
			}
		});
	} else {
		// Child: snap to nearest 90 degree increment before placing blocks. This prevents floating point drift from causing misaligned blocks
		double snapped_angle = Math::round(_target_angle_rad / (Math::PI * 0.5)) * (Math::PI * 0.5);

		Transform3D world_t = get_global_transform();

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
						Vector3i local = block_origin + Vector3i(x, y, z);
						Vector3 world_f = world_t.xform(Vector3(local) + Vector3(0.5f, 0.5f, 0.5f));
						Vector3i world_i =
								Vector3i(Math::round(world_f.x), Math::round(world_f.y), Math::round(world_f.z));
						tool->set_voxel(world_i, v);
					}
				}
			}
		});
	}

	SubGridStreamHelper::close(_stream);


	SubGridManager *mgr =
			Object::cast_to<SubGridManager>(get_parent() ? get_parent()->get_node_or_null(String("SubGridManager")) : nullptr);
	if (mgr != nullptr) {
		mgr->unregister_ship(uuid_to_string(_meta.uuid));
	}
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