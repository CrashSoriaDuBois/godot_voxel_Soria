#include "voxel_tool_sub_grid.h"
#include "lod/sub_grid_chunk_map.h"
#include "storage/voxel_format.h"
#include "voxel_sub_grid.h"
#include "sub_grid_manager.h"

namespace zylann::voxel {

void VoxelToolSubGrid::init(VoxelSubGrid *grid, SubGridManager *manager) {
	ERR_FAIL_COND(grid == nullptr);
	ERR_FAIL_COND(manager == nullptr);
	_grid = grid;
	_manager = manager;
	_channel = VoxelBuffer::CHANNEL_TYPE;
	_mode = MODE_SET;
}

// ____________________________________________________________________________
// VoxelTool overrides

bool VoxelToolSubGrid::is_area_editable(const Box3i &box) const {
	// SubGrids have no fixed bounds - any local position is valid as long
	// as the grid exists. The chunk will be created on first write if needed.
	return _grid != nullptr;
}

VoxelFormat VoxelToolSubGrid::get_format() const {
	return VoxelFormat();
}

// ____________________________________________________________________________
// Low-level voxel access (called by base class set_voxel / get_voxel)

uint64_t VoxelToolSubGrid::_get_voxel(Vector3i pos) const {
	ERR_FAIL_COND_V(_grid == nullptr, 0);
	return _grid->get_voxel(pos, _channel);
}

float VoxelToolSubGrid::_get_voxel_f(Vector3i pos) const {
	// SubGrids are blocky only, SDF not supported
	return 0.f;
}

void VoxelToolSubGrid::_set_voxel(Vector3i pos, uint64_t v) {
	ERR_FAIL_COND(_grid == nullptr);
	_grid->set_voxel((uint32_t)v, pos, _channel);
}

void VoxelToolSubGrid::_set_voxel_f(Vector3i pos, float v) {
	// SubGrids are blocky only, SDF not supported
}

// ____________________________________________________________________________
// Helpers

// File-local uuid helper (same logic as the others in the subgrid system)
static String _uuid_str(const uint8_t *uuid) {
	String s;
	for (int i = 0; i < 16; i++) {
		s += String::num_int64(uuid[i] >> 4, 16);
		s += String::num_int64(uuid[i] & 0xF, 16);
	}
	return s;
}

// ____________________________________________________________________________
// Post-edit: notify SubGridManager which chunks need remeshing
//
// This mirrors VoxelToolTerrain::_post_edit -> terrain->post_edit_area(box).
// The box is in local voxel space. We convert it to chunk positions and
// mark each dirty. The manager's _process() will pick them up next frame.

void VoxelToolSubGrid::_post_edit(const Box3i &box) {
	ERR_FAIL_COND(_grid == nullptr);
	ERR_FAIL_COND(_manager == nullptr);
	_mark_box_dirty(box);
}

void VoxelToolSubGrid::_mark_box_dirty(const Box3i &box) {
	const int chunk_size_po2 = SubGridChunkMap::CHUNK_SIZE_PO2;

	// Convert voxel box corners to chunk positions (floor division).
	// We expand by 1 in each direction to account for the padding border:
	// a voxel at the edge of a chunk affects the neighbor's padding too.
	Vector3i voxel_min = box.position - Vector3i(1, 1, 1);
	Vector3i voxel_max = box.position + box.size + Vector3i(1, 1, 1);

	Vector3i chunk_min = voxel_min >> chunk_size_po2;
	Vector3i chunk_max = voxel_max >> chunk_size_po2;

	String uuid = _uuid_str(_grid->get_metadata().uuid);

	for (int z = chunk_min.z; z <= chunk_max.z; z++) {
		for (int y = chunk_min.y; y <= chunk_max.y; y++) {
			for (int x = chunk_min.x; x <= chunk_max.x; x++) {
				_manager->mark_chunk_dirty(uuid, Vector3i(x, y, z));
			}
		}
	}
}

// ____________________________________________________________________________
// GDScript bindings
//
// Most of the API (set_voxel, get_voxel, do_box, do_sphere, set_channel,
// set_mode, set_value...) is inherited from VoxelTool and already bound there.
// We only need to bind subgrid-specific additions here.

void VoxelToolSubGrid::_bind_methods() {
	// No subgrid-specific methods yet beyond what VoxelTool already exposes.
	// Add here if needed, e.g. a subgrid-local raycast.
}

} // namespace zylann::voxel