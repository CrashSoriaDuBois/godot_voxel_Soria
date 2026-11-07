#pragma once
#include "../edition/voxel_tool.h"

namespace zylann::voxel {

// Forward declarations, no includes needed in the header
class VoxelSubGrid;
class SubGridManager;

class VoxelToolSubGrid : public VoxelTool {
	GDCLASS(VoxelToolSubGrid, VoxelTool)
public:
	VoxelToolSubGrid() = default;
	// Use init() instead of constructor, Ref<> requires default constructor
	void init(VoxelSubGrid *grid, SubGridManager *manager);

	bool is_area_editable(const Box3i &box) const override;
	VoxelFormat get_format() const override;

	bool set_voxel_grid(Vector3i pos, int res, PackedInt32Array cells, PackedInt32Array colors = PackedInt32Array(), bool relevant = true) override;
	bool clear_voxel_grid(Vector3i pos, bool relevant = true) override;
	int set_voxel_grid_cell(Vector3i pos, int res, Vector3i cell, int id, int color = -1, bool relevant = true) override;
	bool set_voxel_grid_cell_color(Vector3i pos, Vector3i cell, int color, bool relevant = false) override;
	Dictionary get_voxel_grid(Vector3i pos) const override;

protected:
	uint64_t _get_voxel(Vector3i pos) const override;
	float _get_voxel_f(Vector3i pos) const override;
	void _set_voxel(Vector3i pos, uint64_t v) override;
	void _set_voxel_f(Vector3i pos, float v) override;
	void _post_edit(const Box3i &box, bool p_relevant = false) override;

private:
	static void _bind_methods();
	void _mark_box_dirty(const Box3i &box, bool relevant = false);

	VoxelSubGrid *_grid = nullptr;
	SubGridManager *_manager = nullptr;
};

} // namespace zylann::voxel
