#pragma once
#include "../edition/voxel_tool.h"

namespace zylann::voxel {

// Forward declarations — no includes needed in the header
class VoxelSubGrid;
class SubGridManager;

class VoxelToolSubGrid : public VoxelTool {
	GDCLASS(VoxelToolSubGrid, VoxelTool)
public:
	VoxelToolSubGrid() = default;
	// Use init() instead of constructor - Ref<> requires default constructor
	void init(VoxelSubGrid *grid, SubGridManager *manager);

	bool is_area_editable(const Box3i &box) const override;
	VoxelFormat get_format() const override;

protected:
	uint64_t _get_voxel(Vector3i pos) const override;
	float _get_voxel_f(Vector3i pos) const override;
	void _set_voxel(Vector3i pos, uint64_t v) override;
	void _set_voxel_f(Vector3i pos, float v) override;
	void _post_edit(const Box3i &box) override;

private:
	static void _bind_methods();
	void _mark_box_dirty(const Box3i &box);

	VoxelSubGrid *_grid = nullptr;
	SubGridManager *_manager = nullptr;
};

} // namespace zylann::voxel