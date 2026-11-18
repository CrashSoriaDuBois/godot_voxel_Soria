#ifndef VOXEL_BLOCKY_CUBE_TILE_PICKER_H
#define VOXEL_BLOCKY_CUBE_TILE_PICKER_H

#include "../../util/godot/classes/margin_container.h"
#include "../../util/godot/classes/texture_2d.h"
#include "../../util/godot/macros.h"
#include "../../util/math/vector2i.h"

ZN_GODOT_NAMESPACE_BEGIN
class Label;
ZN_GODOT_NAMESPACE_END

namespace zylann {

class ZN_PanZoomContainer;

namespace voxel {

// Control to pick one tile of a regular grid by clicking on it, drawn over a texture.
// Used for the manual tile positions of VoxelBlockyModelCube (for atlas tiles, see VoxelBlockyTextureAtlasEditor).
// The texture is only a visual aid and can be null, the grid is still usable. Emits "cell_double_clicked" when a cell is double-clicked.
class VoxelBlockyCubeTilePicker : public MarginContainer {
	GDCLASS(VoxelBlockyCubeTilePicker, MarginContainer)
public:
	VoxelBlockyCubeTilePicker();

	void setup(Ref<Texture2D> texture, const Vector2i grid_size, const Vector2i selected_cell);

	Vector2i get_selected_cell() const {
		return _selected_cell;
	}

private:
	static void _bind_methods();

	void on_texture_rect_draw();
	void on_texture_rect_gui_input(Ref<InputEvent> event);

	// Size of one cell in pixels of the drawn area. Derived from the texture, if any.
	Vector2 get_cell_size() const;
	void update_info();
	void set_hovered_cell(const Vector2i cell);

	Ref<Texture2D> _texture;
	Vector2i _grid_size = Vector2i(1, 1);
	Vector2i _selected_cell = Vector2i(0, 0);
	Vector2i _hovered_cell = Vector2i(-1, -1);

	ZN_PanZoomContainer *_pan_zoom_container = nullptr;
	Control *_texture_rect = nullptr;
	Label *_info_label = nullptr;
};

} // namespace voxel
} // namespace zylann

#endif // VOXEL_BLOCKY_CUBE_TILE_PICKER_H
