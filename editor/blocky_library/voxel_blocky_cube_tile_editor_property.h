#ifndef VOXEL_BLOCKY_CUBE_TILE_EDITOR_PROPERTY_H
#define VOXEL_BLOCKY_CUBE_TILE_EDITOR_PROPERTY_H

#include "../../util/godot/classes/editor_property.h"
#include "../../util/godot/classes/texture_2d.h"
#include "../../util/godot/macros.h"
#include "../../util/math/vector2i.h"

ZN_GODOT_NAMESPACE_BEGIN
class Button;
ZN_GODOT_NAMESPACE_END

namespace zylann::voxel {

class VoxelBlockyLibraryEditorPlugin;

// Inspector control for the manual tile position (Vector2i) of a face of a VoxelBlockyModelCube.
// Shows a preview of the selected tile. When clicked, opens a dialog to pick a tile on a grid.
// Counterpart of VoxelBlockyTileEditorProperty, which does the same for atlas tiles.
class VoxelBlockyCubeTileEditorProperty : public zylann::godot::ZN_EditorProperty {
	GDCLASS(VoxelBlockyCubeTileEditorProperty, zylann::godot::ZN_EditorProperty);

public:
	VoxelBlockyCubeTileEditorProperty();

	void setup(VoxelBlockyLibraryEditorPlugin *plugin);
	void on_tile_picker_confirmed(const Vector2i selected_tile);

protected:
	void _zn_set_read_only(bool p_read_only) override;
	void _zn_update_property() override;

	void on_button_pressed();

private:
	static void _bind_methods();

	struct Resolved {
		Ref<Texture2D> texture;
		Vector2i grid_size = Vector2i(16, 16);
	};

	// Takes the model's own values or the library's ones, depending on the model's settings
	bool resolve(const Object &obj, Resolved &out_resolved) const;
	void update_button(const Object &obj, const Vector2i tile);

	Button *_button = nullptr;
	VoxelBlockyLibraryEditorPlugin *_plugin = nullptr;
};

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_CUBE_TILE_EDITOR_PROPERTY_H
