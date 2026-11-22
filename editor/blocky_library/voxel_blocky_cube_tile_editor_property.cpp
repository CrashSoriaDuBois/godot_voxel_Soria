#include "voxel_blocky_cube_tile_editor_property.h"
#include "../../meshers/blocky/voxel_blocky_library_base.h"
#include "../../meshers/blocky/voxel_blocky_model_cube.h"
#include "../../util/godot/classes/atlas_texture.h"
#include "../../util/godot/classes/button.h"
#include "../../util/godot/editor_scale.h"
#include "voxel_blocky_editor_library_tracker.h"
#include "voxel_blocky_library_editor_plugin.h"
#include "../../meshers/blocky/voxel_blocky_model_mesh.h"

namespace zylann::voxel {

VoxelBlockyCubeTileEditorProperty::VoxelBlockyCubeTileEditorProperty() {
	_button = memnew(Button);
	_button->set_expand_icon(true);
	_button->connect("pressed", callable_mp(this, &VoxelBlockyCubeTileEditorProperty::on_button_pressed));
	add_child(_button);
}

void VoxelBlockyCubeTileEditorProperty::setup(VoxelBlockyLibraryEditorPlugin *plugin) {
	_plugin = plugin;
}

void VoxelBlockyCubeTileEditorProperty::_zn_set_read_only(bool p_read_only) {
	_button->set_disabled(p_read_only);
}

void VoxelBlockyCubeTileEditorProperty::_zn_update_property() {
	const Vector2i tile = get_edited_property_value();

	Object *obj = get_edited_object();
	ERR_FAIL_COND(obj == nullptr);
	update_button(*obj, tile);
}

bool VoxelBlockyCubeTileEditorProperty::resolve(const Object &obj, Resolved &out_resolved) const {
	// Cubes and meshes have the same tile settings
	const VoxelBlockyModelCube *cube = Object::cast_to<VoxelBlockyModelCube>(&obj);

	const VoxelBlockyModelMesh *mesh = Object::cast_to<VoxelBlockyModelMesh>(&obj);

	ERR_FAIL_COND_V(cube == nullptr && mesh == nullptr, false);

	// Models don't know their library. Without one, the defaults are assumed.
	const Ref<VoxelBlockyLibraryBase> library = get_last_inspected_blocky_library();
	const Vector2i library_size = library.is_valid() ? library->get_cube_atlas_size_in_tiles() : Vector2i(16, 16);
	const Ref<Texture2D> library_texture = library.is_valid() ? library->get_cube_preview_texture() : Ref<Texture2D>();

	if (cube != nullptr) {
		out_resolved.grid_size = cube->get_effective_atlas_size_in_tiles(library_size);
		out_resolved.texture = cube->get_effective_preview_texture(library_texture);
	} else {
		out_resolved.grid_size = mesh->get_effective_atlas_size_in_tiles(library_size);
		out_resolved.texture = mesh->get_effective_preview_texture(library_texture);
	}
	return true;
}

void VoxelBlockyCubeTileEditorProperty::update_button(const Object &obj, const Vector2i tile) {
	Resolved resolved;
	if (!resolve(obj, resolved)) {
		return;
	}

	const String title = String("(") + String::num_int64(tile.x) + ", " + String::num_int64(tile.y) + ")";
	String tooltip;
	Ref<AtlasTexture> icon;

	const bool inside_grid =
			tile.x >= 0 && tile.y >= 0 && tile.x < resolved.grid_size.x && tile.y < resolved.grid_size.y;

	if (!inside_grid) {
		tooltip = "This tile is outside of the grid";
	} else if (resolved.texture.is_null()) {
		tooltip = "No preview texture. Assign one on the library, or on the model.";
	} else {
		// Preview of the selected tile
		const Vector2 cell_size = resolved.texture->get_size() / Vector2(resolved.grid_size);
		icon.instantiate();
		icon->set_atlas(resolved.texture);
		icon->set_region(Rect2(Vector2(tile) * cell_size, cell_size));
	}

	_button->set_text(title);
	zylann::godot::set_button_icon(*_button, icon);
	_button->set_tooltip_text(tooltip);
}

void VoxelBlockyCubeTileEditorProperty::on_button_pressed() {
	Object *obj = get_edited_object();
	ZN_ASSERT_RETURN(obj != nullptr);
	ZN_ASSERT_RETURN(_plugin != nullptr);

	Resolved resolved;
	if (!resolve(*obj, resolved)) {
		return;
	}

	const Vector2i tile = get_edited_property_value();
	_plugin->open_cube_tile_picker(resolved.texture, resolved.grid_size, tile, this);
}

void VoxelBlockyCubeTileEditorProperty::on_tile_picker_confirmed(const Vector2i selected_tile) {
	emit_changed(get_edited_property(), selected_tile, "");
}

void VoxelBlockyCubeTileEditorProperty::_bind_methods() {
	//
}

} // namespace zylann::voxel
