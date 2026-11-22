#include "voxel_blocky_tile_selection_inspector_plugin.h"
#include "../../constants/voxel_string_names.h"
#include "../../meshers/blocky/voxel_blocky_model_cube.h"
#include "../../meshers/blocky/voxel_blocky_texture_atlas.h"
#include "voxel_blocky_cube_tile_editor_property.h"
#include "voxel_blocky_editor_library_tracker.h"
#include "voxel_blocky_tile_editor_property.h"
#include "../../meshers/blocky/voxel_blocky_model_mesh.h"

namespace zylann::voxel {

void VoxelBlockyTileSelectionInspectorPlugin::set_plugin(VoxelBlockyLibraryEditorPlugin *plugin) {
	_plugin = plugin;
}

bool VoxelBlockyTileSelectionInspectorPlugin::_zn_can_handle(const Object *p_object) const {
	const VoxelBlockyLibraryBase *library = Object::cast_to<VoxelBlockyLibraryBase>(p_object);
	if (library != nullptr) {
		set_last_inspected_blocky_library(const_cast<VoxelBlockyLibraryBase *>(library));
		return false;
	}
	return Object::cast_to<VoxelBlockyModelCube>(p_object) != nullptr ||
			Object::cast_to<VoxelBlockyModelMesh>(p_object) != nullptr;
}

bool VoxelBlockyTileSelectionInspectorPlugin::_zn_parse_property(
		Object *p_object,
		const Variant::Type p_type,
		const String &p_path,
		const PropertyHint p_hint,
		const String &p_hint_text,
		const BitField<PropertyUsageFlags> p_usage,
		const bool p_wide
) {
	// Cubes and meshes have the same tile settings
	Ref<VoxelBlockyTextureAtlas> atlas;

	if (const VoxelBlockyModelCube *cube = Object::cast_to<VoxelBlockyModelCube>(p_object)) {
		atlas = cube->get_atlas();

	} else if (const VoxelBlockyModelMesh *mesh = Object::cast_to<VoxelBlockyModelMesh>(p_object)) {
		atlas = mesh->get_atlas();

	} else {
		return false;
	}
	if (atlas.is_null()) {
		// Manual tile positions: show a preview of the tile and pick it on a grid
		if (p_type == Variant::VECTOR2I && p_path.begins_with("tile_")) {
			VoxelBlockyCubeTileEditorProperty *ed = memnew(VoxelBlockyCubeTileEditorProperty);
			ed->setup(_plugin);
			add_property_editor(p_path, ed);
			return true;
		}
		return false;
	}

	if (p_path.begins_with("atlas_tile_id_")) {
		const int tile_id = p_object->get(p_path);

		// Replace default editor with this one
		VoxelBlockyTileEditorProperty *ed = memnew(VoxelBlockyTileEditorProperty);
		ed->setup(p_object, VoxelStringNames::get_singleton().atlas, tile_id, _plugin);
		add_property_editor(p_path, ed);
		return true;
	}

	return false;
}

} // namespace zylann::voxel
