#ifndef VOXEL_BLOCKY_ROTATION_INSPECTOR_PLUGIN_H
#define VOXEL_BLOCKY_ROTATION_INSPECTOR_PLUGIN_H

#include "../../meshers/blocky/voxel_blocky_library.h"
#include "../../meshers/blocky/voxel_blocky_model.h"
#include "../../util/godot/classes/editor_inspector_plugin.h"
#include "../../util/godot/macros.h"

ZN_GODOT_FORWARD_DECLARE(class Label);
ZN_GODOT_FORWARD_DECLARE(class EditorUndoRedoManager);

namespace zylann::voxel {

class VoxelBlockyLibraryIndicesDialog;

// - On models: shows the indices a model occupies in the library it was last inspected from, and a button to
//   create/update the variants of its rotation group.
// - On VoxelBlockyLibrary: adds a button to list the indices of its models, with rotation groups highlighted.
class VoxelBlockyRotationInspectorPlugin : public zylann::godot::ZN_EditorInspectorPlugin {
	GDCLASS(VoxelBlockyRotationInspectorPlugin, zylann::godot::ZN_EditorInspectorPlugin)
public:
	void set_indices_dialog(VoxelBlockyLibraryIndicesDialog *dialog);
	void set_undo_redo(EditorUndoRedoManager *undo_redo);

protected:
	bool _zn_can_handle(const Object *p_object) const override;

	bool _zn_parse_property(
			Object *p_object,
			const Variant::Type p_type,
			const String &p_path,
			const PropertyHint p_hint,
			const String &p_hint_text,
			const BitField<PropertyUsageFlags> p_usage,
			const bool p_wide
	) override;

	void _zn_parse_end(Object *p_object) override;

private:
	void on_update_variants_button_pressed(Ref<VoxelBlockyModel> model, Label *info_label);
	void on_inspect_indices_button_pressed(Ref<VoxelBlockyLibrary> library);

	static void _bind_methods() {}

	VoxelBlockyLibraryIndicesDialog *_indices_dialog = nullptr;
	EditorUndoRedoManager *_undo_redo = nullptr;
};

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_ROTATION_INSPECTOR_PLUGIN_H
