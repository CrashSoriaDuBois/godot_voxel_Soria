#ifndef VOXEL_BLOCKY_MODEL_EDITOR_INSPECTOR_PLUGIN_H
#define VOXEL_BLOCKY_MODEL_EDITOR_INSPECTOR_PLUGIN_H

#include "../../util/godot/classes/editor_inspector_plugin.h"
#include "../../meshers/blocky/voxel_blocky_model.h"
#include "../../util/godot/macros.h"

ZN_GODOT_FORWARD_DECLARE(class EditorUndoRedoManager);
ZN_GODOT_FORWARD_DECLARE(class LineEdit);

namespace zylann::voxel {

class VoxelBlockyModelEditorInspectorPlugin : public zylann::godot::ZN_EditorInspectorPlugin {
	GDCLASS(VoxelBlockyModelEditorInspectorPlugin, zylann::godot::ZN_EditorInspectorPlugin)
public:
	// TODO GDX: `EditorUndoRedoManager` isn't a singleton yet in GDExtension, so it has to be injected
	void set_undo_redo(EditorUndoRedoManager *urm);

protected:
	bool _zn_can_handle(const Object *p_object) const override;
	void _zn_parse_begin(Object *p_object) override;

private:
	// When compiling with GodotCpp, `_bind_methods` isn't optional.
	static void _bind_methods() {}
	// Name editor, at the top of the inspector.
	void on_name_text_submitted(const String &text, Ref<VoxelBlockyModel> model);
	void on_name_focus_exited(Ref<VoxelBlockyModel> model, LineEdit *line_edit);
	void apply_name(Ref<VoxelBlockyModel> model, const String &new_name);

	EditorUndoRedoManager *_undo_redo = nullptr;
};

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_MODEL_EDITOR_INSPECTOR_PLUGIN_H
