#include "voxel_blocky_rotation_inspector_plugin.h"
#include "../../meshers/blocky/blocky_rotation_variants.h"
#include "../../meshers/blocky/voxel_blocky_model_rotated.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/classes/button.h"
#include "../../util/godot/classes/editor_undo_redo_manager.h"
#include "../../util/godot/classes/label.h"
#include "../../util/godot/classes/v_box_container.h"
#include "../../util/godot/core/array.h"
#include "../../util/godot/core/string.h"
#include "voxel_blocky_editor_library_tracker.h"
#include "voxel_blocky_library_indices_dialog.h"

namespace zylann::voxel {

namespace {

// Models don't know which library they belong to. The last inspected one is used.
Ref<VoxelBlockyLibrary> get_inspected_simple_library() {
	const Ref<VoxelBlockyLibraryBase> library = get_last_inspected_blocky_library();
	return Ref<VoxelBlockyLibrary>(Object::cast_to<VoxelBlockyLibrary>(library.ptr()));
}

// Returns -1 if not found
int find_model_index(const VoxelBlockyLibrary &library, const VoxelBlockyModel &model) {
	const unsigned int count = library.get_model_count();
	for (unsigned int i = 0; i < count; ++i) {
		if (library.get_model(i).ptr() == &model) {
			return i;
		}
	}
	return -1;
}

String make_variant_name(const String &base_name, const unsigned int ortho_index) {
	return base_name + " [r" + String::num_int64(ortho_index) + "]";
}

String get_group_info_text(const VoxelBlockyModel &model) {
	const Ref<VoxelBlockyLibrary> library = get_inspected_simple_library();
	if (library.is_null()) {
		return "Library unknown: select the library first to see the index of this model.";
	}
	const int index = find_model_index(**library, model);
	if (index < 0) {
		return "This model is not in the last inspected library.";
	}

	const VoxelBlockyModelRotated *variant = Object::cast_to<VoxelBlockyModelRotated>(&model);
	if (variant != nullptr) {
		return String("Index ") + String::num_int64(index) + ": variant " + String::num_int64(variant->get_variant()) +
				" of the group starting at index " + String::num_int64(index - variant->get_variant());
	}

	const blocky::RotationMode mode = static_cast<blocky::RotationMode>(model.get_rotation_mode());
	if (mode == blocky::ROTATION_MODE_NONE) {
		return String("Index ") + String::num_int64(index) + ". No rotation group.";
	}
	const unsigned int count = blocky::get_rotation_variant_count(mode);
	return String("Occupies indices ") + String::num_int64(index) + "-" + String::num_int64(index + count - 1) + " (" +
			blocky::get_rotation_mode_name(mode) + ")";
}

} // namespace

void VoxelBlockyRotationInspectorPlugin::set_indices_dialog(VoxelBlockyLibraryIndicesDialog *dialog) {
	_indices_dialog = dialog;
}

void VoxelBlockyRotationInspectorPlugin::set_undo_redo(EditorUndoRedoManager *undo_redo) {
	_undo_redo = undo_redo;
}

bool VoxelBlockyRotationInspectorPlugin::_zn_can_handle(const Object *p_object) const {
	return Object::cast_to<VoxelBlockyModel>(p_object) != nullptr ||
			Object::cast_to<VoxelBlockyLibrary>(p_object) != nullptr;
}

bool VoxelBlockyRotationInspectorPlugin::_zn_parse_property(
		Object *p_object,
		const Variant::Type p_type,
		const String &p_path,
		const PropertyHint p_hint,
		const String &p_hint_text,
		const BitField<PropertyUsageFlags> p_usage,
		const bool p_wide
) {
	// Controls are added right above the `rotation_mode` property. The default property editor is kept.
	if (p_path != "rotation_mode") {
		return false;
	}
	const VoxelBlockyModel *model_ptr = Object::cast_to<VoxelBlockyModel>(p_object);
	if (model_ptr == nullptr) {
		return false;
	}
	Ref<VoxelBlockyModel> model(model_ptr);

	VBoxContainer *box = memnew(VBoxContainer);

	Label *label = memnew(Label);
	label->set_text(get_group_info_text(**model));
	box->add_child(label);

	// Variants are generated from their source model
	if (Object::cast_to<VoxelBlockyModelRotated>(model_ptr) == nullptr) {
		Button *button = memnew(Button);
		button->set_text(ZN_TTR("Update rotation variants"));
		button->set_tooltip_text(
				ZN_TTR("Creates, renames or removes the models following this one, to match its rotation mode.")
		);
		button->connect(
				"pressed",
				callable_mp(this, &VoxelBlockyRotationInspectorPlugin::on_update_variants_button_pressed)
						.bind(model, label)
		);
		box->add_child(button);
	}

	add_custom_control(box);
	return false;
}

void VoxelBlockyRotationInspectorPlugin::_zn_parse_end(Object *p_object) {
	const VoxelBlockyLibrary *library_ptr = Object::cast_to<VoxelBlockyLibrary>(p_object);
	if (library_ptr == nullptr) {
		return;
	}
	Ref<VoxelBlockyLibrary> library(library_ptr);

	Button *button = memnew(Button);
	button->set_text(ZN_TTR("Inspect indices..."));
	button->connect(
			"pressed",
			callable_mp(this, &VoxelBlockyRotationInspectorPlugin::on_inspect_indices_button_pressed).bind(library)
	);
	add_custom_control(button);
}

void VoxelBlockyRotationInspectorPlugin::on_inspect_indices_button_pressed(Ref<VoxelBlockyLibrary> library) {
	ZN_ASSERT_RETURN(_indices_dialog != nullptr);
	_indices_dialog->set_library(library);
	_indices_dialog->popup_centered();
}

void VoxelBlockyRotationInspectorPlugin::on_update_variants_button_pressed(
		Ref<VoxelBlockyModel> model,
		Label *info_label
) {
	ZN_ASSERT_RETURN(_undo_redo != nullptr);
	ZN_ASSERT_RETURN(model.is_valid());

	Ref<VoxelBlockyLibrary> library = get_inspected_simple_library();
	if (library.is_null()) {
		ERR_PRINT("Cannot update rotation variants: select the library first, so it is known which one to edit.");
		return;
	}

	const int model_index = find_model_index(**library, **model);
	if (model_index < 0) {
		ERR_PRINT("Cannot update rotation variants: this model is not in the last inspected library.");
		return;
	}

	const blocky::RotationMode mode = static_cast<blocky::RotationMode>(model->get_rotation_mode());
	const unsigned int variant_count = blocky::get_rotation_variant_count(mode);

	// Indices of models must never change (they are voxel IDs), so slots are never inserted or removed: variants are
	// written into free (null) slots, or slots they already own. Anything else is a conflict.
	Array old_models = library->get_models_array().duplicate();
	Array new_models = library->get_models_array().duplicate();

	// 1. Variants of this model that are no longer wanted or are not where they should be get removed
	for (int i = 0; i < new_models.size(); ++i) {
		Ref<VoxelBlockyModelRotated> variant = new_models[i];
		if (variant.is_valid() && variant->get_source() == model) {
			const int v = variant->get_variant();
			if (i != model_index + v || v == 0 || v >= int(variant_count)) {
				new_models[i] = Ref<VoxelBlockyModel>();
			}
		}
	}

	// 2. Fill the slots following the model
	String base_name = model->get_name();
	if (base_name.is_empty()) {
		base_name = String("model_") + String::num_int64(model_index);
	}

	StdVector<int> conflicts;

	for (unsigned int v = 1; v < variant_count; ++v) {
		const int slot = model_index + v;
		if (slot >= new_models.size()) {
			new_models.resize(slot + 1);
		}

		const Ref<VoxelBlockyModel> current = new_models[slot];
		Ref<VoxelBlockyModelRotated> variant = current;

		const bool is_ours = variant.is_valid() && variant->get_source() == model && variant->get_variant() == int(v);
		if (current.is_valid() && !is_ours) {
			conflicts.push_back(slot);
			continue;
		}

		if (variant.is_null()) {
			variant.instantiate();
			variant->set_source(model);
			variant->set_variant(v);
			new_models[slot] = variant;
		}

		variant->set_name(make_variant_name(base_name, blocky::get_rotation_variant_ortho_index(mode, v)));
	}

	if (conflicts.size() > 0) {
		String text = "Cannot update rotation variants: these slots are not free: ";
		for (size_t i = 0; i < conflicts.size(); ++i) {
			if (i > 0) {
				text += ", ";
			}
			text += String::num_int64(conflicts[i]);
		}
		ERR_PRINT(text);
		if (info_label != nullptr) {
			info_label->set_text(text);
		}
		return;
	}

	EditorUndoRedoManager &ur = *_undo_redo;
	ur.create_action("Update rotation variants");
	ur.add_do_property(library.ptr(), "models", new_models);
	ur.add_undo_property(library.ptr(), "models", old_models);
	ur.commit_action();

	if (info_label != nullptr) {
		info_label->set_text(get_group_info_text(**model));
	}
}

} // namespace zylann::voxel
