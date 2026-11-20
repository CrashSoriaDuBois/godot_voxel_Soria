#include "voxel_blocky_library_indices_dialog.h"
#include "../../meshers/blocky/blocky_rotation_variants.h"
#include "../../meshers/blocky/voxel_blocky_model_rotated.h"
#include "../../util/godot/classes/button.h"
#include "../../util/godot/classes/item_list.h"
#include "../../util/godot/classes/v_box_container.h"
#include "../../util/godot/core/string.h"
#include "../../util/godot/editor_scale.h"

#include <cmath>

namespace zylann::voxel {

VoxelBlockyLibraryIndicesDialog::VoxelBlockyLibraryIndicesDialog() {
	const float editor_scale = EDSCALE;

	set_title(ZN_TTR("VoxelBlockyLibrary model indices"));
	set_min_size(Vector2(400, 400) * editor_scale);

	VBoxContainer *v_box_container = memnew(VBoxContainer);
	v_box_container->set_anchor(SIDE_RIGHT, 1);
	v_box_container->set_anchor(SIDE_BOTTOM, 1);
	v_box_container->set_offset(SIDE_LEFT, 4 * editor_scale);
	v_box_container->set_offset(SIDE_TOP, 4 * editor_scale);
	v_box_container->set_offset(SIDE_RIGHT, -4 * editor_scale);
	v_box_container->set_offset(SIDE_BOTTOM, -4 * editor_scale);

	_item_list = memnew(ItemList);
	_item_list->set_v_size_flags(Control::SIZE_EXPAND_FILL);
	v_box_container->add_child(_item_list);

	Button *button = get_ok_button();
	button->set_custom_minimum_size(Vector2(100 * EDSCALE, 0));

	add_child(v_box_container);
}

void VoxelBlockyLibraryIndicesDialog::set_library(Ref<VoxelBlockyLibrary> library) {
	ZN_ASSERT_RETURN(library.is_valid());

	_item_list->clear();

	const unsigned int model_count = library->get_model_count();

	// Rotation groups are drawn with a background color, to see which indices they occupy
	unsigned int group_counter = 0;
	unsigned int group_end = 0; // Exclusive
	Color group_color;

	for (unsigned int i = 0; i < model_count; ++i) {
		const Ref<VoxelBlockyModel> model = library->get_model(i);

		String text = String::num_int64(i) + ": ";

		if (model.is_null()) {
			text += "(empty)";
			const int item_index = _item_list->add_item(text);
			_item_list->set_item_custom_fg_color(item_index, Color(0.5, 0.5, 0.5));
			continue;
		}

		String name = model->get_name();
		if (name.is_empty()) {
			name = String("<unnamed ") + model->get_class() + ">";
		}
		text += name;

		const bool is_variant = Object::cast_to<VoxelBlockyModelRotated>(model.ptr()) != nullptr;
		const blocky::RotationMode mode = static_cast<blocky::RotationMode>(model->get_rotation_mode());

		if (!is_variant && mode != blocky::ROTATION_MODE_NONE) {
			const unsigned int count = blocky::get_rotation_variant_count(mode);
			group_end = i + count;
			++group_counter;
			group_color = Color::from_hsv(std::fmod(group_counter * 0.17f, 1.f), 0.35f, 0.4f);
			text += String("   [") + blocky::get_rotation_mode_name(mode) + ", indices " + String::num_int64(i) + "-" +
					String::num_int64(i + count - 1) + "]";
		}

		const int item_index = _item_list->add_item(text);

		if (i < group_end) {
			_item_list->set_item_custom_bg_color(item_index, group_color);
		}
	}
}

} // namespace zylann::voxel
