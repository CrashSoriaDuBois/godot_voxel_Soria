#include "voxel_blocky_cube_tile_picker.h"
#include "../../constants/voxel_string_names.h"
#include "../../util/godot/classes/input_event_mouse_button.h"
#include "../../util/godot/classes/input_event_mouse_motion.h"
#include "../../util/godot/classes/label.h"
#include "../../util/godot/classes/v_box_container.h"
#include "../../util/godot/core/mouse_button.h"
#include "../../util/godot/pan_zoom_container.h"

#include <algorithm>
#include <cmath>

namespace zylann::voxel {

namespace {

void draw_grid(CanvasItem &ci, const Vector2i grid_size, const Vector2 cell_size, const Color color) {
	const Vector2 extent = Vector2(grid_size) * cell_size;
	for (int y = 0; y < grid_size.y + 1; ++y) {
		ci.draw_line(Vector2(0, y * cell_size.y), Vector2(extent.x, y * cell_size.y), color);
	}
	for (int x = 0; x < grid_size.x + 1; ++x) {
		ci.draw_line(Vector2(x * cell_size.x, 0), Vector2(x * cell_size.x, extent.y), color);
	}
}

// Returns (-1, -1) if the position is outside of the grid
Vector2i get_cell_at(const Vector2 pos, const Vector2 cell_size, const Vector2i grid_size) {
	if (cell_size.x <= 0.f || cell_size.y <= 0.f) {
		return Vector2i(-1, -1);
	}
	// Floor, so positions slightly outside on the negative side don't land on cell 0
	const Vector2i cell(
			static_cast<int>(std::floor(pos.x / cell_size.x)), //
			static_cast<int>(std::floor(pos.y / cell_size.y))
	);
	if (cell.x < 0 || cell.y < 0 || cell.x >= grid_size.x || cell.y >= grid_size.y) {
		return Vector2i(-1, -1);
	}
	return cell;
}

} // namespace

VoxelBlockyCubeTilePicker::VoxelBlockyCubeTilePicker() {
	const VoxelStringNames &sn = VoxelStringNames::get_singleton();

	VBoxContainer *root = memnew(VBoxContainer);
	root->set_anchors_and_offsets_preset(Control::PRESET_FULL_RECT);

	_info_label = memnew(Label);
	root->add_child(_info_label);

	_pan_zoom_container = memnew(ZN_PanZoomContainer);
	_pan_zoom_container->set_v_size_flags(Control::SIZE_EXPAND_FILL);

	_texture_rect = memnew(Control);
	_texture_rect->set_mouse_filter(Control::MOUSE_FILTER_PASS);
	_texture_rect->set_texture_filter(CanvasItem::TEXTURE_FILTER_NEAREST);
	_texture_rect->connect(sn.draw, callable_mp(this, &VoxelBlockyCubeTilePicker::on_texture_rect_draw));
	_texture_rect->connect(sn.gui_input, callable_mp(this, &VoxelBlockyCubeTilePicker::on_texture_rect_gui_input));
	_texture_rect->set_focus_mode(Control::FOCUS_ALL);
	_pan_zoom_container->add_child(_texture_rect);

	root->add_child(_pan_zoom_container);
	add_child(root);
}

void VoxelBlockyCubeTilePicker::setup(Ref<Texture2D> texture, const Vector2i grid_size, const Vector2i selected_cell) {
	_texture = texture;
	_grid_size = Vector2i(std::max(grid_size.x, 1), std::max(grid_size.y, 1));
	_selected_cell = selected_cell;
	_hovered_cell = Vector2i(-1, -1);

	const Vector2 content_size = Vector2(_grid_size) * get_cell_size();
	_pan_zoom_container->set_content_rect(Rect2(Vector2(), content_size));
	_texture_rect->set_size(content_size);
	_texture_rect->queue_redraw();
	update_info();
}

Vector2 VoxelBlockyCubeTilePicker::get_cell_size() const {
	if (_texture.is_valid()) {
		const Vector2 texture_size = _texture->get_size();
		if (texture_size.x > 0.f && texture_size.y > 0.f) {
			return texture_size / Vector2(_grid_size);
		}
	}
	// No texture: still allow picking tiles on a plain grid
	return Vector2(32, 32);
}

void VoxelBlockyCubeTilePicker::update_info() {
	String text = String("Grid ") + String::num_int64(_grid_size.x) + "x" + String::num_int64(_grid_size.y) +
			"   Selected: (" + String::num_int64(_selected_cell.x) + ", " + String::num_int64(_selected_cell.y) + ")";
	if (_texture.is_null()) {
		text += "   (no preview texture)";
	}
	_info_label->set_text(text);
}

void VoxelBlockyCubeTilePicker::set_hovered_cell(const Vector2i cell) {
	if (cell != _hovered_cell) {
		_hovered_cell = cell;
		_texture_rect->queue_redraw();
	}
}

void VoxelBlockyCubeTilePicker::on_texture_rect_gui_input(Ref<InputEvent> event) {
	const Vector2 cell_size = get_cell_size();

	Ref<InputEventMouseMotion> mouse_motion_event = event;
	if (mouse_motion_event.is_valid()) {
		set_hovered_cell(get_cell_at(mouse_motion_event->get_position(), cell_size, _grid_size));
		return;
	}

	Ref<InputEventMouseButton> mouse_button_event = event;
	if (mouse_button_event.is_valid() && mouse_button_event->is_pressed() &&
		mouse_button_event->get_button_index() == ZN_GODOT_MouseButton_LEFT) {
		const Vector2i cell = get_cell_at(mouse_button_event->get_position(), cell_size, _grid_size);
		if (cell.x >= 0) {
			_selected_cell = cell;
			_texture_rect->queue_redraw();
			update_info();
			accept_event();
			if (mouse_button_event->is_double_click()) {
				emit_signal("cell_double_clicked");
			}
		}
	}
}

void VoxelBlockyCubeTilePicker::on_texture_rect_draw() {
	CanvasItem &ci = *_texture_rect;

	const Vector2 cell_size = get_cell_size();
	const Vector2 content_size = Vector2(_grid_size) * cell_size;

	if (_texture.is_valid()) {
		ci.draw_texture(_texture, Vector2());
	} else {
		ci.draw_rect(Rect2(Vector2(), content_size), Color(0.15f, 0.15f, 0.15f), true);
	}

	draw_grid(ci, _grid_size, cell_size, Color(0.f, 0.f, 0.f, 0.25f));

	if (_selected_cell.x >= 0 && _selected_cell.y >= 0 && _selected_cell.x < _grid_size.x &&
		_selected_cell.y < _grid_size.y) {
		const Rect2 rect(Vector2(_selected_cell) * cell_size, cell_size);
		ci.draw_rect(rect, Color(1.0f, 0.8f, 0.2f, 0.3f), true);
		ci.draw_rect(rect, Color(1.0f, 0.8f, 0.2f), false, 3.f);
	}

	if (_hovered_cell.x >= 0) {
		ci.draw_rect(Rect2(Vector2(_hovered_cell) * cell_size, cell_size), Color(0.5f, 0.5f, 1.0f, 0.6f), false, 2.f);
	}
}

void VoxelBlockyCubeTilePicker::_bind_methods() {
	ADD_SIGNAL(MethodInfo("cell_double_clicked"));
}

} // namespace zylann::voxel
