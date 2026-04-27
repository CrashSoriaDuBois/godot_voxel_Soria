#include "../../util/containers/fixed_array.h"
#include "../../util/containers/span.h"
#include "voxel_generator_template_generators.h"
//VoxelGeneratorTemplateGenerators
namespace zylann::voxel {

VoxelGeneratorTemplateGenerators::VoxelGeneratorTemplateGenerators() {}

VoxelGeneratorTemplateGenerators::~VoxelGeneratorTemplateGenerators() {}

void VoxelGeneratorTemplateGenerators::set_channel(VoxelBuffer::ChannelId p_channel) {
	ERR_FAIL_INDEX(p_channel, VoxelBuffer::MAX_CHANNELS);
	bool changed = false;
	{
		RWLockWrite wlock(_parameters_lock);
		if (_parameters.channel != p_channel) {
			_parameters.channel = p_channel;
			changed = true;
		}
	}
	if (changed) {
		emit_changed();
	}
}

VoxelBuffer::ChannelId VoxelGeneratorTemplateGenerators::get_channel() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.channel;
}

int VoxelGeneratorTemplateGenerators::get_used_channels_mask() const {
	RWLockRead rlock(_parameters_lock);
	return (1 << _parameters.channel);
}

void VoxelGeneratorTemplateGenerators::set_height_start(float start) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.range.start = start;
}

float VoxelGeneratorTemplateGenerators::get_height_start() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.range.start;
}

void VoxelGeneratorTemplateGenerators::set_height_range(float range) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.range.height = range;
}

float VoxelGeneratorTemplateGenerators::get_height_range() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.range.height;
}

void VoxelGeneratorTemplateGenerators::set_iso_scale(float iso_scale) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.iso_scale = iso_scale;
}

float VoxelGeneratorTemplateGenerators::get_iso_scale() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.iso_scale;
}

void VoxelGeneratorTemplateGenerators::set_offset(const Vector2i offset) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.offset = offset;
}

Vector2i VoxelGeneratorTemplateGenerators::get_offset() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.offset;
}

void VoxelGeneratorTemplateGenerators::_b_set_channel(godot::VoxelBuffer::ChannelId p_channel) {
	set_channel(VoxelBuffer::ChannelId(p_channel));
}

godot::VoxelBuffer::ChannelId VoxelGeneratorTemplateGenerators::_b_get_channel() const {
	return godot::VoxelBuffer::ChannelId(get_channel());
}

void VoxelGeneratorTemplateGenerators::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_channel", "channel"), &VoxelGeneratorTemplateGenerators::_b_set_channel);
	ClassDB::bind_method(D_METHOD("get_channel"), &VoxelGeneratorTemplateGenerators::_b_get_channel);

	ClassDB::bind_method(D_METHOD("set_height_start", "start"), &VoxelGeneratorTemplateGenerators::set_height_start);
	ClassDB::bind_method(D_METHOD("get_height_start"), &VoxelGeneratorTemplateGenerators::get_height_start);

	ClassDB::bind_method(D_METHOD("set_height_range", "range"), &VoxelGeneratorTemplateGenerators::set_height_range);
	ClassDB::bind_method(D_METHOD("get_height_range"), &VoxelGeneratorTemplateGenerators::get_height_range);

	ClassDB::bind_method(D_METHOD("set_iso_scale", "scale"), &VoxelGeneratorTemplateGenerators::set_iso_scale);
	ClassDB::bind_method(D_METHOD("get_iso_scale"), &VoxelGeneratorTemplateGenerators::get_iso_scale);

	ClassDB::bind_method(D_METHOD("set_offset", "offset"), &VoxelGeneratorTemplateGenerators::set_offset);
	ClassDB::bind_method(D_METHOD("get_offset"), &VoxelGeneratorTemplateGenerators::get_offset);

	ADD_PROPERTY(
			PropertyInfo(Variant::INT, "channel", PROPERTY_HINT_ENUM, godot::VoxelBuffer::CHANNEL_ID_HINT_STRING),
			"set_channel",
			"get_channel"
	);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height_start"), "set_height_start", "get_height_start");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height_range"), "set_height_range", "get_height_range");

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2I, "offset"), "set_offset", "get_offset");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "iso_scale"), "set_iso_scale", "get_iso_scale");
}

} // namespace zylann::voxel
