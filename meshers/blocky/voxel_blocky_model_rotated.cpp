#include "voxel_blocky_model_rotated.h"
#include "../../constants/voxel_string_names.h"
#include "../../util/math/ortho_basis.h"
#include "blocky_model_baking_context.h"
#include "blocky_rotation_variants.h"

namespace zylann::voxel {

VoxelBlockyModelRotated::VoxelBlockyModelRotated() {
	// Like VoxelBlockyModelEmpty: the default box collision of the base class must not leak into invalid variants
	set_collision_aabbs(Span<const AABB>());
}

void VoxelBlockyModelRotated::set_source(Ref<VoxelBlockyModel> source) {
	if (source == _source) {
		return;
	}
	const VoxelStringNames &sn = VoxelStringNames::get_singleton();

	if (_source.is_valid()) {
		_source->disconnect(sn.changed, callable_mp(this, &VoxelBlockyModelRotated::_on_source_changed));
	}

	_source = source;

	if (_source.is_valid()) {
		_source->connect(sn.changed, callable_mp(this, &VoxelBlockyModelRotated::_on_source_changed));
	}

	emit_changed();
}

Ref<VoxelBlockyModel> VoxelBlockyModelRotated::get_source() const {
	return _source;
}

void VoxelBlockyModelRotated::set_variant(int variant) {
	ZN_ASSERT_RETURN(variant >= 0 && variant < math::ORTHOGONAL_BASIS_COUNT);
	if (variant != _variant) {
		_variant = variant;
		emit_changed();
	}
}

int VoxelBlockyModelRotated::get_variant() const {
	return _variant;
}

Ref<VoxelBlockyModel> VoxelBlockyModelRotated::make_rotated_model() const {
	if (_source.is_null()) {
		return Ref<VoxelBlockyModel>();
	}
	// Variants of variants are not supported
	if (Object::cast_to<VoxelBlockyModelRotated>(_source.ptr()) != nullptr) {
		return Ref<VoxelBlockyModel>();
	}
	const blocky::RotationMode mode = static_cast<blocky::RotationMode>(_source->get_rotation_mode());
	if (mode == blocky::ROTATION_MODE_NONE || _variant >= blocky::get_rotation_variant_count(mode)) {
		return Ref<VoxelBlockyModel>();
	}

	Ref<VoxelBlockyModel> model = _source->duplicate();
	ZN_ASSERT_RETURN_V(model.is_valid(), Ref<VoxelBlockyModel>());

	// Note: `set_mesh_ortho_rotation_index` is not enough, it would not rotate collision boxes.
	// `rotate_ortho` composes the rotation with the one the source already has.
	const uint8_t ortho_index = blocky::get_rotation_variant_ortho_index(mode, _variant);
	if (ortho_index != math::ORTHOGONAL_BASIS_IDENTITY_INDEX) {
		model->rotate_ortho(math::get_ortho_basis_from_index(ortho_index));
	}
	return model;
}

void VoxelBlockyModelRotated::bake(blocky::ModelBakingContext &ctx) const {
	Ref<VoxelBlockyModel> model = make_rotated_model();
	if (model.is_null()) {
		// Invalid setup, reported by configuration warnings. Bake nothing.
		ctx.model.clear();
		VoxelBlockyModel::bake(ctx);
		return;
	}
	model->bake(ctx);
	// The copy doesn't know where it is in the group
	ctx.model.rotation_variant = _variant;
}

bool VoxelBlockyModelRotated::is_empty() const {
	return _source.is_null() || _source->is_empty();
}

Ref<Mesh> VoxelBlockyModelRotated::get_preview_mesh() const {
	Ref<VoxelBlockyModel> model = make_rotated_model();
	if (model.is_null()) {
		return Ref<Mesh>();
	}
	return model->get_preview_mesh();
}

void VoxelBlockyModelRotated::_on_source_changed() {
	emit_changed();
}

#ifdef TOOLS_ENABLED

void VoxelBlockyModelRotated::get_configuration_warnings(PackedStringArray &out_warnings) const {
	out_warnings.append(String("This model is generated from the rotation group of its source model. Edit the source "
							   "model instead: changes made here are ignored."));
	if (_source.is_null()) {
		out_warnings.append(String("No source model assigned."));
	} else if (_source->get_rotation_mode() == blocky::ROTATION_MODE_NONE) {
		out_warnings.append(String("The source model has no rotation mode."));
	}
}

#endif

void VoxelBlockyModelRotated::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_source", "source"), &VoxelBlockyModelRotated::set_source);
	ClassDB::bind_method(D_METHOD("get_source"), &VoxelBlockyModelRotated::get_source);

	ClassDB::bind_method(D_METHOD("set_variant", "variant"), &VoxelBlockyModelRotated::set_variant);
	ClassDB::bind_method(D_METHOD("get_variant"), &VoxelBlockyModelRotated::get_variant);

	ADD_PROPERTY(
			PropertyInfo(Variant::OBJECT, "source", PROPERTY_HINT_RESOURCE_TYPE, VoxelBlockyModel::get_class_static()),
			"set_source",
			"get_source"
	);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "variant", PROPERTY_HINT_RANGE, "0,23,1"), "set_variant", "get_variant");
}

} // namespace zylann::voxel
