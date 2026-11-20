#ifndef VOXEL_BLOCKY_MODEL_ROTATED_H
#define VOXEL_BLOCKY_MODEL_ROTATED_H

#include "voxel_blocky_model.h"

namespace zylann::voxel {

// A rotated copy of another model, which has a rotation mode (see VoxelBlockyModel::set_rotation_mode).
// These models are created and named by the editor tools, they fill the slots following their source model in the
// library. Everything is taken from the source model, so editing the source updates all its variants.
class VoxelBlockyModelRotated : public VoxelBlockyModel {
	GDCLASS(VoxelBlockyModelRotated, VoxelBlockyModel)
public:
	VoxelBlockyModelRotated();

	void set_source(Ref<VoxelBlockyModel> source);
	Ref<VoxelBlockyModel> get_source() const;

	// Index of this variant in the group of the source model. 0 is the source itself (identity), so variants held by
	// this class start from 1.
	void set_variant(int variant);
	int get_variant() const;

	void bake(blocky::ModelBakingContext &ctx) const override;
	bool is_empty() const override;
	Ref<Mesh> get_preview_mesh() const override;

#ifdef TOOLS_ENABLED
	void get_configuration_warnings(PackedStringArray &out_warnings) const override;
#endif

private:
	static void _bind_methods();

	void _on_source_changed();

	// Returns a null reference if the setup is not valid
	Ref<VoxelBlockyModel> make_rotated_model() const;

	Ref<VoxelBlockyModel> _source;
	uint8_t _variant = 0;
};

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_MODEL_ROTATED_H
