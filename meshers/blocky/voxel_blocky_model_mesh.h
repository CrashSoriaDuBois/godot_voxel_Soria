#ifndef VOXEL_BLOCKY_MODEL_MESH_H
#define VOXEL_BLOCKY_MODEL_MESH_H

#include "voxel_blocky_model.h"
#include "blocky_tile_slots.h"
#include "voxel_blocky_texture_atlas.h"

namespace zylann::voxel {

// Model using a mesh for visuals
class VoxelBlockyModelMesh : public VoxelBlockyModel {
	GDCLASS(VoxelBlockyModelMesh, VoxelBlockyModel)
public:
	void set_mesh(Ref<Mesh> mesh);
	Ref<Mesh> get_mesh() const {
		return _mesh;
	}

	void bake(blocky::ModelBakingContext &ctx) const override;
	bool is_empty() const override;

	Ref<Mesh> get_preview_mesh() const override;

	void set_side_vertex_tolerance(float tolerance);
	float get_side_vertex_tolerance() const;

	void set_side_cutout_enabled(bool enabled);
	bool is_side_cutout_enabled() const;

	// If enabled, triangles of the inside of the model that use dynamic tiles (blob, random, extended) get connected like sides do, for the direction they
	// face (stairs, ladders...). Their UVs must be authored like the ones of the side facing the same direction. It costs more work when meshing voxels of this model.
	void set_inner_connected_tiles(bool enabled);
	bool is_inner_connected_tiles() const;

	VoxelBlockyModelMesh();

	// Tiles
	void set_tile(int slot, Vector2i pos);
	Vector2i get_tile(int slot) const;

	// Atlas tiles
	void set_atlas(Ref<VoxelBlockyTextureAtlas> atlas);
	Ref<VoxelBlockyTextureAtlas> get_atlas() const;

	void set_atlas_tile_id(int slot, int tile_id);
	int get_atlas_tile_id(int slot) const;

	// Same settings as VoxelBlockyModelCube
	void set_atlas_size_in_tiles(Vector2i s);
	Vector2i get_atlas_size_in_tiles() const;

	void set_use_library_tile_size(bool enabled);
	bool get_use_library_tile_size() const;

	void set_use_library_texture(bool enabled);
	bool get_use_library_texture() const;

	void set_preview_texture(Ref<Texture2D> texture);
	Ref<Texture2D> get_preview_texture() const;

	Vector2i get_effective_atlas_size_in_tiles(const Vector2i library_size) const;

	Ref<Texture2D> get_effective_preview_texture(const Ref<Texture2D> &library_texture) const;

	// Resolves tiles for baking
	void make_tile_slot_mapping(
			blocky::TileSlotMapping &mapping,
			blocky::AtlasIndexer &atlas_indexer,
			const Vector2i library_size
	) const;

	// Library-aware preview
	Ref<Mesh> get_preview_mesh_with_library_size(const Vector2i library_size) const;

private:
	static void _bind_methods();

	Ref<Mesh> _mesh;
	// Margin near sides of the voxel where triangles will be considered to be "on the side". Those triangles will
	// be processed by the neighbor side culling system.
	float _side_vertex_tolerance = 0.001f;
	bool _side_cutout_enabled = false;
	bool _inner_connected_tiles = false;

	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

	Ref<VoxelBlockyTextureAtlas> _atlas;

	FixedArray<Vector2i, blocky::TILE_SLOT_COUNT> _tiles;
	FixedArray<int, blocky::TILE_SLOT_COUNT> _atlas_tiles;

	Vector2i _atlas_size_in_tiles = Vector2i(16, 16);

	bool _use_library_tile_size = true;
	bool _use_library_texture = true;

	Ref<Texture2D> _preview_texture;
};

void rotate_mesh_arrays(Span<Vector3f> vertices, Span<Vector3f> normals, Span<float> tangents, const Basis3f &basis);

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_MODEL_MESH_H
