#include "blocky_tile_slots.h"
#include "../../util/math/conv.h"
#include "blocky_atlas_indexer.h"

#include <cstring>
#include <limits>

namespace zylann::voxel::blocky {

const char *get_tile_slot_name(const unsigned int slot) {
	static const char *names[TILE_SLOT_COUNT] = {
		"red", "green", "blue", "yellow", "cyan", "magenta", "white", "black"
	};
	ZN_ASSERT_RETURN_V(slot < TILE_SLOT_COUNT, "");
	return names[slot];
}

int get_tile_slot_from_name(const char *name) {
	for (unsigned int slot = 0; slot < TILE_SLOT_COUNT; ++slot) {
		if (strcmp(name, get_tile_slot_name(slot)) == 0) {
			return slot;
		}
	}
	return -1;
}

unsigned int get_tile_slot_from_color(const Color &color) {
	// Bits: red = 1, green = 2, blue = 4
	const unsigned int bits = (color.r > 0.5f ? 1 : 0) | (color.g > 0.5f ? 2 : 0) | (color.b > 0.5f ? 4 : 0);
	static const uint8_t bits_to_slot[8] = {
		TILE_SLOT_BLACK, // 0
		TILE_SLOT_RED, // 1
		TILE_SLOT_GREEN, // 2
		TILE_SLOT_YELLOW, // 1 + 2
		TILE_SLOT_BLUE, // 4
		TILE_SLOT_MAGENTA, // 1 + 4
		TILE_SLOT_CYAN, // 2 + 4
		TILE_SLOT_WHITE, // 1 + 2 + 4
	};
	return bits_to_slot[bits];
}

TileSlotMapping::TileSlotMapping() {
	for (unsigned int slot = 0; slot < TILE_SLOT_COUNT; ++slot) {
		uv_offsets[slot] = Vector2f(0.f, 0.f);
		dynamic_tile_indices[slot] = std::numeric_limits<uint16_t>::max();
	}
}

void make_manual_tile_slot_mapping(
		TileSlotMapping &mapping,
		const FixedArray<Vector2i, TILE_SLOT_COUNT> &tiles,
		const Vector2i grid_size
) {
	ZN_ASSERT_RETURN(grid_size.x > 0 && grid_size.y > 0);
	for (unsigned int slot = 0; slot < TILE_SLOT_COUNT; ++slot) {
		const Vector2i tile = tiles[slot];
		mapping.uv_offsets[slot] = Vector2f(float(tile.x) / float(grid_size.x), float(tile.y) / float(grid_size.y));
	}
}

void make_atlas_tile_slot_mapping(
		TileSlotMapping &mapping,
		const Ref<VoxelBlockyTextureAtlas> &atlas,
		const FixedArray<int, TILE_SLOT_COUNT> &tile_ids,
		AtlasIndexer &atlas_indexer
) {
	ZN_ASSERT_RETURN(atlas.is_valid());

	const Vector2i atlas_res = atlas->get_resolution();
	ZN_ASSERT_RETURN(atlas_res.x > 0 && atlas_res.y > 0);
	const Vector2f atlas_pixel_step_norm = Vector2f(1) / to_vec2f(atlas_res);

	const unsigned int atlas_index = atlas_indexer.get_or_create_atlas_index(atlas);

	for (unsigned int slot = 0; slot < TILE_SLOT_COUNT; ++slot) {
		const int tile_id = tile_ids[slot];
		if (tile_id < 0 || !atlas->is_valid_tile_id(tile_id)) {
			continue;
		}
		const VoxelBlockyTextureAtlas::Tile &tile = atlas->get_tile(tile_id);

		// UVs are authored in the tile at (0, 0), which starts at the origin of the atlas. Moving them to the tile is
		// a translation. Like for cubes, UVs of tiles that change at meshing time match the first tile of the group.
		mapping.uv_offsets[slot] = Vector2f(tile.position_x, tile.position_y) * atlas_pixel_step_norm;

		// We don't do special baking for Single tiles, it is equivalent to fixed UVs.
		if (tile.type != VoxelBlockyTextureAtlas::TILE_TYPE_SINGLE) {
			mapping.dynamic_tile_indices[slot] = atlas_indexer.get_or_create_tile_index(atlas_index, tile_id);
		}
	}
}

} // namespace zylann::voxel::blocky
