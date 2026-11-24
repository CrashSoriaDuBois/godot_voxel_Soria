#ifndef VOXEL_BLOCKY_TILE_SLOTS_H
#define VOXEL_BLOCKY_TILE_SLOTS_H

#include "../../util/containers/fixed_array.h"
#include "../../util/math/color.h"
#include "../../util/math/vector2f.h"
#include "../../util/math/vector2i.h"
#include "voxel_blocky_texture_atlas.h"

namespace zylann::voxel::blocky {

struct AtlasIndexer;

// Meshes don't have "sides" like cubes do, so triangles tell which tile they use with their vertex color. There are 8
// possible colors, the corners of the RGB cube, so they are not altered by 8-bit quantization or color space
// conversions (each channel is either 0 or 1, and is thresholded when reading). Colors are called "slots".
// Meshes without vertex colors are white.
//
// UVs of the mesh must be authored to fit the tile at (0, 0) of the atlas. Picking a tile for a slot then translates
// the UVs by that many tiles. Using tile (0, 0) leaves UVs untouched.
enum TileSlot : uint8_t {
	TILE_SLOT_RED,		//#FF0000
	TILE_SLOT_GREEN,	//#00FF00
	TILE_SLOT_BLUE,		//#0000FF
	TILE_SLOT_YELLOW,	//#FFFF00
	TILE_SLOT_CYAN,		//#00FFFF
	TILE_SLOT_MAGENTA,	//#FF00FF
	TILE_SLOT_WHITE,	//#FFFFFF
	TILE_SLOT_BLACK,	//#000000
	TILE_SLOT_COUNT	
};

const char *get_tile_slot_name(const unsigned int slot);

// Returns -1 if the name is not a slot
int get_tile_slot_from_name(const char *name);

// Slot of a vertex color. Each channel is considered "on" above 0.5.
unsigned int get_tile_slot_from_color(const Color &color);

// What baking has to do for each slot
struct TileSlotMapping {
	// Translation to apply to the UVs of triangles using each slot, in texture normalized coordinates
	Vector2f uv_offsets[TILE_SLOT_COUNT];
	// Baked tile index for slots using tiles that change dynamically at meshing time (blob, random...). Max if none.
	// It only applies to triangles lying on a side of the voxel, because these tiles are chosen per side.
	uint16_t dynamic_tile_indices[TILE_SLOT_COUNT];

	// If true, triangles that are not on a side of the voxel but use dynamic tiles are kept apart, so tiles can connect
	// on them too (stairs, ladders...). See BakedModel::Model::InnerTileGroup.
	bool inner_dynamic_tiles = false;

	TileSlotMapping();
};

// Tiles are positions in a regular grid
void make_manual_tile_slot_mapping(
		TileSlotMapping &mapping,
		const FixedArray<Vector2i, TILE_SLOT_COUNT> &tiles,
		const Vector2i grid_size
);

// Tiles are the ones of an atlas. Negative or invalid IDs leave the UVs of the slot untouched.
void make_atlas_tile_slot_mapping(
		TileSlotMapping &mapping,
		const Ref<VoxelBlockyTextureAtlas> &atlas,
		const FixedArray<int, TILE_SLOT_COUNT> &tile_ids,
		AtlasIndexer &atlas_indexer
);

} // namespace zylann::voxel::blocky

#endif // VOXEL_BLOCKY_TILE_SLOTS_H
