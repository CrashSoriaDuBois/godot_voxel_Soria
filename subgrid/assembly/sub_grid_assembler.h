#pragma once

#include "../lod/sub_grid_chunk_map.h"
#include "../sub_grid_metadata.h"
#include "core/templates/hash_set.h"
#include "edition/voxel_tool.h"

// Forward declarations
namespace zylann::voxel {
class VoxelToolTerrain;
class VoxelLodTerrain; // add this
}

namespace zylann::voxel {

class SubGridAssembler {
public:
	struct AssembledBody {
		uint8_t uuid[16];
		SubGridMetadata metadata;
		SubGridChunkMap chunks;
		HashMap<Vector3i, uint32_t> world_blocks;
		Vector3i local_origin_in_world;
		Vector<AssembledBody *> children;
	};

	struct AssemblyConfig {
		int max_blocks = 4096;
		bool diagonal_stick = true;
		uint32_t bearing_voxel_id_min = 2;
		uint32_t bearing_voxel_id_max = 7;
	};

	static AssembledBody *assemble(
			VoxelLodTerrain *terrain,
			Vector3i start_world_pos,
			const AssemblyConfig &config,
			String &out_error);

	static void _free_tree(AssembledBody *body);
	static void generate_uuid_v4(uint8_t *out_16bytes);
	static bool is_bearing_voxel(const AssemblyConfig &config, uint32_t voxel_id);
	static Vector3i facing_from_bearing_id(uint32_t voxel_id);

	static const Vector3i OFFSETS_CARDINAL_AND_EDGE[18];

private:
	static void flood_fill(
			VoxelTool *tool,
			Vector3i start,
			Vector3i anchor,
			Vector3i local_origin,
			const AssemblyConfig &config,
			HashSet<Vector3i> &visited,
			AssembledBody *out_body,
			String &out_error);

	static void _erase_from_terrain(VoxelTool *tool, AssembledBody *body);
	static bool is_movable(VoxelTool *tool, Vector3i pos);
	static Vector3i _facing_from_index(uint32_t idx);
};

} // namespace zylann::voxel