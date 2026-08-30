#include "sub_grid_assembler.h"
#include "edition/voxel_tool.h"
#include "storage/voxel_buffer.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "core/math/random_number_generator.h"

#include "../../meshers/blocky/voxel_blocky_library_base.h"
#include "../../meshers/blocky/blocky_baked_library.h"
#include "../../meshers/blocky/voxel_mesher_blocky.h"

namespace zylann::voxel {

// Matches SimAssemblyContraption::DIRECTION_OFFSETS exactly:
// 6 cardinal faces + 12 edge diagonals (no face-corners or body-diagonals)
const Vector3i SubGridAssembler::OFFSETS_CARDINAL_AND_EDGE[18] = {
	// cardinal
	{ 1, 0, 0 },
	{ -1, 0, 0 },
	{ 0, 1, 0 },
	{ 0, -1, 0 },
	{ 0, 0, 1 },
	{ 0, 0, -1 },
	// edge diagonals
	{ 1, 1, 0 },
	{ -1, -1, 0 },
	{ 1, -1, 0 },
	{ -1, 1, 0 },
	{ 1, 0, 1 },
	{ -1, 0, -1 },
	{ 1, 0, -1 },
	{ -1, 0, 1 },
	{ 0, 1, 1 },
	{ 0, -1, -1 },
	{ 0, -1, 1 },
	{ 0, 1, -1 }
};

// ________________________________________________________________
// Public entry point

SubGridAssembler::AssembledBody *SubGridAssembler::assemble(
		VoxelLodTerrain *terrain,
		Vector3i start_world_pos,
		const AssemblyConfig &config,
		String &out_error
) {
	ERR_FAIL_COND_V(terrain == nullptr, nullptr);

	Ref<VoxelTool> tool = terrain->get_voxel_tool();
	ERR_FAIL_COND_V(!tool.is_valid(), nullptr);

	Ref<VoxelMesherBlocky> blocky_mesher = terrain->get_mesher();
	Ref<VoxelBlockyLibraryBase> lib = blocky_mesher.is_valid() ? blocky_mesher->get_library() : Ref<VoxelBlockyLibraryBase>();

	// Same channel depths the terrain actually uses - this gets baked permanently into this
	// ship's first save, so it must come from the terrain's real format, not VoxelBuffer's
	// raw defaults.
	VoxelFormat format;
	Ref<godot::VoxelFormat> terrain_format = terrain->get_format();
	if (terrain_format.is_valid()) {
		format = terrain_format->get_internal();
	}

	// Check starting block is not air
	uint32_t start_voxel = tool->get_voxel(start_world_pos);
	if (start_voxel == 0) {
		out_error = "Assembly start position is air";
		return nullptr;
	}

	// Allocate the root body
	AssembledBody *root = memnew(AssembledBody);
	generate_uuid_v4(root->uuid);
	root->metadata.is_root = true;
	memcpy(root->metadata.uuid, root->uuid, 16);

	root->local_origin_in_world = start_world_pos;

	// visited is shared across ALL recursive flood-fills so no block
	// gets claimed by both parent and child bodies
	HashSet<Vector3i> visited;

	flood_fill(
			tool.ptr(),
			start_world_pos,
			Vector3i(INT32_MIN, INT32_MIN, INT32_MIN), // no anchor for root
			start_world_pos, // local origin = start pos
			config,
			visited,
			root,
			format,
			out_error
	);

	if (!out_error.is_empty()) {
		// Clean up partial result
		_free_tree(root);
		return nullptr;
	}

	// We do this AFTER the full flood-fill (including children) so a failed assembly leaves the terrain untouched.
	_erase_from_terrain(tool.ptr(), root, lib);

	return root;
}

// ________________________________________________________________
// Recursive flood-fill

void SubGridAssembler::flood_fill(
		VoxelTool *tool,
		Vector3i start,
		Vector3i anchor,
		Vector3i local_origin,
		const AssemblyConfig &config,
		HashSet<Vector3i> &visited,
		AssembledBody *out_body,
		const VoxelFormat &format, // ADD this
		String &out_error
) {
	out_body->chunks.set_format(format);

	LocalVector<Vector3i> frontier;
	frontier.push_back(start);
	uint32_t head = 0;

	int block_count = 0;

	// Set channel once for normal voxel reads
	tool->set_channel(VoxelBuffer::CHANNEL_TYPE);

	while (head < frontier.size()) {
		Vector3i pos = frontier[head];
		head++;

		if (visited.has(pos)) {
			continue;
		}
		visited.insert(pos);

		// Stop at the anchor - prevents re-entering the parent body
		if (pos == anchor) {
			continue;
		}

		uint32_t voxel = tool->get_voxel(pos); // tool channel is CHANNEL_TYPE here

		// Air - skip
		if (voxel == 0) {
			continue;
		}

		if (!is_movable(tool, pos)) {
			out_error = "Block is not movable";
			return;
		}

		block_count++;
		if (block_count > config.max_blocks) {
			out_error = "Structure too large";
			return;
		}

		// Store block
		Vector3i local_pos = pos - local_origin;
		out_body->world_blocks[pos] = voxel;

		// Carry every tracked channel from terrain into the ship, not just TYPE - otherwise a
		// painted color/data5 value never even reaches the ship's chunk map. Channel is
		// switched on the shared `tool` and restored to CHANNEL_TYPE before continuing, since
		// is_movable() and the next loop iteration's occupancy read both assume TYPE is active.
		for (int c = 0; c < SubGridChunkMap::SUBGRID_CHANNEL_COUNT; c++) {
			const VoxelBuffer::ChannelId channel = SubGridChunkMap::SUBGRID_CHANNELS[c];
			uint32_t v;
			if (channel == VoxelBuffer::CHANNEL_TYPE) {
				v = voxel;
			} else {
				tool->set_channel(channel);
				v = tool->get_voxel(pos);
			}
			out_body->chunks.set_voxel(v, local_pos, channel);
		}
		tool->set_channel(VoxelBuffer::CHANNEL_TYPE);

		// Bearing block: spawn a child body, change later
		if (is_bearing_voxel(config, voxel)) {
			// Facing comes directly from block ID - no channel read needed
			Vector3i facing = facing_from_bearing_id(voxel);
			Vector3i attach_pos = pos + facing;

			if (!visited.has(attach_pos) && tool->get_voxel(attach_pos) != 0) {
				AssembledBody *child = memnew(AssembledBody);
				generate_uuid_v4(child->uuid);
				child->metadata.pivot_in_parent_local = pos - local_origin;
				child->metadata.rotation_axis = facing;
				child->metadata.is_root = false;
				child->local_origin_in_world = attach_pos;

				child->terrain_anchored = out_body->world_blocks.size() == 0;

				memcpy(child->metadata.uuid, child->uuid, 16);
				memcpy(child->metadata.parent_uuid, out_body->uuid, 16);

				flood_fill(tool, attach_pos, pos, attach_pos, config, visited, child, format, out_error);

				if (!out_error.is_empty()) {
					memdelete(child);
					return;
				}

				out_body->children.push_back(child);
			}

			continue;
		}

		//Normal block: add neighbors
		int num_offsets = config.diagonal_stick ? 18 : 6;
		for (int i = 0; i < num_offsets; i++) {
			Vector3i neighbor = pos + OFFSETS_CARDINAL_AND_EDGE[i];
			if (!visited.has(neighbor) && neighbor != anchor) {
				frontier.push_back(neighbor);
			}
		}
	}
}

// ________________________________________________________________
// Erase assembled blocks from terrain (called only on success)


void SubGridAssembler::_erase_from_terrain(VoxelTool *tool, AssembledBody *body, Ref<VoxelBlockyLibraryBase> lib) {
	tool->set_channel(VoxelBuffer::CHANNEL_TYPE);
	for (const KeyValue<Vector3i, uint32_t> &kv : body->world_blocks) {
		bool relevant = false;
		if (lib.is_valid()) {
			RWLockRead lock(lib->get_baked_data_rw_lock());
			const blocky::BakedLibrary &baked = lib->get_baked_data();
			relevant = baked.has_model(kv.value) && baked.models[kv.value].light_emission > 0;
		}
		tool->set_voxel(kv.key, 0, relevant); // position, value, p_relevant
	}
	for (AssembledBody *child : body->children) {
		_erase_from_terrain(tool, child, lib);
	}
}

// ________________________________________________________________
// Helpers

bool SubGridAssembler::is_movable(VoxelTool *tool, Vector3i pos) {
	// Non-movable = bedrock equivalent.
	// In your library, block ID 1 might be bedrock, etc.
	// For now: everything non-air is movable.
	// Replace with a lookup into VoxelBlockyLibrary tags when you have them.
	uint32_t v = tool->get_voxel(pos);
	return v != 0;
}
/*
Vector3i SubGridAssembler::_facing_from_index(uint32_t idx) {
	// Matches Godot's Direction enum order
	switch (idx) {
		case 0:
			return { 1, 0, 0 }; // +X
		case 1:
			return { -1, 0, 0 }; // -X
		case 2:
			return { 0, 1, 0 }; // +Y
		case 3:
			return { 0, -1, 0 }; // -Y
		case 4:
			return { 0, 0, 1 }; // +Z
		case 5:
			return { 0, 0, -1 }; // -Z
		default:
			ERR_PRINT("Invalid facing index");
			return { 0, 1, 0 };
	}
} */

void SubGridAssembler::_free_tree(AssembledBody *body) {
	if (body == nullptr)
		return;
	for (AssembledBody *child : body->children) {
		_free_tree(child);
	}
	memdelete(body);
}

void SubGridAssembler::generate_uuid_v4(uint8_t *out_16bytes) {
	// Simple UUID v4 using Godot's random number generator
	// Good enough for ship IDs - not cryptographically secure
	RandomNumberGenerator rng;
	rng.randomize();
	for (int i = 0; i < 16; i++) {
		out_16bytes[i] = (uint8_t)(rng.randi() & 0xFF);
	}
	// Set version 4 bits per RFC 4122
	out_16bytes[6] = (out_16bytes[6] & 0x0F) | 0x40;
	out_16bytes[8] = (out_16bytes[8] & 0x3F) | 0x80;
}

bool SubGridAssembler::is_bearing_voxel(const AssemblyConfig &config, uint32_t voxel_id) {
	if (config.bearing_voxel_id_max == 0)
		return false;
	return voxel_id >= config.bearing_voxel_id_min && voxel_id <= config.bearing_voxel_id_max;
}

Vector3i SubGridAssembler::facing_from_bearing_id(uint32_t voxel_id) {
	switch (voxel_id) {
		case 2:
			return { 1, 0, 0 }; // +X
		case 3:
			return { -1, 0, 0 }; // -X
		case 4:
			return { 0, 1, 0 }; // +Y
		case 5:
			return { 0, -1, 0 }; // -Y
		case 6:
			return { 0, 0, 1 }; // +Z
		case 7:
			return { 0, 0, -1 }; // -Z
		default:
			return { 0, 1, 0 };
	}
}

} // namespace zylann::voxel
