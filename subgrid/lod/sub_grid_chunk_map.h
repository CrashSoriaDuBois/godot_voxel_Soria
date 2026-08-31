#pragma once
#include "../../storage/voxel_buffer.h"
#include "../../storage/voxel_data_map.h"
#include "../../storage/voxel_format.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_set.h"
#include <functional>
#include <memory>

namespace zylann::voxel {

// Maximum number of LOD levels supported.
static const int SUBGRID_MAX_LODS = 4;

class SubGridChunkMap {
public:
	static const int CHUNK_SIZE_PO2 = 4;
	static const int LIGHT_PADDING = 15;
	static const int TEXTURE_BORDER = 2;

	// Channels carried through LOD downsampling and disassembly merges. CHANNEL_DATA5 IS included here: LOD1+ never floods on its own (mirrors terrain's
	// "LOD1+ never independently floods, just extracts from stored CHANNEL_DATA5"), so it must inherit an already-correct value via downsampling instead.
	static constexpr VoxelBuffer::ChannelId SUBGRID_CHANNELS[] = {
		VoxelBuffer::CHANNEL_TYPE,
		VoxelBuffer::CHANNEL_COLOR,
		VoxelBuffer::CHANNEL_DATA5,
	};
	static constexpr int SUBGRID_CHANNEL_COUNT = 3;

	static void apply_subgrid_channel_depths(VoxelBuffer &buf) {
		buf.set_channel_depth(VoxelBuffer::CHANNEL_COLOR, VoxelBuffer::DEPTH_16_BIT);
		buf.set_channel_depth(VoxelBuffer::CHANNEL_DATA5, VoxelBuffer::DEPTH_8_BIT);
	}

	void set_format(const VoxelFormat &format) {
		_format = format;
	}
	const VoxelFormat &get_format() const {
		return _format;
	}

	// -------------------------------------------------------------------------
	// LOD0 voxel editing (these are the only voxels ever persisted)

	void set_voxel(uint32_t value, Vector3i local_pos, int channel);
	uint32_t get_voxel(Vector3i local_pos, int channel) const;

	void populate_from_world(const HashMap<Vector3i, uint32_t> &world_blocks, Vector3i local_origin, int channel);

	// -------------------------------------------------------------------------
	// LOD0 chunk management (persistence layer)

	// Load a buffer from disk into LOD0. Does NOT mark dirty called during load.
	// Automatically propagates downsampled copies to LOD1+.
	void set_block_buffer(Vector3i lod0_chunk_pos, std::shared_ptr<VoxelBuffer> buf);

	// Get raw LOD0 buffer (for saving)
	std::shared_ptr<VoxelBuffer> get_chunk_buffer(Vector3i lod0_chunk_pos) const;

	// -------------------------------------------------------------------------
	// LOD-aware buffer access (used by SubGridManager for mesh tasks)

	// Get buffer at any LOD level. lod_pos is in LOD-space coordinates.
	// Returns nullptr if not present.
	std::shared_ptr<VoxelBuffer> get_lod_chunk_buffer(Vector3i lod_pos, int lod) const;

	// -------------------------------------------------------------------------
	// Dirty tracking (LOD0 only edits always happen at LOD0)

	Vector<Vector3i> get_dirty_chunks() const;
	void mark_chunk_clean(Vector3i lod0_chunk_pos);

	// -------------------------------------------------------------------------
	// LOD propagation
	// Call after any LOD0 chunk changes. Resamples into LOD1, LOD2, LOD3.
	// lod0_chunk_pos is the changed LOD0 chunk.
	// Returns the set of affected LOD-space positions per LOD (index 1,2,3).
	// Index 0 is unused (that's the LOD0 chunk itself).
	void update_lods_for_chunk(
			Vector3i lod0_chunk_pos,
			FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> &out_affected_lod_chunks
	);

	// Rebuild ALL LOD levels from scratch (called on full load)
	void rebuild_all_lods();

	// -------------------------------------------------------------------------
	// Iteration

	void for_each_chunk(std::function<void(Vector3i chunk_pos, VoxelDataBlock &block)> callback);

	// LOD0 chunk positions only (for persistence)
	const HashSet<Vector3i> &get_all_chunk_positions() const {
		return _all_lod0_chunk_positions;
	}

	// LOD-space chunk positions at a given LOD level
	const HashSet<Vector3i> &get_lod_chunk_positions(int lod) const {
		return _all_lod_chunk_positions[lod];
	}

	// -------------------------------------------------------------------------
	// Lifecycle

	void clear_buffers();

	int get_chunk_count() const {
		return _lod_maps[0].get_block_count();
	}

	// Mark a chunk dirty for persistence only (no LOD propagation).
	void mark_chunk_dirty_for_save(Vector3i lod0_chunk_pos) {
		if (_all_lod0_chunk_positions.has(lod0_chunk_pos)) {
			_dirty_chunks.insert(lod0_chunk_pos);
		}
	}

	// -------------------------------------------------------------------------
	// Coordinate helpers

	static Vector3i voxel_to_lod0_chunk(Vector3i voxel_pos) {
		return voxel_pos >> CHUNK_SIZE_PO2;
	}

	// Convert LOD0 chunk pos to LOD-space chunk pos at given lod
	static Vector3i lod0_to_lod_chunk(Vector3i lod0_pos, int lod) {
		// Floor division for negative coordinates
		auto floor_div = [](int a, int b) -> int { return a / b - (a % b != 0 && (a ^ b) < 0 ? 1 : 0); };
		const int stride = 1 << lod;
		return Vector3i(floor_div(lod0_pos.x, stride), floor_div(lod0_pos.y, stride), floor_div(lod0_pos.z, stride));
	}

	// World-space voxel origin of a LOD-space chunk
	static Vector3i lod_chunk_to_voxel_origin(Vector3i lod_pos, int lod) {
		const int cs = 1 << CHUNK_SIZE_PO2;
		const int stride = 1 << lod;
		return lod_pos * (cs * stride);
	}

private:
	VoxelFormat _format;
	// One VoxelDataMap per LOD level.
	// LOD0 = actual edited voxels (16^3 chunks)
	// LOD1 = downsampled 2x (each chunk covers 32^3 voxels)
	// LOD2 = downsampled 4x (each chunk covers 64^3 voxels)
	// LOD3 = downsampled 8x (each chunk covers 128^3 voxels)
	// All LOD maps use chunk size = 16 (CHUNK_SIZE_PO2 = 4).
	// The LOD-space coordinate differs: LOD1 chunk (0,0,0) covers
	// the same world space as LOD0 chunks (0,0,0) through (1,1,1).
	FixedArray<VoxelDataMap, SUBGRID_MAX_LODS> _lod_maps;

	// LOD0 chunk positions (for persistence and dirty tracking)
	HashSet<Vector3i> _all_lod0_chunk_positions;

	// LOD-space positions per level (for mesh scheduling)
	FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> _all_lod_chunk_positions;

	// LOD0 dirty chunks (for save)
	HashSet<Vector3i> _dirty_chunks;

	// Build one LOD level from the level below.
	// lod_pos is the target position in lod-space.
	// Reads from lod-1 space (which at lod=1 means lod0).
	void _downsample_lod_chunk(Vector3i lod_pos, int lod);
};

} // namespace zylann::voxel
