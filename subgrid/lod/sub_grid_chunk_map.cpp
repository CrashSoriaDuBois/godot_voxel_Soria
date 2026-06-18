#include "sub_grid_chunk_map.h"

namespace zylann::voxel {

void SubGridChunkMap::set_block_buffer(Vector3i lod0_chunk_pos, std::shared_ptr<VoxelBuffer> buf) {
	_lod_maps[0].set_block_buffer(lod0_chunk_pos, buf, /*overwrite=*/true);
	_all_lod0_chunk_positions.insert(lod0_chunk_pos);
	_all_lod_chunk_positions[0].insert(lod0_chunk_pos);

	// Propagate into LOD1+ immediately so they're ready before first mesh task
	FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> affected;
	update_lods_for_chunk(lod0_chunk_pos, affected);
}

void SubGridChunkMap::set_voxel(uint32_t value, Vector3i local_pos, int channel) {
	const Vector3i chunk_pos = voxel_to_lod0_chunk(local_pos);
	const int cs = 1 << CHUNK_SIZE_PO2;
	Vector3i voxel_in_chunk = local_pos - (chunk_pos << CHUNK_SIZE_PO2);

	VoxelDataBlock *block = _lod_maps[0].get_block(chunk_pos);
	if (block == nullptr) {
		auto buf = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_POOL);
		buf->create(cs, cs, cs);
		block = _lod_maps[0].set_block_buffer(chunk_pos, buf, /*overwrite=*/false);
		_all_lod0_chunk_positions.insert(chunk_pos);
		_all_lod_chunk_positions[0].insert(chunk_pos);
	}

	block->get_voxels().set_voxel(value, voxel_in_chunk.x, voxel_in_chunk.y, voxel_in_chunk.z, channel);
	_dirty_chunks.insert(chunk_pos);
	// Note: LOD propagation for dirty chunks is triggered by SubGridManager
	// via update_lods_for_chunk() after it calls mark_chunk_dirty().
	// We don't propagate here to avoid double work during batch edits.
}

uint32_t SubGridChunkMap::get_voxel(Vector3i local_pos, int channel) const {
	const Vector3i chunk_pos = voxel_to_lod0_chunk(local_pos);
	Vector3i voxel_in_chunk = local_pos - (chunk_pos << CHUNK_SIZE_PO2);
	const VoxelDataBlock *block = _lod_maps[0].get_block(chunk_pos);
	if (block == nullptr) {
		return 0;
	}
	std::shared_ptr<VoxelBuffer> buf = block->get_voxels_shared();
	if (!buf) {
		return 0;
	}
	return buf->get_voxel(voxel_in_chunk.x, voxel_in_chunk.y, voxel_in_chunk.z, channel);
}

void SubGridChunkMap::populate_from_world(
		const HashMap<Vector3i, uint32_t> &world_blocks,
		Vector3i local_origin,
		int channel
) {
	for (const KeyValue<Vector3i, uint32_t> &kv : world_blocks) {
		set_voxel(kv.value, kv.key - local_origin, channel);
	}
}

std::shared_ptr<VoxelBuffer> SubGridChunkMap::get_chunk_buffer(Vector3i lod0_chunk_pos) const {
	const VoxelDataBlock *block = _lod_maps[0].get_block(lod0_chunk_pos);
	if (block == nullptr) {
		return nullptr;
	}
	return block->get_voxels_shared();
}

std::shared_ptr<VoxelBuffer> SubGridChunkMap::get_lod_chunk_buffer(Vector3i lod_pos, int lod) const {
	ERR_FAIL_INDEX_V(lod, SUBGRID_MAX_LODS, nullptr);
	const VoxelDataBlock *block = _lod_maps[lod].get_block(lod_pos);
	if (block == nullptr) {
		return nullptr;
	}
	return block->get_voxels_shared();
}

Vector<Vector3i> SubGridChunkMap::get_dirty_chunks() const {
	Vector<Vector3i> out;
	for (const Vector3i &pos : _dirty_chunks) {
		out.push_back(pos);
	}
	return out;
}

void SubGridChunkMap::mark_chunk_clean(Vector3i lod0_chunk_pos) {
	_dirty_chunks.erase(lod0_chunk_pos);
}

void SubGridChunkMap::_downsample_lod_chunk(Vector3i lod_pos, int lod) {
	ERR_FAIL_COND(lod <= 0 || lod >= SUBGRID_MAX_LODS);

	const int cs = 1 << CHUNK_SIZE_PO2; // 16
	// Each LOD chunk covers 'stride' chunks from lod-1 per axis
	const int stride = 2; // always halving by 2 each level
	// lod_pos in lod-space corresponds to lod_pos*2 .. lod_pos*2+1 in (lod-1)-space

	// Allocate or reuse the output buffer
	VoxelDataBlock *dst_block = _lod_maps[lod].get_block(lod_pos);
	if (dst_block == nullptr) {
		auto buf = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_POOL);
		buf->create(cs, cs, cs);
		buf->fill(0, VoxelBuffer::CHANNEL_TYPE);
		_lod_maps[lod].set_block_buffer(lod_pos, buf, /*overwrite=*/false);
		dst_block = _lod_maps[lod].get_block(lod_pos);
	}

	VoxelBuffer &dst = dst_block->get_voxels();
	dst.fill(0, VoxelBuffer::CHANNEL_TYPE);

	// The 2x2x2 source chunks in (lod-1)-space
	const Vector3i src_base = lod_pos * stride;

	for (int dz = 0; dz < stride; dz++) {
		for (int dy = 0; dy < stride; dy++) {
			for (int dx = 0; dx < stride; dx++) {
				Vector3i src_chunk_pos = src_base + Vector3i(dx, dy, dz);
				const VoxelDataBlock *src_block = _lod_maps[lod - 1].get_block(src_chunk_pos);
				if (src_block == nullptr) {
					continue;
				}
				const VoxelBuffer &src = src_block->get_voxels_const();

				// Nearest-neighbor downsample: each output voxel samples
				// every other input voxel (step of 2).
				// Output region within dst: half-chunk offset per dx/dy/dz
				const int half_cs = cs / 2;
				const int ox = dx * half_cs;
				const int oy = dy * half_cs;
				const int oz = dz * half_cs;

				for (int z = 0; z < half_cs; z++) {
					for (int y = 0; y < half_cs; y++) {
						for (int x = 0; x < half_cs; x++) {
							uint32_t v = src.get_voxel(x * 2, y * 2, z * 2, VoxelBuffer::CHANNEL_TYPE);
							dst.set_voxel(v, ox + x, oy + y, oz + z, VoxelBuffer::CHANNEL_TYPE);
						}
					}
				}
			}
		}
	}
}

void SubGridChunkMap::update_lods_for_chunk(
		Vector3i lod0_chunk_pos,
		FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> &out_affected_lod_chunks
) {
	// Starting from lod0, propagate upward.
	// At each level, one or more LOD-space chunks may need rebuilding.
	// A change at lod0 pos (x,y,z) affects:
	//   LOD1: (x/2, y/2, z/2)
	//   LOD2: (x/4, y/4, z/4)
	//   LOD3: (x/8, y/8, z/8)
	// But since we rebuild level-by-level, we rebuild:
	//   LOD1 chunk containing lod0_chunk_pos
	//   LOD2 chunk containing that LOD1 chunk
	//   LOD3 chunk containing that LOD2 chunk

	Vector3i current_lod0 = lod0_chunk_pos;

	for (int lod = 1; lod < SUBGRID_MAX_LODS; lod++) {
		Vector3i lod_pos = lod0_to_lod_chunk(lod0_chunk_pos, lod);

		// Rebuild this LOD chunk from its (lod-1)-space sources
		_downsample_lod_chunk(lod_pos, lod);
		_all_lod_chunk_positions[lod].insert(lod_pos);
		out_affected_lod_chunks[lod].insert(lod_pos);
	}
}

void SubGridChunkMap::rebuild_all_lods() {
	// Clear LOD1+ maps
	for (int lod = 1; lod < SUBGRID_MAX_LODS; lod++) {
		_lod_maps[lod].clear();
		_all_lod_chunk_positions[lod].clear();
	}

	// Rebuild from LOD0
	FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> dummy;
	for (const Vector3i &pos : _all_lod0_chunk_positions) {
		update_lods_for_chunk(pos, dummy);
	}
}

void SubGridChunkMap::for_each_chunk(std::function<void(Vector3i chunk_pos, VoxelDataBlock &block)> callback) {
	_lod_maps[0].for_each_block([&callback](Vector3i pos, VoxelDataBlock &block) { callback(pos, block); });
}

void SubGridChunkMap::clear_buffers() {
	for (int lod = 0; lod < SUBGRID_MAX_LODS; lod++) {
		_lod_maps[lod].clear();
		_all_lod_chunk_positions[lod].clear();
	}
	_all_lod0_chunk_positions.clear();
	_dirty_chunks.clear();
}

} // namespace zylann::voxel