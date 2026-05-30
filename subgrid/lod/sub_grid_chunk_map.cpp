#include "sub_grid_chunk_map.h"

namespace zylann::voxel {

void SubGridChunkMap::set_block_buffer(Vector3i chunk_pos, std::shared_ptr<VoxelBuffer> buf) {
	_map.set_block_buffer(chunk_pos, buf, false);
	// Not marked dirty - it was just loaded from disk
	_all_chunk_positions.insert(chunk_pos);
}

void SubGridChunkMap::set_voxel(uint32_t value, Vector3i local_pos, int channel) {
	Vector3i chunk_pos = voxel_to_chunk(local_pos);
	Vector3i voxel_in_chunk = local_pos - (chunk_pos << CHUNK_SIZE_PO2);

	VoxelDataBlock *block = _map.get_block(chunk_pos);
	if (block == nullptr) {
		int cs = 1 << CHUNK_SIZE_PO2;
		// VoxelBuffer requires explicit allocator - use pool allocator
		std::shared_ptr<VoxelBuffer> buf = make_shared_instance<VoxelBuffer>(VoxelBuffer::ALLOCATOR_POOL);
		buf->create(cs, cs, cs);
		block = _map.set_block_buffer(chunk_pos, buf, false);
	}

	block->get_voxels().set_voxel(value, voxel_in_chunk.x, voxel_in_chunk.y, voxel_in_chunk.z, channel);
	_dirty_chunks.insert(chunk_pos);
	_all_chunk_positions.insert(chunk_pos);
}

uint32_t SubGridChunkMap::get_voxel(Vector3i local_pos, int channel) const {
	Vector3i chunk_pos = voxel_to_chunk(local_pos);
	Vector3i voxel_in_chunk = local_pos - (chunk_pos << CHUNK_SIZE_PO2);

	const VoxelDataBlock *block = _map.get_block(chunk_pos);
	if (block == nullptr) {
		return 0;
	}
	// get_voxels_shared() returns std::shared_ptr - dereference it
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
		Vector3i local_pos = kv.key - local_origin;
		set_voxel(kv.value, local_pos, channel);
	}
}

Vector<Vector3i> SubGridChunkMap::get_dirty_chunks() const {
	Vector<Vector3i> out;
	for (const Vector3i &pos : _dirty_chunks) {
		out.push_back(pos);
	}
	return out;
}

void SubGridChunkMap::mark_chunk_clean(Vector3i chunk_pos) {
	_dirty_chunks.erase(chunk_pos);
}

std::shared_ptr<VoxelBuffer> SubGridChunkMap::get_chunk_buffer(Vector3i chunk_pos) const {
	const VoxelDataBlock *block = _map.get_block(chunk_pos);
	if (block == nullptr) {
		return nullptr;
	}
	return block->get_voxels_shared();
}

void SubGridChunkMap::for_each_chunk(std::function<void(Vector3i chunk_pos, VoxelDataBlock &block)> callback) {
	// VoxelDataMap::for_each_block passes (Vector3i pos, VoxelDataBlock &block). position FIRST, block SECOND
	_map.for_each_block([&callback](Vector3i pos, VoxelDataBlock &block) { callback(pos, block); });
}

} // namespace zylann::voxel