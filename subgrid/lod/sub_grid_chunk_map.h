#pragma once

#include "../../storage/voxel_buffer.h"
#include "../../storage/voxel_data_map.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_set.h"
#include <functional>
#include <memory>

namespace zylann::voxel {

class SubGridChunkMap {
public:
	static const int CHUNK_SIZE_PO2 = 4; // 2^4 = 16 voxels per side

	void set_voxel(uint32_t value, Vector3i local_pos, int channel);
	uint32_t get_voxel(Vector3i local_pos, int channel) const;

	void populate_from_world(const HashMap<Vector3i, uint32_t> &world_blocks, Vector3i local_origin, int channel);

	Vector<Vector3i> get_dirty_chunks() const;
	void mark_chunk_clean(Vector3i chunk_pos);

	std::shared_ptr<VoxelBuffer> get_chunk_buffer(Vector3i chunk_pos) const;
	void set_block_buffer(Vector3i chunk_pos, std::shared_ptr<VoxelBuffer> buf);

	void for_each_chunk(std::function<void(Vector3i chunk_pos, VoxelDataBlock &block)> callback);

	int get_chunk_count() const {
		return _map.get_block_count();
	}

	const HashSet<Vector3i> &get_all_chunk_positions() const {
		return _all_chunk_positions;
	}

private:
	VoxelDataMap _map;
	HashSet<Vector3i> _dirty_chunks;

	HashSet<Vector3i> _all_chunk_positions;

	static Vector3i voxel_to_chunk(Vector3i voxel_pos) {
		return voxel_pos >> CHUNK_SIZE_PO2;
	}
};

} // namespace zylann::voxel