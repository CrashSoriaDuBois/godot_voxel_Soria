#include "sub_grid_mesh_task.h"
#include "../constants/voxel_constants.h"
#include "../meshers/blocky/blocky_light.h"
#include "../meshers/blocky/voxel_blocky_library_base.h"

namespace zylann::voxel {

namespace {

// Mirrors mesh_block_task.cpp's extract_light_slices exactly: raw block_size^3 for persistence, and a TEXTURE_BORDER-padded slice for the shader's sampler3D.
void extract_light_slices(
		const StdVector<uint8_t> &big_buffer,
		const Vector3i big_size,
		const int padding,
		const int block_size,
		VoxelMesher::Output &output
) {
	auto extract_block = [&](Vector3i origin, int size, StdVector<uint8_t> &out_data) {
		out_data.resize(size * size * size);
		for (int z = 0; z < size; ++z) {
			for (int x = 0; x < size; ++x) {
				for (int y = 0; y < size; ++y) {
					const int bx = origin.x + x;
					const int by = origin.y + y;
					const int bz = origin.z + z;
					const int big_idx = by + bx * big_size.y + bz * big_size.x * big_size.y;
					const int out_idx = y + x * size + z * size * size;
					out_data[out_idx] = big_buffer[big_idx];
				}
			}
		}
	};

	const Vector3i raw_origin(padding, padding, padding);
	extract_block(raw_origin, block_size, output.light_surface.data);

	const Vector3i padded_origin(padding - TEXTURE_BORDER, padding - TEXTURE_BORDER, padding - TEXTURE_BORDER);
	const int padded_block_size = block_size + 2 * TEXTURE_BORDER;
	extract_block(padded_origin, padded_block_size, output.light_surface.texture_data);

	output.light_surface.was_computed = true;
}

// Runs the flood over a CHANNEL_TYPE-only buffer of the given size, writing into `output`.
void run_flood(
		const VoxelBuffer &light_buf,
		Ref<VoxelMesherBlocky> mesher,
		int chunk_size,
		VoxelMesher::Output &output
) {
	Ref<VoxelBlockyLibraryBase> lib = mesher->get_library();
	if (!lib.is_valid()) {
		return;
	}
	Span<const uint8_t> light_type_channel;
	if (!light_buf.get_channel_as_bytes_read_only(VoxelBuffer::CHANNEL_TYPE, light_type_channel)) {
		return;
	}
	const Vector3i light_size = light_buf.get_size();
	StdVector<uint8_t> big_buf;

	RWLockRead lock(lib->get_baked_data_rw_lock());
	const blocky::BakedLibrary &baked = lib->get_baked_data();
	const VoxelBuffer::Depth depth = light_buf.get_channel_depth(VoxelBuffer::CHANNEL_TYPE);

	if (depth == VoxelBuffer::DEPTH_8_BIT) {
		blocky::flood_fill_light(light_type_channel, light_size, baked, big_buf);
	} else if (depth == VoxelBuffer::DEPTH_16_BIT) {
		Span<const uint16_t> ids = light_type_channel.reinterpret_cast_to<const uint16_t>();
		blocky::flood_fill_light(ids, light_size, baked, big_buf);
	}

	extract_light_slices(big_buf, light_size, SubGridChunkMap::LIGHT_PADDING, chunk_size, output);
}

} // namespace

SubGridMeshTaskResult run_mesh_task(SubGridMeshTaskInput input) {
	SubGridMeshTaskResult result;
	result.ship_uuid = input.ship_uuid;
	result.chunk_pos = input.chunk_pos;
	result.lod = input.lod;

	const int chunk_size = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;

	// -------- Flood-only path: no geometry, no collision, just light. --------
	if (!input.requires_geometry) {
		result.light_only = true;
		if (input.light_padded_buffer) {
			run_flood(*input.light_padded_buffer, input.mesher, chunk_size, result.output);
		}
		return result;
	}

	// -------- Normal path: mesh + (own flood if light_dirty) --------
	const int pad = 1;

	if (input.light_dirty && input.light_padded_buffer) {
		VoxelMesher::Output light_output;
		run_flood(*input.light_padded_buffer, input.mesher, chunk_size, light_output);

		// Persist the raw slice into this chunk's own CHANNEL_DATA5, exactly like
		// MeshBlockTask does for terrain (own block only - no neighbor cross-writes).
		if (!light_output.light_surface.data.empty()) {
			input.padded_buffer->decompress_channel(VoxelBuffer::CHANNEL_DATA5);
			Span<uint8_t> dst;
			if (input.padded_buffer->get_channel_as_bytes(VoxelBuffer::CHANNEL_DATA5, dst)) {
				// input.padded_buffer is (chunk_size+2*pad)^3 with pad=1; the raw light slice
				// is chunk_size^3 and must land at the buffer's own [pad, pad+chunk_size) interior,
				// not its [0,0,0) origin.
				const int padded_size = chunk_size + 2 * pad;
				const StdVector<uint8_t> &raw = light_output.light_surface.data;
				for (int z = 0; z < chunk_size; z++) {
					for (int x = 0; x < chunk_size; x++) {
						for (int y = 0; y < chunk_size; y++) {
							const int src_idx = y + x * chunk_size + z * chunk_size * chunk_size;
							const int dx = x + pad, dy = y + pad, dz = z + pad;
							const int dst_idx = dy + dx * padded_size + dz * padded_size * padded_size;
							dst[dst_idx] = raw[src_idx];
						}
					}
				}
			}
		}
		result.output.light_surface = std::move(light_output.light_surface);
	}

	VoxelMesher::Input mesher_input{ *input.padded_buffer,
									 nullptr,
									 (input.chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2) - Vector3i(pad, pad, pad),
									 (uint8_t)input.lod,
									 false,
									 false,
									 false,
									 false };
	input.mesher->build(result.output, mesher_input); // build() overwrites result.output.surfaces
													  // but leaves light_surface untouched since
													  // VoxelMesher::build() has no concept of it.

	if (input.build_collision && input.lod == 0) {
		Vector3i chunk_voxel_origin = input.chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2;
		result.collision = SubGridCollisionBuilder::build(
				*input.padded_buffer, chunk_voxel_origin, chunk_size, input.weight_table
		);
		result.has_collision = true;
	}
	return result;
}

} // namespace zylann::voxel
