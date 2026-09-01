#include "sub_grid_mesh_task.h"
#include "../constants/voxel_constants.h"
#include "../meshers/blocky/blocky_light.h"
#include "../meshers/blocky/voxel_blocky_library_base.h"

namespace zylann::voxel {

namespace {

// Mirrors mesh_block_task.cpp's extract_light_slices: raw block_size^3 for persistence, TEXTURE_BORDER-padded slice for the shader's sampler3D.
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

// Runs a real flood over a CHANNEL_TYPE-only, LIGHT_PADDING-sized buffer.
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

// LOD1+ normal rebuilds: no flood, just re-slice the already-downsampled, already-persisted
// CHANNEL_DATA5 with a TEXTURE_BORDER halo - mirrors terrain's "else" branch exactly.
void extract_light_slices_from_data5(const VoxelBuffer &data5_buf, int chunk_size, VoxelMesher::Output &output) {
	Span<const uint8_t> stored;
	if (!data5_buf.get_channel_as_bytes_read_only(VoxelBuffer::CHANNEL_DATA5, stored)) {
		return;
	}
	StdVector<uint8_t> big_buf;
	big_buf.resize(stored.size());
	memcpy(big_buf.data(), stored.data(), stored.size());
	extract_light_slices(big_buf, data5_buf.get_size(), TEXTURE_BORDER, chunk_size, output);
}

} // namespace

SubGridMeshTaskResult run_mesh_task(SubGridMeshTaskInput input) {
	SubGridMeshTaskResult result;
	result.ship_uuid = input.ship_uuid;
	result.chunk_pos = input.chunk_pos;
	result.lod = input.lod;

	const int chunk_size = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const int pad = 1;

	// -------- Flood-only path: no geometry, no collision, just light. --------
	if (!input.requires_geometry) {
		result.light_only = true;
		/*print_line(
				String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) + " FLOOD_ONLY branch"
		);*/
		if (input.light_padded_buffer) {
			run_flood(*input.light_padded_buffer, input.mesher, chunk_size, result.output);
			/*print_line(
					String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
					" flood_only result: was_computed=" +
					(result.output.light_surface.was_computed ? "true" : "false") +
					" data.size=" + itos(result.output.light_surface.data.size()) +
					" texture_data.size=" + itos(result.output.light_surface.texture_data.size())
			);*/
		} /*else {
			print_line(
					String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
					" FLOOD_ONLY but light_padded_buffer is NULL!"
			);
		}*/
		return result;
	}

	if (input.should_flood && input.light_padded_buffer) {
		/*print_line(
				String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) + " REAL_FLOOD branch"
		);*/
		run_flood(*input.light_padded_buffer, input.mesher, chunk_size, result.output);
		/*print_line(
				String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
				" real_flood result: was_computed=" + (result.output.light_surface.was_computed ? "true" : "false") +
				" data.size=" + itos(result.output.light_surface.data.size()) +
				" texture_data.size=" + itos(result.output.light_surface.texture_data.size())
		);*/
	} else if (input.lod > 0 && input.data5_extract_buffer) {
		/*print_line(
				String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
				" EXTRACT_FROM_DATA5 branch"
		);*/
		// Print whether the source buffer actually has any non-zero data5 before extracting
		{
			Span<const uint8_t> raw_check;
			if (input.data5_extract_buffer->get_channel_as_bytes_read_only(VoxelBuffer::CHANNEL_DATA5, raw_check)) {
				int non_zero = 0;
				for (uint8_t v : raw_check)
					if (v != 0)
						++non_zero;
				/*print_line(
						String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
						" data5_extract_buffer non_zero=" + itos(non_zero) + " / " + itos(raw_check.size())
				);*/
			}/* else {
				print_line(
						String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
						" data5_extract_buffer channel is COMPRESSED/UNIFORM (never decompressed - likely all-zero "
						"default)"
				);
			}*/
		}
		extract_light_slices_from_data5(*input.data5_extract_buffer, chunk_size, result.output);
		/*print_line(
				String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
				" extract result: was_computed=" + (result.output.light_surface.was_computed ? "true" : "false") +
				" data.size=" + itos(result.output.light_surface.data.size())
		);*/
	} /*else {
		print_line(
				String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
				" NO LIGHT WORK DONE (should_flood=" + (input.should_flood ? "true" : "false") + " light_padded=" +
				(input.light_padded_buffer ? "set" : "null") + " lod>0=" + (input.lod > 0 ? "true" : "false") +
				" data5_extract=" + (input.data5_extract_buffer ? "set" : "null") + ")"
		);
	}*/

	VoxelMesher::Input mesher_input{ *input.padded_buffer,
									 nullptr,
									 (input.chunk_pos << SubGridChunkMap::CHUNK_SIZE_PO2) - Vector3i(pad, pad, pad),
									 (uint8_t)input.lod,
									 false,
									 false,
									 false,
									 false };
	input.mesher->build(result.output, mesher_input);

	/*print_line(
			String("RUN_TASK lod=") + itos(input.lod) + " pos=" + String(input.chunk_pos) +
			" AFTER mesher->build(): light_surface.was_computed=" +
			(result.output.light_surface.was_computed ? "true" : "false")
	);*/

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
