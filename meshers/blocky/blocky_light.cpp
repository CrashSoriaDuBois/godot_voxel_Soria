#include "blocky_light.h"
#include "blocky_baked_library.h"
#include <queue>
#include "../../util/io/log.h"
#include <string>

namespace zylann::voxel::blocky {


static uint8_t encode_light(uint8_t color, uint8_t intensity) {
	return ((color & 0xF) << 4) | (intensity & 0xF);
}
static uint8_t get_light_color(uint8_t v) {
	return (v >> 4) & 0xF;
}
static uint8_t get_light_intensity(uint8_t v) {
	return v & 0xF;
}

template <typename Type_T>
static void flood_fill_light_impl(
		const Span<const Type_T> type_buffer,
		const Vector3i block_size,
		const BakedLibrary &library,
		StdVector<uint8_t> &out_light
) {
	const int volume = block_size.x * block_size.y * block_size.z;
	out_light.assign(volume, 0);

	struct LightNode {
		int index;
		uint8_t value;
	};

	std::queue<LightNode> queue;

	const int row_size = block_size.y;
	const int deck_size = block_size.x * row_size;
	const int neighbor_offsets[6] = { row_size, -row_size, deck_size, -deck_size, 1, -1 };

	int total_non_air = 0; // no ); here

	for (int i = 0; i < volume; i++) {
		if (static_cast<uint32_t>(type_buffer[i]) != AIR_ID) {
			++total_non_air;
		}

		const uint32_t voxel_id = static_cast<uint32_t>(type_buffer[i]);
		if (voxel_id == AIR_ID || !library.has_model(voxel_id)) {
			continue;
		}
		const BakedModel &model = library.models[voxel_id];
		if (model.light_emission > 0) {
			const uint8_t encoded = encode_light(model.light_color_index, model.light_emission);
			out_light[i] = encoded;
			queue.push({ i, encoded });
		}
	}

	while (!queue.empty()) {
		const LightNode node = queue.front();
		queue.pop();

		const uint8_t intensity = get_light_intensity(node.value);
		const uint8_t color = get_light_color(node.value);

		if (intensity == 0) {
			continue;
		}
		const uint8_t next_intensity = intensity - 1;

		for (int n = 0; n < 6; n++) {
			const int neighbor_idx = node.index + neighbor_offsets[n];
			if (neighbor_idx < 0 || neighbor_idx >= volume) {
				continue;
			}

			const uint32_t neighbor_type = static_cast<uint32_t>(type_buffer[neighbor_idx]);
			if (neighbor_type != AIR_ID && library.has_model(neighbor_type)) {
				if (!library.models[neighbor_type].is_transparent) {
					continue;
				}
			}

			const uint8_t current_intensity = get_light_intensity(out_light[neighbor_idx]);
			if (next_intensity > current_intensity) {
				const uint8_t new_value = encode_light(color, next_intensity);
				out_light[neighbor_idx] = new_value;
				queue.push({ neighbor_idx, new_value });
			}
		}
	}
}

// Explicit non-template public functions — these are what the header declares
void flood_fill_light(
		Span<const uint8_t> type_buffer,
		Vector3i block_size,
		const BakedLibrary &library,
		StdVector<uint8_t> &out_light
) {
	flood_fill_light_impl(type_buffer, block_size, library, out_light);
}

void flood_fill_light(
		Span<const uint16_t> type_buffer,
		Vector3i block_size,
		const BakedLibrary &library,
		StdVector<uint8_t> &out_light
) {
	flood_fill_light_impl(type_buffer, block_size, library, out_light);
}

} // namespace zylann::voxel::blocky
