#ifndef VOXEL_GRID_METADATA_H
#define VOXEL_GRID_METADATA_H

#include "../../util/math/color.h"
#include "../../util/memory/memory.h"
#include "custom_voxel_metadata.h"
#include "voxel_metadata.h"
#include <cstring>

namespace zylann::voxel {

class VoxelGridMetadata : public ICustomVoxelMetadata {
public:
	static constexpr uint8_t TYPE_INDEX = VoxelMetadata::TYPE_APP_SPECIFIC_BEGIN;
	static constexpr unsigned int MAX_CELLS = 64;
	static constexpr uint8_t FLAG_HAS_COLOR = 0x80;

	uint8_t res = 0;
	bool has_color = false;
	uint16_t cells[MAX_CELLS] = {};
	uint16_t colors[MAX_CELLS]; // RGB565, only meaningful if has_color

	VoxelGridMetadata() {
		for (unsigned int i = 0; i < MAX_CELLS; ++i) {
			colors[i] = 0xFFFF;
		}
	}

	static inline unsigned int cell_count(unsigned int r) {
		return r * r * r;
	}

	// y fastest, then x, then z (same convention as VoxelBuffer)
	inline unsigned int index(unsigned int x, unsigned int y, unsigned int z) const {
		return y + x * res + z * res * res;
	}

	static inline Color decode_color(uint16_t c) {
		return Color(float((c >> 11) & 31) / 31.f, float((c >> 5) & 63) / 63.f, float(c & 31) / 31.f);
	}

	size_t get_serialized_size() const override {
		const unsigned int n = cell_count(res);
		return 1 + 2 * n + (has_color ? 2 * n : 0);
	}

	size_t serialize(Span<uint8_t> dst) const override {
		const unsigned int n = cell_count(res);
		size_t p = 0;
		dst[p++] = res | (has_color ? FLAG_HAS_COLOR : 0);
		for (unsigned int i = 0; i < n; ++i) {
			dst[p++] = cells[i] & 0xff;
			dst[p++] = cells[i] >> 8;
		}
		if (has_color) {
			for (unsigned int i = 0; i < n; ++i) {
				dst[p++] = colors[i] & 0xff;
				dst[p++] = colors[i] >> 8;
			}
		}
		return p;
	}

	bool deserialize(Span<const uint8_t> src, uint64_t &out_read_size) override {
		out_read_size = 0;
		if (src.size() < 1) {
			return false;
		}
		const unsigned int r = src[0] & 0x7f;
		const bool hc = (src[0] & FLAG_HAS_COLOR) != 0;
		if (r < 2 || r > 4) {
			return false;
		}
		const unsigned int n = cell_count(r);
		const size_t needed = 1 + 2 * n + (hc ? 2 * n : 0);
		if (src.size() < needed) {
			return false;
		}
		res = r;
		has_color = hc;
		size_t p = 1;
		for (unsigned int i = 0; i < n; ++i, p += 2) {
			cells[i] = uint16_t(src[p]) | (uint16_t(src[p + 1]) << 8);
		}
		for (unsigned int i = 0; i < MAX_CELLS; ++i) {
			colors[i] = 0xFFFF;
		}
		if (hc) {
			for (unsigned int i = 0; i < n; ++i, p += 2) {
				colors[i] = uint16_t(src[p]) | (uint16_t(src[p + 1]) << 8);
			}
		}
		out_read_size = needed;
		return true;
	}

	ICustomVoxelMetadata *duplicate() override {
		VoxelGridMetadata *d = ZN_NEW(VoxelGridMetadata);
		d->res = res;
		d->has_color = has_color;
		memcpy(d->cells, cells, sizeof(cells));
		memcpy(d->colors, colors, sizeof(colors));
		return d;
	}

	uint8_t get_type_index() const override {
		return TYPE_INDEX;
	}

	bool equals(const ICustomVoxelMetadata &other) const override {
		if (other.get_type_index() != TYPE_INDEX) {
			return false;
		}
		const VoxelGridMetadata &o = static_cast<const VoxelGridMetadata &>(other);
		const unsigned int n = cell_count(res);
		return res == o.res && has_color == o.has_color && memcmp(cells, o.cells, 2 * n) == 0 &&
				(!has_color || memcmp(colors, o.colors, 2 * n) == 0);
	}
};

} // namespace zylann::voxel

#endif
