#ifndef VOXEL_GRID_METADATA_H
#define VOXEL_GRID_METADATA_H

#include "../../util/math/color.h"
#include "../../util/memory/memory.h"
#include "custom_voxel_metadata.h"
#include "voxel_metadata.h"
#include <cstring>

#include "../../util/containers/std_vector.h"
#include "../../util/thread/mutex.h"
#include "../../util/godot/core/aabb.h"

#include "../../util/math/color8.h"

namespace zylann::voxel {

class VoxelGridMetadata : public ICustomVoxelMetadata {
public:
	static constexpr uint8_t TYPE_INDEX = VoxelMetadata::TYPE_APP_SPECIFIC_BEGIN;
	static constexpr unsigned int MAX_CELLS = 64;
	static constexpr uint8_t FLAG_HAS_COLOR = 0x80;

	// Collision boxes in voxel-local space (0..1), built lazily by VoxelBoxMover.
	// Mutable because readers fill it while holding only a read lock; the mutex serializes that.
	mutable Mutex collision_mutex;
	mutable StdVector<AABB> collision_boxes;
	mutable uint32_t collision_cache_mask = 0;
	mutable bool collision_cache_valid = false;

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
		return Color(Color8::from_u16(c));
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

	void assign_from(const VoxelGridMetadata &o) {
		res = o.res;
		has_color = o.has_color;
		memcpy(cells, o.cells, sizeof(cells));
		memcpy(colors, o.colors, sizeof(colors));
		collision_cache_valid = false;
	}

	// Rotates the cells by an integer cube rotation M (columns = rotated X, Y, Z axes, same convention as
	// SubGridAssembler/_paste_rotated_chunks_to_terrain). Works on doubled cell-center coordinates, so it is exact.
	void rotate_into(const int M[3][3], VoxelGridMetadata &out) const {
		out.res = res;
		out.has_color = has_color;
		out.collision_cache_valid = false;
		for (int z = 0; z < res; ++z) {
			for (int x = 0; x < res; ++x) {
				for (int y = 0; y < res; ++y) {
					const int d[3] = { 2 * x + 1 - res, 2 * y + 1 - res, 2 * z + 1 - res };
					int c[3];
					for (int i = 0; i < 3; ++i) {
						const int dd = M[i][0] * d[0] + M[i][1] * d[1] + M[i][2] * d[2];
						c[i] = (dd + res - 1) / 2;
					}
					const unsigned int src = index(x, y, z);
					const unsigned int dst = out.index(c[0], c[1], c[2]);
					out.cells[dst] = cells[src];
					out.colors[dst] = colors[src];
				}
			}
		}
	}
};

} // namespace zylann::voxel

#endif
