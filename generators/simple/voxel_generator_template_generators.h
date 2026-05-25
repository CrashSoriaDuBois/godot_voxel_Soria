#ifndef VOXEL_GENERATOR_TEMPLATE_GENERATORS_H
#define VOXEL_GENERATOR_TEMPLATE_GENERATORS_H

#include "../../constants/voxel_constants.h"
#include "../../storage/voxel_buffer.h"
#include "../../storage/voxel_buffer_gd.h"
#include "../../util/containers/span.h"
#include "../../util/math/funcs.h"
#include "../../util/math/vector3f.h"
#include "../../util/math/vector3i.h"
#include "../../util/thread/rw_lock.h"
#include "../voxel_generator.h"
//VoxelGeneratorTemplateGenerators VOXEL_GENERATOR_TEMPLATE_GENERATORS_H
namespace zylann::voxel {

// Common base class for basic heightmap generators
class VoxelGeneratorTemplateGenerators : public VoxelGenerator {
	GDCLASS(VoxelGeneratorTemplateGenerators, VoxelGenerator)
public:
	VoxelGeneratorTemplateGenerators();
	~VoxelGeneratorTemplateGenerators();

	void set_channel(VoxelBuffer::ChannelId p_channel);
	VoxelBuffer::ChannelId get_channel() const;

	int get_used_channels_mask() const override;

	void set_height_start(float start);
	float get_height_start() const;

	void set_height_range(float range);
	float get_height_range() const;

	void set_iso_scale(float iso_scale);
	float get_iso_scale() const;

	void set_offset(const Vector2i offset);
	Vector2i get_offset() const;

protected:
	void _b_set_channel(godot::VoxelBuffer::ChannelId p_channel);
	godot::VoxelBuffer::ChannelId _b_get_channel() const;

	// float height_func(x, y)
	template <typename Height_F>
	Result generate2D(VoxelBuffer &out_buffer, Height_F height_func, Vector3i p_origin, int lod) {
		Parameters params;
		{
			RWLockRead rlock(_parameters_lock);
			params = _parameters;
		}

		const Vector3i origin(p_origin.x - params.offset.x, p_origin.y, p_origin.z - params.offset.y);

		const int channel = params.channel;
		const Vector3i bs = out_buffer.get_size();
		const bool use_sdf = channel == VoxelBuffer::CHANNEL_SDF;

		if (origin.y > get_height_start() + get_height_range()) {
			// The bottom of the block is above the highest ground can go (default is air)
			Result result;
			result.max_lod_hint = true;
			return result;
		}
		if (origin.y + (bs.y << lod) < get_height_start()) {
			// The top of the block is below the lowest ground can go
			out_buffer.clear_channel(params.channel, use_sdf ? 0 : params.matter_type);
			Result result;
			result.max_lod_hint = true;
			return result;
		}

		const int stride = 1 << lod;

		if (use_sdf) {
			int gz = origin.z;

			for (int z = 0; z < bs.z; ++z, gz += stride) {
				int gx = origin.x;

				for (int x = 0; x < bs.x; ++x, gx += stride) {
					float h = params.range.xform(height_func(gx, gz));
					int gy = origin.y;
					for (int y = 0; y < bs.y; ++y, gy += stride) {
						const float sdf = params.iso_scale * (gy - h);
						out_buffer.set_voxel_f(sdf, x, y, z, channel);
					}

				} // for x
			} // for z

		} else {
			// Blocky

			int gz = origin.z;
			for (int z = 0; z < bs.z; ++z, gz += stride) {

				int gx = origin.x;
				for (int x = 0; x < bs.x; ++x, gx += stride) {

					// Output is blocky, so we can go for just one sample
					float h = params.range.xform(height_func(gx, gz));
					h -= origin.y;
					int ih = math::arithmetic_rshift(int(h), lod);
					if (ih > 0) {
						if (ih > bs.y) {
							ih = bs.y;
						}
						out_buffer.fill_area(params.matter_type, Vector3i(x, 0, z), Vector3i(x + 1, ih, z + 1), channel);
					}

				} // for x
			} // for z
		} // use_sdf

		return Result();
	}

	// float height_func(x, y)
	template <typename Height_F>
	void generate_series_template(
			Height_F height_func,
			Span<const float> positions_x,
			Span<const float> positions_y,
			Span<const float> positions_z,
			unsigned int channel,
			Span<float> out_values,
			Vector3f min_pos,
			Vector3f max_pos
	) {
		Parameters params;
		{
			RWLockRead rlock(_parameters_lock);
			params = _parameters;
		}

		// const int channel = params.channel;
		const bool use_sdf = channel == VoxelBuffer::CHANNEL_SDF;

		if (use_sdf) {
			for (unsigned int i = 0; i < out_values.size(); ++i) {
				const float h = params.range.xform(height_func(positions_x[i], positions_z[i]));
				const float sd = positions_y[i] - h;
				// Not scaling here, since the return values are uncompressed floats
				out_values[i] = sd;
			}
		} else {
			for (unsigned int i = 0; i < out_values.size(); ++i) {
				const float h = params.range.xform(height_func(positions_x[i], positions_z[i]));
				const float sd = positions_y[i] - h;
				out_values[i] = sd < 0.f ? params.matter_type : 0;
			}
		}
	}

template <typename Noise_F>
	VoxelGenerator::Result generate3D(VoxelBuffer &out_buffer, const Noise_F &noise_func, Vector3i p_origin, int lod) {
		Parameters params;
		{
			RWLockRead rlock(_parameters_lock);
			params = _parameters;
		}

		const Vector3i origin(p_origin.x - params.offset.x, p_origin.y, p_origin.z - params.offset.y);

		const Vector3i size = out_buffer.get_size();
		const int channel = params.channel;
		const bool use_sdf = channel == VoxelBuffer::CHANNEL_SDF;
		const int stride = 1 << lod;

		Result result;

		if (use_sdf) {
			// --- SDF-based 3D terrain (smooth mode) ---
			for (int z = 0; z < size.z; ++z) {
				const int lz = origin.z + (z << lod);
				for (int x = 0; x < size.x; ++x) {
					const int lx = origin.x + (x << lod);
					for (int y = 0; y < size.y; ++y) {
						const int ly = origin.y + (y << lod);

						// Sample 3D noise at voxel coordinates
						const float n = noise_func(lx, ly, lz); // Expecting -1..1 range
						const float t = (ly - params.range.start) / params.range.height;
						const float bias = 2.0f * t - 1.0f;

						// Combine height bias and noise to form an SDF
						const float sdf = params.iso_scale * (n + bias);

						out_buffer.set_voxel_f(sdf, x, y, z, channel);
					}
				}
			}

		} else {
			// --- Blocky (voxel material) mode ---
			for (int z = 0; z < size.z; ++z) {
				const int lz = origin.z + (z << lod);
				for (int x = 0; x < size.x; ++x) {
					const int lx = origin.x + (x << lod);
					for (int y = 0; y < size.y; ++y) {
						const int ly = origin.y + (y << lod);

						// Sample noise and map to 0..1
						const float n = noise_func(lx, ly, lz);
						const float t = (ly - params.range.start) / params.range.height;
						float bias = 2.0f * t - 1.0f; // use only for surface arreglar const

						// Threshold to decide solid/air
						if (t > 0.8f) { // top 20% of range
							bias = 2.0f * (t - 0.8f) / 0.2f - 1.0f; // only affect surface
						}
						const float d = n + bias;

						const float density = n - 0.5f; //3d noise without restriction
						if (density < 0.0f) {
							out_buffer.set_voxel(params.matter_type, x, y, z, channel);
						} else {
							out_buffer.set_voxel(0, x, y, z, channel);
						}
					}
				}
			}
		}

		return result;
	}

template <typename Noise_F, typename Height_F>
	VoxelGenerator::Result carved3D(VoxelBuffer &out_buffer, const Noise_F &noise_func, const Height_F height_func, Vector3i p_origin, int lod) {
		Parameters params;
		{
			RWLockRead rlock(_parameters_lock);
			params = _parameters;
		}

		const Vector3i origin(p_origin.x - params.offset.x, p_origin.y, p_origin.z - params.offset.y);

		const Vector3i size = out_buffer.get_size();
		const int channel = params.channel;
		const bool use_sdf = channel == VoxelBuffer::CHANNEL_SDF;
		const int stride = 1 << lod;

		Result result;
		out_buffer.clear_channel(channel, 0);

		if (use_sdf) {
			// --- SDF-based 3D terrain (smooth mode) ---
			for (int z = 0; z < size.z; ++z) {
				const int lz = origin.z + (z << lod);
				for (int x = 0; x < size.x; ++x) {
					const int lx = origin.x + (x << lod);
					for (int y = 0; y < size.y; ++y) {
						const int ly = origin.y + (y << lod);

						// Sample 3D noise at voxel coordinates
						const float n = noise_func(lx, ly, lz); // Expecting -1..1 range
						const float t = (ly - params.range.start) / params.range.height;
						const float bias = 2.0f * t - 1.0f;

						// Combine height bias and noise to form an SDF
						const float sdf = params.iso_scale * (n + bias);

						out_buffer.set_voxel_f(sdf, x, y, z, channel);
					}
				}
			}

		} else {
			// --- Blocky (voxel material) mode ---
			for (int z = 0; z < size.z; ++z) {
				const int lz = origin.z + (z << lod);
				for (int x = 0; x < size.x; ++x) {
					const int lx = origin.x + (x << lod);

					float h_world = params.range.xform(height_func(lz, lx));
					float h_local = h_world - origin.y;
					int ih = math::arithmetic_rshift(int(h_local), lod);

					if (ih <= 0) {
						// surface is at or below this block bottom, nothing to fill in this column
						continue;
					}
					if (ih > size.y) {
						ih = size.y;
					}
					for (int y = 0; y < ih; ++y) {
						const int ly = origin.y + (y << lod); // world Y for this voxel

						float dist_to_surface = h_world - ly; // for dirt placement, negative value is above surface

						int material = STONE;
						material = (dist_to_surface < dirt_depth) ? DIRT : material;
						material = (dist_to_surface < grass_depth) ? GRASS : material;
						const float n = noise_func(lx, ly, lz); // Expect -1..1
						const float density = n - 0.5f; // threshold around 0.5
						if (density < 0.0f) {

							out_buffer.set_voxel(material, x, y, z, channel);
						}
						// else leave it as cleared (air) - carving happens here.
					}
				}
			}
		}

		return result;
	}

private:
	static void _bind_methods();

	struct Range {
		float start = -50.f;
		float height = 200.f;

		inline float xform(float x) const {
			return x * height + start;
		}
	};

	struct Parameters {
		VoxelBuffer::ChannelId channel = VoxelBuffer::CHANNEL_TYPE;
		int matter_type = 1;
		Range range;
		float iso_scale = 1.f;
		Vector2i offset;
	};

	RWLock _parameters_lock;
	Parameters _parameters;

	const int AIR = 0;
	const int GRASS = 1;
	const int DIRT = 2;
	const int STONE = 4;

	const float dirt_depth = 5.0f;
	const float grass_depth = 2.0f;
};

} // namespace zylann::voxel

#endif // VOXEL_GENERATOR_TEMPLATE_GENERATORS_H
