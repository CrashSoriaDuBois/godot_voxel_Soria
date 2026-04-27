#ifndef VOXEL_GENERATOR_CUSTOM_H
#define VOXEL_GENERATOR_CUSTOM_H

#include "../../util/containers/span.h"
#include "../../util/godot/macros.h"
#include "../../util/math/vector3f.h"
#include "../../util/thread/rw_lock.h"
#include "voxel_generator_template_generators.h"

#include "../../thirdparty/fast_noise_2/include/FastNoise/FastNoise.h"

ZN_GODOT_FORWARD_DECLARE(class Curve)
//ZN_GODOT_FORWARD_DECLARE(class Noise)

namespace zylann::voxel {

class VoxelGeneratorCustom : public VoxelGeneratorTemplateGenerators {
	GDCLASS(VoxelGeneratorCustom, VoxelGeneratorTemplateGenerators)

public:
	VoxelGeneratorCustom();
	~VoxelGeneratorCustom();

	//void set_noise(Ref<Noise> noise);
	//Ref<Noise> get_noise() const;

	void set_curve(Ref<Curve> curve);
	Ref<Curve> get_curve() const;

	Result generate_block(VoxelGenerator::VoxelQueryData input) override;

	bool supports_series_generation() const override {
		return true;
	}

	void generate_series(
			Span<const float> positions_x,
			Span<const float> positions_y,
			Span<const float> positions_z,
			unsigned int channel,
			Span<float> out_values,
			Vector3f min_pos,
			Vector3f max_pos
	) override;
	//
	void set_seed(int seed);
	int get_seed() const;

	void set_frequency(float frequency);
	float get_frequency() const;

	void set_frequency2(float frequency2);
	float get_frequency2() const;

	void set_encoded_graph(String graph);
	String get_encoded_graph() const;

	void set_encoded_graph2(String graph2);
	String get_encoded_graph2() const;

private:
	//void _on_noise_changed();
	void _on_curve_changed();

	static void _bind_methods();

private:
	//Ref<Noise> _noise;

	FastNoise::SmartNode<> _noise_node;
	FastNoise::SmartNode<> _noise_node2;

	Ref<Curve> _curve;
	int _seed = 1337;
	float _frequency = 0.02f;
	float _frequency2 = 0.02f;
	String _encoded_graph = "EwDD9Sg/DQAEAAAAAAAgQAkAAGZmJj8AAAAAPw==";
	String _encoded_graph2 = "EQACAAAAAADgQBAAAACIQRMAmpmZPh8AFgABAAAACwADAAAAAgAAAAMAAAAEAAAAAAAAAD8BFAD//wAAAAAAAD8AAAAAPwAAAAA/AAAAAD8BFwAAAIC/AACAPz0KF0BSuB5AEwAAAKBABgAAj8J1PACamZk+AAAAAAA=";

	struct Parameters {
		//Ref<Noise> noise;
		Ref<Curve> curve;

		int seed = 1337;
		float frequency = 0.02f;
		float frequency2 = 0.02f;
		String encoded_graph = "EwDD9Sg/DQAEAAAAAAAgQAkAAGZmJj8AAAAAPw==";
		String encoded_graph2 = "EQACAAAAAADgQBAAAACIQRMAmpmZPh8AFgABAAAACwADAAAAAgAAAAMAAAAEAAAAAAAAAD8BFAD//wAAAAAAAD8AAAAAPwAAAAA/AAAAAD8BFwAAAIC/AACAPz0KF0BSuB5AEwAAAKBABgAAj8J1PACamZk+AAAAAAA=";

	};

	Parameters _parameters;
	RWLock _parameters_lock;
};

} // namespace zylann::voxel

#endif // VOXEL_GENERATOR_CUSTOM_H
