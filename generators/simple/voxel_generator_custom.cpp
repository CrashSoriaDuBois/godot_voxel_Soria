#include "../../constants/voxel_string_names.h"
#include "../../util/godot/classes/curve.h"
#include "../../util/godot/classes/fast_noise_lite.h"
#include "voxel_generator_custom.h"

#include "../../thirdparty/fast_noise_2/include/FastNoise/FastNoise.h"

namespace zylann::voxel {

VoxelGeneratorCustom::VoxelGeneratorCustom() {
	//_noise_node = FastNoise::New<FastNoise::Simplex>();

	_noise_node = FastNoise::NewFromEncodedNodeTree(_encoded_graph.utf8().get_data());
	_noise_node2 = FastNoise::NewFromEncodedNodeTree(_encoded_graph2.utf8().get_data());
}

VoxelGeneratorCustom::~VoxelGeneratorCustom() {}


void VoxelGeneratorCustom::set_curve(Ref<Curve> curve) {
	if (_curve == curve) {
		return;
	}
	if (_curve.is_valid()) {
		_curve->disconnect(
				VoxelStringNames::get_singleton().changed, callable_mp(this, &VoxelGeneratorCustom::_on_curve_changed)
		);
	}
	_curve = curve;
	RWLockWrite wlock(_parameters_lock);
	if (_curve.is_valid()) {
		_curve->connect(
				VoxelStringNames::get_singleton().changed, callable_mp(this, &VoxelGeneratorCustom::_on_curve_changed)
		);
		// The Curve resource is not thread-safe so we make a copy of it for use in threads
		_parameters.curve = _curve->duplicate();
		_parameters.curve->bake();
	} else {
		_parameters.curve.unref();
	}
}

Ref<Curve> VoxelGeneratorCustom::get_curve() const {
	return _curve;
}

// Seed
void VoxelGeneratorCustom::set_seed(int seed) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.seed = seed;
	_seed = seed; // update internal FastNoise2 usage
}

int VoxelGeneratorCustom::get_seed() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.seed;
}

// Frequency
void VoxelGeneratorCustom::set_frequency(float frequency) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.frequency = frequency;
	_frequency = frequency; // update internal FastNoise2 usage
}

float VoxelGeneratorCustom::get_frequency() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.frequency;
}

// Frequency2
void VoxelGeneratorCustom::set_frequency2(float frequency2) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.frequency2 = frequency2;
	_frequency2 = frequency2; // update internal FastNoise2 usage
}

float VoxelGeneratorCustom::get_frequency2() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.frequency2;
}

// Encoded graph
void VoxelGeneratorCustom::set_encoded_graph(String graph) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.encoded_graph = graph;
	_encoded_graph = graph; // update internal copy
	_noise_node = FastNoise::NewFromEncodedNodeTree(graph.utf8().get_data()); // rebuild noise
}

String VoxelGeneratorCustom::get_encoded_graph() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.encoded_graph;
}

// Encoded graph2
void VoxelGeneratorCustom::set_encoded_graph2(String graph2) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.encoded_graph2 = graph2;
	_encoded_graph2 = graph2; // update internal copy
	_noise_node2 = FastNoise::NewFromEncodedNodeTree(graph2.utf8().get_data()); // rebuild noise
}

String VoxelGeneratorCustom::get_encoded_graph2() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.encoded_graph2;
}

//
VoxelGenerator::Result VoxelGeneratorCustom::generate_block(VoxelGenerator::VoxelQueryData input) {
	Parameters params;
	{
		RWLockRead rlock(_parameters_lock);
		params = _parameters;
	}

	Result result;

	//ERR_FAIL_COND_V(params.noise.is_null(), result);
	//Noise &noise = **params.noise;

	VoxelBuffer &out_buffer = input.voxel_buffer; //reference to the voxel buffer we need to fill


	//here the terrain is generated
	if (_curve.is_null()) {
		result = VoxelGeneratorTemplateGenerators::carved3D(
				out_buffer,

				//[&noise](int x, int z) { return 0.5 + 0.5 * noise.get_noise_2d(x, z); },
				[this](int x, int y, int z) {
					float nx = static_cast<float>(x) * _frequency2;
					float ny = static_cast<float>(y) * _frequency2;
					float nz = static_cast<float>(z) * _frequency2;
					float n = _noise_node2->GenSingle3D(nx, ny, nz, _seed);
					return n;
				},
				[this](int x, int z) {
					float nx = static_cast<float>(x) * _frequency;
					float nz = static_cast<float>(z) * _frequency;
					float n = _noise_node->GenSingle2D(nx, nz, _seed);
					return 0.5f + 0.5f * n;
				},

				input.origin_in_voxels,
				input.lod
		);
	} else {
		Curve &curve = **params.curve;
		result = VoxelGeneratorTemplateGenerators::carved3D(
				out_buffer,

				//[&noise, &curve](int x, int z) { return curve.sample_baked(0.5 + 0.5 * noise.get_noise_2d(x, z)); },

			[this](int x, int y, int z) {
				float nx = static_cast<float>(x) * _frequency2;
				float ny = static_cast<float>(y) * _frequency2;
				float nz = static_cast<float>(z) * _frequency2;
				float n = _noise_node2->GenSingle3D(nx, ny, nz, _seed);
				return n;
				},
			[this, &curve](int x, int z) {
				float nx = static_cast<float>(x) * _frequency;
				float nz = static_cast<float>(z) * _frequency;
				float n = _noise_node->GenSingle2D(nx, nz, _seed);
					return curve.sample_baked(0.5f + 0.5f * n);
				},

				input.origin_in_voxels,
				input.lod
		);
	}

	out_buffer.compress_uniform_channels();
	return result;
}

void VoxelGeneratorCustom::generate_series(
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

	//ERR_FAIL_COND(params.noise.is_null());
	//Noise &noise = **params.noise;

	if (_curve.is_null()) {
		generate_series_template(
				[this](int x, int z) {
					float nx = static_cast<float>(x) * _frequency;
					float nz = static_cast<float>(z) * _frequency;
					float n = _noise_node->GenSingle2D(nx, nz, _seed);
					return 0.5f + 0.5f * n;
				},
				positions_x,
				positions_y,
				positions_z,
				channel,
				out_values,
				min_pos,
				max_pos
		);
	} else {
		Curve &curve = **params.curve;
		generate_series_template(
				[this, &curve](int x, int z) {
					float nx = static_cast<float>(x) * _frequency;
					float nz = static_cast<float>(z) * _frequency;
					float n = _noise_node->GenSingle2D(nx, nz, _seed);
					return curve.sample_baked(0.5f + 0.5f * n);
				},
				positions_x,
				positions_y,
				positions_z,
				channel,
				out_values,
				min_pos,
				max_pos
		);
	}
}

void VoxelGeneratorCustom::_on_curve_changed() {
	ERR_FAIL_COND(_curve.is_null());
	RWLockWrite wlock(_parameters_lock);
	_parameters.curve = _curve->duplicate();
	_parameters.curve->bake();
}

void VoxelGeneratorCustom::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_seed", "seed"), &VoxelGeneratorCustom::set_seed);
    ClassDB::bind_method(D_METHOD("get_seed"), &VoxelGeneratorCustom::get_seed);
    ClassDB::bind_method(D_METHOD("set_frequency", "frequency"), &VoxelGeneratorCustom::set_frequency);
    ClassDB::bind_method(D_METHOD("get_frequency"), &VoxelGeneratorCustom::get_frequency);
	ClassDB::bind_method(D_METHOD("set_frequency2", "frequency2"), &VoxelGeneratorCustom::set_frequency2);  //
	ClassDB::bind_method(D_METHOD("get_frequency2"), &VoxelGeneratorCustom::get_frequency2);					//
    ClassDB::bind_method(D_METHOD("set_encoded_graph", "graph"), &VoxelGeneratorCustom::set_encoded_graph);
    ClassDB::bind_method(D_METHOD("get_encoded_graph"), &VoxelGeneratorCustom::get_encoded_graph);
	    ClassDB::bind_method(D_METHOD("set_encoded_graph2", "graph2"), &VoxelGeneratorCustom::set_encoded_graph2); //
    ClassDB::bind_method(D_METHOD("get_encoded_graph2"), &VoxelGeneratorCustom::get_encoded_graph2);				//

    ADD_PROPERTY(PropertyInfo(Variant::INT, "seed"), "set_seed", "get_seed");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frequency"), "set_frequency", "get_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frequency2"), "set_frequency2", "get_frequency2"); //
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "encoded_graph"), "set_encoded_graph", "get_encoded_graph");
	    ADD_PROPERTY(PropertyInfo(Variant::STRING, "encoded_graph2"), "set_encoded_graph2", "get_encoded_graph2"); //


	ClassDB::bind_method(D_METHOD("set_curve", "curve"), &VoxelGeneratorCustom::set_curve);
	ClassDB::bind_method(D_METHOD("get_curve"), &VoxelGeneratorCustom::get_curve);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_curve", "get_curve");
}

} // namespace zylann::voxel
