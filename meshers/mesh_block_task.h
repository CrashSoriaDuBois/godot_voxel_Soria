#ifndef VOXEL_MESH_BLOCK_TASK_H
#define VOXEL_MESH_BLOCK_TASK_H

#include "../constants/voxel_constants.h"
#include "../engine/ids.h"
#include "../engine/meshing_dependency.h"
#include "../engine/priority_dependency.h"
#include "../storage/voxel_buffer.h"
#include "../util/containers/std_vector.h"
#include "../util/godot/classes/array_mesh.h"
#include "../util/tasks/cancellation_token.h"
#include "../util/tasks/threaded_task.h"

#ifdef VOXEL_ENABLE_SMOOTH_MESHING
#include "../engine/detail_rendering/detail_rendering.h"
#endif

#ifdef VOXEL_ENABLE_GPU
#include "../generators/generate_block_gpu_task.h"
#endif

namespace zylann::voxel {

class VoxelData;

// Asynchronous task generating a mesh from voxel blocks and their neighbors, in a particular volume
class MeshBlockTask
#ifdef VOXEL_ENABLE_GPU
		: public IGeneratingVoxelsThreadedTask
#else
		: public IThreadedTask
#endif
{
public:
	enum LightMode : uint8_t {
		LIGHT_MODE_FULL, // geometry + (flood if light_dirty) + texture   [LOD0]
		LIGHT_MODE_FLOOD_ONLY, // flood + texture, skip geometry/collision/navmesh [edit-triggered neighbor refresh]
		LIGHT_MODE_UPLOAD_ONLY // no flood, just re-extract padded halo from CHANNEL_DATA5 + upload [LOD1+ initial
							   // build]
	};
	LightMode light_mode = LIGHT_MODE_FULL;

	bool light_dirty = false;
	//static const int LIGHT_PADDING = 15;
	MeshBlockTask();
	~MeshBlockTask();

	const char *get_debug_name() const override {
		return "MeshBlock";
	}

	void run(ThreadedTaskContext &ctx) override;
	TaskPriority get_priority() override;
	bool is_cancelled() override;
	void apply_result() override;

#ifdef VOXEL_ENABLE_GPU
	void set_gpu_results(StdVector<GenerateBlockGPUTaskResult> &&results) override;
#endif

	static int debug_get_running_count();

	// 3x3x3 or 4x4x4 grid of voxel blocks.
	FixedArray<std::shared_ptr<VoxelBuffer>, constants::MAX_BLOCK_COUNT_PER_REQUEST> blocks;
	// TODO Need to provide format
	// FixedArray<uint8_t, VoxelBuffer::MAX_CHANNELS> channel_depths;
	Vector3i mesh_block_position; // In mesh blocks of the specified lod
	VolumeID volume_id;
	uint8_t lod_index = 0;
	uint8_t blocks_count = 0;
	// If true, a rendering mesh resource will be created if possible.
	bool require_visual = true;
	// If true, a collision mesh is required if possible
	bool collision_hint = false;
	// If true, the mesh will be used in a context with LOD, which might require a few extra things in the way it is
	// built
	bool lod_hint = false;
	// Detail textures might be enabled, but we don't always want to update them in every mesh update.
	// So this boolean is also checked to know if they should be computed.
	bool require_detail_texture = false;
	uint8_t detail_texture_generator_override_begin_lod_index = 0;
	bool detail_texture_use_gpu = false;
	bool block_generation_use_gpu = false;
	PriorityDependency priority_dependency;
	std::shared_ptr<MeshingDependency> meshing_dependency;
	std::shared_ptr<VoxelData> data;
#ifdef VOXEL_ENABLE_SMOOTH_MESHING
	DetailRenderingSettings detail_texture_settings;
#endif
	Ref<VoxelGenerator> detail_texture_generator_override;
	TaskCancellationToken cancellation_token;

private:
#ifdef VOXEL_ENABLE_GPU
	void gather_voxels_gpu(zylann::ThreadedTaskContext &ctx);
#endif
	void gather_voxels_cpu();
	void build_mesh();

	void gather_light_only();
	void build_light_only();

	bool _has_run = false;
	bool _too_far = false;
	bool _has_mesh_resource = false;
	bool _needs_light_recompute = false;
#ifdef VOXEL_ENABLE_GPU
	uint8_t _stage = 0;
#endif
	VoxelBuffer _voxels; // existing: 18^3 for mesher (padding=1)
	VoxelBuffer _light_voxels; // new: 46^3 for flood (padding=15)
	VoxelMesher::Output _surfaces_output;
	Ref<Mesh> _mesh;
	Ref<Mesh> _shadow_occluder_mesh;
	StdVector<uint16_t> _mesh_material_indices; // Indexed by mesh surface
#ifdef VOXEL_ENABLE_SMOOTH_MESHING
	std::shared_ptr<DetailTextureOutput> _detail_textures;
#endif
#ifdef VOXEL_ENABLE_GPU
	StdVector<GenerateBlockGPUTaskResult> _gpu_generation_results;
#endif
};

// Builds a mesh resource from multiple surfaces data, and returns a mapping of where materials specified in the input
// will be in the returned mesh. Empty surfaces won't be added to the mesh. If the mesh is totally empty, null will be
// returned.
Ref<ArrayMesh> build_mesh( //
		Span<const VoxelMesher::Output::Surface> surfaces, //
		Mesh::PrimitiveType primitive, //
		int flags, //
		StdVector<uint16_t> &mesh_material_indices //
);

// Builds a triangles mesh resource from a single surface. If the surface is empty, returns null.
Ref<ArrayMesh> build_mesh(Array surface);

} // namespace zylann::voxel

#endif // VOXEL_MESH_BLOCK_TASK_H
