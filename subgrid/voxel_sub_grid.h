#pragma once

#include "lod/sub_grid_chunk_map.h"
#include "meshers/blocky/voxel_blocky_library.h"
#include "meshers/blocky/voxel_mesher_blocky.h"
//#include "scene/3d/mesh_instance_3d.h"
#include "scene/3d/node_3d.h"
#include "streaming/sub_grid_stream_helper.h"
#include "streams/sqlite/voxel_stream_sqlite.h"
#include "sub_grid_metadata.h"


namespace zylann::voxel {

class VoxelToolSubGrid;
class SubGridManager;

class VoxelSubGrid : public Node3D {
	GDCLASS(VoxelSubGrid, Node3D)

public:
	// lock mode (mirrors SwivelBearingBlockEntity::LockingSetting)
	enum LockMode : uint8_t { LOCKED_ALWAYS = 0, LOCKED_DEFAULT = 1, UNLOCKED_DEFAULT = 2, UNLOCKED_ALWAYS = 3 };

	// LIFE CYCLE
	void initialize_root(
			const SubGridMetadata &meta,
			SubGridChunkMap &&chunks,
			const String &saves_dir,
			Ref<VoxelMesherBlocky> mesher,
			Ref<VoxelBlockyLibrary> library
	);

	void initialize_root_from_disk(
			const SubGridMetadata &meta,
			const String &saves_dir,
			Ref<VoxelMesherBlocky> mesher,
			Ref<VoxelBlockyLibrary> library
	);

	void initialize_child(
			const SubGridMetadata &meta,
			SubGridChunkMap &&chunks,
			const String &saves_dir,
			bool async_stream = true
	);

	SubGridMetadata &get_metadata_mut() { 
		return _meta; 
	}

	// EDITING
	uint32_t get_voxel(Vector3i local_pos, int channel) const;
	void set_voxel(uint32_t value, Vector3i local_pos, int channel);

	Ref<VoxelToolSubGrid> get_voxel_tool();
	void set_manager(SubGridManager *manager);

	void grab(Vector3 grab_point_local);
	void grab_with_rotation(Vector3 grab_point_local);
	void release();
	void set_grab_target(Transform3D target);
	void apply_impulse(Vector3 impulse, Vector3 world_point);
	void apply_central_impulse(Vector3 impulse);

	// kinetics (children only)
	void set_angular_speed_rpm(float rpm) {
		_angular_speed_rpm = rpm;
	}
	void set_redstone_signal(bool powered);

	// LOD
	void set_viewer(Node3D *viewer) {
		_viewer = viewer;
	}

	// PERSISTANCE
	void flush_dirty_chunks();
	void save_and_close();
	void destroy();
	void _close_stream();

	void disassemble(class VoxelLodTerrain *terrain);

	// ACCESORIES
	const SubGridMetadata &get_metadata() const {
		return _meta;
	}
	bool is_root() const {
		return _meta.is_root;
	}
	VoxelSubGrid *get_root();

	bool is_rotation_grid_aligned(float tolerance_degrees) const;
	bool is_tree_grid_aligned(float tolerance_degrees) const;

	bool _is_world_anchored = false;
	bool _needs_initial_save = false;

	Transform3D _promoted_world_transform;
	Vector3 _promoted_pivot_world;

	//void rebuild_all_meshes();
	void load_chunks_from_stream();
	void clear_chunk_buffers();

	// Read-only access to chunk data for SubGridManager (padding, LOD queries).
	const SubGridChunkMap &get_chunk_map() const { return _chunks; }
	
	// Read-only access to the viewer so SubGridManager can compute LOD.
	Node3D *get_viewer() const { return _viewer; }

	double get_target_angle_rad() const {return _target_angle_rad;}

	float get_angular_speed_rpm() const { return _angular_speed_rpm; }

	double advance_rotation(double delta);
	Transform3D compute_local_transform() const;

protected:
	// Godot virtuals. must be inside the class body
	void _notification(int p_what);

	static void _bind_methods();

private:

	// DATA
	SubGridMetadata _meta;
	SubGridChunkMap _chunks;
	String _saves_dir;

	Ref<VoxelMesherBlocky> _mesher;
	Ref<VoxelBlockyLibrary> _library;

	// LOD
	Node3D *_viewer = nullptr;

	static const float LOD_DISTANCES[4]; // {32, 64, 128, 256}

	// sub-contraption state (children only)
	float _angular_speed_rpm = 0.0f;
	double _target_angle_rad = 0.0;
	bool _redstone_signal = false;
	LockMode _lock_mode = LOCKED_DEFAULT;

	// Typed list of child VoxelSubGrid nodes for fast iteration. (Godot child nodes are the authority - this is a cache)
	Vector<ObjectID> _children;

	// INTERNAL METHODS
	void _save_chunk(Vector3i chunk_pos);
	bool _should_lock() const;

	SubGridManager *_manager = nullptr;

	void _disassemble_root_to_terrain(VoxelLodTerrain *terrain);
	void _disassemble_child_to_parent();

};

} // namespace zylann::voxel