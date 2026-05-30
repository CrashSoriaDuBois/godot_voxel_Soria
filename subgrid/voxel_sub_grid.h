#pragma once

#include "lod/sub_grid_chunk_map.h"
#include "meshers/blocky/voxel_blocky_library.h"
#include "meshers/blocky/voxel_mesher_blocky.h"
#include "scene/3d/mesh_instance_3d.h"
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
	// --- lock mode (mirrors SwivelBearingBlockEntity::LockingSetting) ---
	enum LockMode : uint8_t { LOCKED_ALWAYS = 0, LOCKED_DEFAULT = 1, UNLOCKED_DEFAULT = 2, UNLOCKED_ALWAYS = 3 };

	// --- lifecycle ---
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

	void initialize_child(const SubGridMetadata &meta, SubGridChunkMap &&chunks, const String &saves_dir);

	SubGridMetadata &get_metadata_mut() { 
		return _meta; 
	}

	// --- editing ---
	uint32_t get_voxel(Vector3i local_pos, int channel) const;
	void set_voxel(uint32_t value, Vector3i local_pos, int channel);

	Ref<VoxelToolSubGrid> get_voxel_tool();
	void set_manager(SubGridManager *manager);

	// --- kinetics (children only) ---
	void set_angular_speed_rpm(float rpm) {
		_angular_speed_rpm = rpm;
	}
	void set_redstone_signal(bool powered);

	// --- LOD ---
	void set_viewer(Node3D *viewer) {
		_viewer = viewer;
	}

	// --- persistence ---
	void flush_dirty_chunks();
	void save_and_close();
	void destroy();
	void disassemble_to_terrain(class VoxelLodTerrain *terrain);

	// --- accessors ---
	const SubGridMetadata &get_metadata() const {
		return _meta;
	}
	bool is_root() const {
		return _meta.is_root;
	}
	VoxelSubGrid *get_root();

	bool is_rotation_grid_aligned(float tolerance_degrees) const;
	bool is_tree_grid_aligned(float tolerance_degrees) const;

	//void rebuild_all_meshes();
	void load_chunks_from_stream();

	// Called by SubGridManager on the main thread after a mesh task completes.
	void apply_chunk_mesh(Vector3i chunk_pos, int lod, Ref<ArrayMesh> mesh);

	// Removes all LOD variants of a chunk's mesh EXCEPT the given lod.
	// Called by SubGridManager when a LOD transition is detected.
	void remove_chunk_meshes_except(Vector3i chunk_pos, int keep_lod);
	
	// Read-only access to chunk data for SubGridManager (padding, LOD queries).
	const SubGridChunkMap &get_chunk_map() const { return _chunks; }
	
	// Read-only access to the viewer so SubGridManager can compute LOD.
	Node3D *get_viewer() const { return _viewer; }

	void set_target_angle_rad(double a) {_target_angle_rad = a;_meta.target_angle_rad = a;_apply_transform_from_angle(a);}
	double get_target_angle_rad() const {return _target_angle_rad;}

protected:
	// Godot virtuals - must be inside the class body
	void _notification(int p_what);

	static void _bind_methods();

private:
	//void _process_lod();
	void _process_rotation(); 

	// --- data ---
	SubGridMetadata _meta;
	SubGridChunkMap _chunks;
	Ref<VoxelStreamSQLite> _stream;
	String _saves_dir;

	Ref<VoxelMesherBlocky> _mesher;
	Ref<VoxelBlockyLibrary> _library;

	// --- rendering ---
	// Key encodes (chunk_pos, lod). see _chunk_mesh_key()
	HashMap<uint64_t, MeshInstance3D *> _mesh_nodes;

	// --- LOD ---
	Node3D *_viewer = nullptr;
	//HashMap<Vector3i, int> _chunk_current_lod;

	static const float LOD_DISTANCES[4]; // {32, 64, 128, 256}

	// --- sub-contraption state (children only) ---
	float _angular_speed_rpm = 0.0f;
	double _target_angle_rad = 0.0;
	double _last_target_angle_rad = 0.0;
	bool _redstone_signal = false;
	LockMode _lock_mode = LOCKED_DEFAULT;

	// Typed list of child VoxelSubGrid nodes for fast iteration
	// (Godot child nodes are the authority - this is a cache)
	Vector<VoxelSubGrid *> _children;

	// --- internal methods ---
	//void _build_chunk_mesh_at_lod(Vector3i chunk_pos, int lod);
	void _save_chunk(Vector3i chunk_pos);

	void _update_rotation(double delta);
	void _apply_transform_from_angle(double angle_rad);
	bool _should_lock() const;

	SubGridManager *_manager = nullptr;

	//int _lod_for_chunk(Vector3i chunk_pos) const;

	static uint64_t _chunk_mesh_key(Vector3i p, int lod) {
		return ((uint64_t)(uint16_t)p.x) | ((uint64_t)(uint16_t)p.y << 16) | ((uint64_t)(uint16_t)p.z << 32) |
				((uint64_t)(uint8_t)lod << 48);
	}
};

} // namespace zylann::voxel