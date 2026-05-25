#pragma once

#include "meshers/blocky/voxel_blocky_library.h"
#include "meshers/blocky/voxel_mesher_blocky.h"
#include "scene/main/node.h"
#include "sub_grid_metadata.h"
#include "voxel_sub_grid.h"

namespace zylann::voxel {

class VoxelLodTerrain;

class SubGridManager : public Node {
	GDCLASS(SubGridManager, Node)

public:
	void initialize(
			VoxelLodTerrain *terrain,
			const String &saves_dir,
			Ref<VoxelMesherBlocky> mesher,
			Ref<VoxelBlockyLibrary> library
	);

	void save_all();
	void load_all();

protected:
	void _notification(int p_what);
	static void _bind_methods();

private:
	VoxelLodTerrain *_terrain = nullptr;
	String _saves_dir;
	Ref<VoxelMesherBlocky> _mesher;
	Ref<VoxelBlockyLibrary> _library;

	void _save_metadata_index(const Vector<SubGridMetadata> &metas);
	Vector<SubGridMetadata> _load_metadata_index();
	void _collect_metadata_recursive(VoxelSubGrid *sg, Vector<SubGridMetadata> &out);

	static String _uuid_to_string(const uint8_t *uuid) {
		String s;
		for (int i = 0; i < 16; i++) {
			s += String::num_int64(uuid[i] >> 4, 16);
			s += String::num_int64(uuid[i] & 0xF, 16);
		}
		return s;
	}
};

} // namespace zylann::voxel