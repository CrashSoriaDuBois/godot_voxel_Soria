#include "sub_grid_manager.h"
#include "core/config/project_settings.h"
#include "core/io/file_access.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include <vector>

namespace zylann::voxel {

void SubGridManager::_bind_methods() {
	ClassDB::bind_method(
			D_METHOD("initialize", "terrain", "saves_dir", "mesher", "library"), &SubGridManager::initialize
	);
	ClassDB::bind_method(D_METHOD("save_all"), &SubGridManager::save_all);
	ClassDB::bind_method(D_METHOD("load_all"), &SubGridManager::load_all);
}

void SubGridManager::_notification(int p_what) {
	if (p_what == NOTIFICATION_EXIT_TREE) {
		print_line("SubGridManager EXIT_TREE - calling save_all");
		//save_all();
	}
}

void SubGridManager::initialize(
		VoxelLodTerrain *terrain,
		const String &saves_dir,
		Ref<VoxelMesherBlocky> mesher,
		Ref<VoxelBlockyLibrary> library
) {
	_terrain = terrain;
	_saves_dir = saves_dir;
	_mesher = mesher;
	_library = library;
}

void SubGridManager::save_all() {
	print_line("save_all called");
	Vector<SubGridMetadata> metas;
	Node *parent = get_parent();
	if (parent == nullptr)
		return;

	for (int i = 0; i < parent->get_child_count(); i++) {
		VoxelSubGrid *sg = Object::cast_to<VoxelSubGrid>(parent->get_child(i));
		if (sg != nullptr && sg->is_root()) {
			sg->flush_dirty_chunks();
			sg->get_metadata_mut().world_position = sg->get_global_position();
			sg->get_metadata_mut().world_rotation = sg->get_global_basis().get_rotation_quaternion();
			print_line(String("Saving ship at: ") + String(sg->get_metadata().world_position));
			_collect_metadata_recursive(sg, metas);
		}
	}
	_save_metadata_index(metas);
	print_line(String("Saved ") + itos(metas.size()) + " subgrid(s) to index");
}

void SubGridManager::_collect_metadata_recursive(VoxelSubGrid *sg, Vector<SubGridMetadata> &out) {
	// Update rotation angle in metadata before saving
	// target_angle_rad is already kept in sync by _update_rotation so just flush chunks and collect
	sg->flush_dirty_chunks();
	out.push_back(sg->get_metadata());

	for (int i = 0; i < sg->get_child_count(); i++) {
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(sg->get_child(i));
		if (child != nullptr) {
			_collect_metadata_recursive(child, out);
		}
	}
}

void SubGridManager::_save_metadata_index(const Vector<SubGridMetadata> &metas) {
	String abs_path = ProjectSettings::get_singleton()->globalize_path(_saves_dir + "/ships_index.bin");

	std::vector<uint8_t> buf;

	auto write4 = [&](int32_t v) {
		uint8_t tmp[4];
		memcpy(tmp, &v, 4);
		buf.insert(buf.end(), tmp, tmp + 4);
	};

	write4((int32_t)metas.size());

	for (const SubGridMetadata &meta : metas) {
		PackedByteArray blob = meta.serialize();
		int bsize = blob.size();

		// Verify blob content before copying
		const uint8_t *bptr = blob.ptr();
		print_line(
				String("blob[66..69] before copy: ") + String::num_int64(bptr[66], 16) + " " +
				String::num_int64(bptr[67], 16) + " " + String::num_int64(bptr[68], 16) + " " +
				String::num_int64(bptr[69], 16)
		);

		write4(bsize);
		buf.insert(buf.end(), bptr, bptr + bsize);

		// Verify in buf
		int world_pos_offset_in_buf = (int)buf.size() - bsize + 66;
		print_line(
				String("buf[world_pos_offset..+3]: ") + String::num_int64(buf[world_pos_offset_in_buf], 16) + " " +
				String::num_int64(buf[world_pos_offset_in_buf + 1], 16) + " " +
				String::num_int64(buf[world_pos_offset_in_buf + 2], 16) + " " +
				String::num_int64(buf[world_pos_offset_in_buf + 3], 16)
		);
	}

	Ref<FileAccess> f = FileAccess::open(abs_path, FileAccess::WRITE);
	ERR_FAIL_COND_MSG(!f.is_valid(), String("Failed to open: ") + abs_path);
	f->store_buffer(buf.data(), buf.size());
	f->flush();
	print_line(String("Wrote ") + itos(buf.size()) + " bytes");
}

void SubGridManager::load_all() {
	Vector<SubGridMetadata> metas = _load_metadata_index();
	if (metas.is_empty()) {
		print_line("No saved ships found");
		return;
	}

	HashMap<String, VoxelSubGrid *> uuid_to_node;
	Node *parent = get_parent();

	// Roots first (depth ordering - roots before children)
	for (const SubGridMetadata &meta : metas) {
		if (!meta.is_root)
			continue;

		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		parent->add_child(sg);
		sg->initialize_root_from_disk(meta, _saves_dir, _mesher, _library);
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
		print_line(String("Loaded root ship at ") + String(meta.world_position));
	}

	// Children (parents guaranteed to exist now)
	for (const SubGridMetadata &meta : metas) {
		if (meta.is_root)
			continue;

		String parent_uuid_str = _uuid_to_string(meta.parent_uuid);
		VoxelSubGrid **parent_sg = uuid_to_node.getptr(parent_uuid_str);
		ERR_CONTINUE_MSG(parent_sg == nullptr, String("Parent not found for child subgrid"));

		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		(*parent_sg)->add_child(sg); // add before initialize so parent is set
		sg->initialize_child(meta, SubGridChunkMap(), _saves_dir);
		//load chunks from disk
		sg->load_chunks_from_stream();
		sg->rebuild_all_meshes();
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

	print_line(String("Loaded ") + itos(metas.size()) + " subgrid(s)");
}

Vector<SubGridMetadata> SubGridManager::_load_metadata_index() {
	Vector<SubGridMetadata> result;

	String abs_path = ProjectSettings::get_singleton()->globalize_path(_saves_dir + "/ships_index.bin");
	print_line(String("Reading index from: ") + abs_path);

	Ref<FileAccess> check = FileAccess::open(abs_path, FileAccess::READ);
	if (!check.is_valid()) {
		print_line("Index file not found");
		return result;
	}
	check.unref();

	Ref<FileAccess> f = FileAccess::open(abs_path, FileAccess::READ);
	ERR_FAIL_COND_V_MSG(!f.is_valid(), result, String("Failed to open index for reading: ") + abs_path);

	int count = f->get_32();
	print_line(String("Index contains ") + itos(count) + " entries");

	for (int i = 0; i < count; i++) {
		int blob_size = f->get_32();
		print_line(String("Reading blob of size: ") + itos(blob_size));
		PackedByteArray bytes = f->get_buffer(blob_size);
		print_line(String("Actually read ") + itos(bytes.size()) + " bytes");

		// Verify bytes at world_position offset before deserializing
		const uint8_t *p = bytes.ptr();
		print_line(
				String("file bytes[66..69]: ") + String::num_int64(p[66], 16) + " " + String::num_int64(p[67], 16) +
				" " + String::num_int64(p[68], 16) + " " + String::num_int64(p[69], 16)
		);

		SubGridMetadata meta = SubGridMetadata::deserialize(bytes);
		print_line(String("After deserialize world_position: ") + String(meta.world_position));
		result.push_back(meta);
	}
	return result;
}

} // namespace zylann::voxel