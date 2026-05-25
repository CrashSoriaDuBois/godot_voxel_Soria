#include "../../subgrid/sub_grid_metadata.h"  
#include "test_subgrid.h"
#include "../../storage/voxel_buffer.h"
#include "../../subgrid/lod/sub_grid_chunk_map.h"
#include "../../subgrid/streaming/sub_grid_stream_helper.h"
#include "../../util/testing/test_macros.h"
#include "core/io/dir_access.h"
#include "core/io/file_access.h"
#include "core/config/project_settings.h"

namespace zylann::voxel::tests {

namespace {

bool file_exists_at(const String &path) {
	Ref<FileAccess> f = FileAccess::open(path, FileAccess::READ);
	return f.is_valid();
}

} // anonymous namespace

void test_subgrid_chunk_map_set_get() {
	SubGridChunkMap map;

	map.set_voxel(42, Vector3i(0, 0, 0), VoxelBuffer::CHANNEL_TYPE);
	map.set_voxel(7, Vector3i(15, 15, 15), VoxelBuffer::CHANNEL_TYPE);
	map.set_voxel(99, Vector3i(16, 0, 0), VoxelBuffer::CHANNEL_TYPE);

	ZN_TEST_ASSERT(map.get_voxel(Vector3i(0, 0, 0), VoxelBuffer::CHANNEL_TYPE) == 42);
	ZN_TEST_ASSERT(map.get_voxel(Vector3i(15, 15, 15), VoxelBuffer::CHANNEL_TYPE) == 7);
	ZN_TEST_ASSERT(map.get_voxel(Vector3i(16, 0, 0), VoxelBuffer::CHANNEL_TYPE) == 99);
	ZN_TEST_ASSERT(map.get_voxel(Vector3i(5, 5, 5), VoxelBuffer::CHANNEL_TYPE) == 0);

	Vector<Vector3i> dirty = map.get_dirty_chunks();
	ZN_TEST_ASSERT(dirty.size() == 2);

	for (Vector3i pos : dirty) {
		map.mark_chunk_clean(pos);
	}
	ZN_TEST_ASSERT(map.get_dirty_chunks().size() == 0);
	ZN_TEST_ASSERT(map.get_voxel(Vector3i(100, 100, 100), VoxelBuffer::CHANNEL_TYPE) == 0);
}

void test_subgrid_sqlite_lifecycle() {
	String saves_dir_virtual = "user://test_subgrid_saves";
	String saves_dir = ProjectSettings::get_singleton()->globalize_path(saves_dir_virtual);

	Error err = DirAccess::make_dir_recursive_absolute(saves_dir);
	ZN_TEST_ASSERT_MSG(err == OK, String("Failed to create dir: ") + saves_dir + " err=" + itos(err));

	uint8_t uuid[16] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

	Ref<VoxelStreamSQLite> stream = SubGridStreamHelper::open(saves_dir_virtual, uuid);
	ZN_TEST_ASSERT(stream.is_valid());

	// Force the stream to actually open the database by saving a block
	// VoxelStreamSQLite is lazy - the file is created on first write, not on set_database_path
	{
		VoxelBuffer vb(VoxelBuffer::ALLOCATOR_DEFAULT);
		vb.create(16, 16, 16);
		vb.fill(1, VoxelBuffer::CHANNEL_TYPE);
		VoxelStream::VoxelQueryData q{ vb, Vector3i(0, 0, 0), 0, VoxelStream::RESULT_BLOCK_NOT_FOUND };
		stream->save_voxel_block(q);
		stream->flush(); // ensure it is written to disk
	}

	String path = ProjectSettings::get_singleton()->globalize_path(
			saves_dir_virtual.path_join("ships").path_join(SubGridStreamHelper::uuid_to_filename(uuid))
	);
	print_line(String("Expected SQLite file at: ") + path);
	ZN_TEST_ASSERT_MSG(file_exists_at(path), String("SQLite file not found at: ") + path);

	SubGridStreamHelper::close(stream);
	ZN_TEST_ASSERT(!stream.is_valid());

	// Re-open, load back the block, verify it round-trips
	Ref<VoxelStreamSQLite> stream2 = SubGridStreamHelper::open(saves_dir_virtual, uuid);
	ZN_TEST_ASSERT(stream2.is_valid());
	{
		VoxelBuffer vb(VoxelBuffer::ALLOCATOR_DEFAULT);
		vb.create(16, 16, 16);
		VoxelStream::VoxelQueryData q{ vb, Vector3i(0, 0, 0), 0, VoxelStream::RESULT_BLOCK_NOT_FOUND };
		stream2->load_voxel_block(q);
		ZN_TEST_ASSERT(q.result == VoxelStream::RESULT_BLOCK_FOUND);
		ZN_TEST_ASSERT(vb.get_voxel(0, 0, 0, VoxelBuffer::CHANNEL_TYPE) == 1);
	}

	SubGridStreamHelper::close_and_delete(stream2, saves_dir, uuid);
	ZN_TEST_ASSERT(!file_exists_at(path));
}

void test_subgrid_metadata_serialization() {
	SubGridMetadata original;
	for (int i = 0; i < 16; i++) {
		original.uuid[i] = (uint8_t)i;
		original.parent_uuid[i] = (uint8_t)(i + 16);
	}
	original.is_root = true;
	original.pivot_in_parent_local = Vector3i(3, 2, 1);
	original.rotation_axis = Vector3i(0, 1, 0);
	original.target_angle_rad = 1.23456789;
	original.lock_mode = SubGridMetadata::LOCKED_DEFAULT;
	original.world_position = Vector3(10.5f, 20.5f, 30.5f);
	original.world_rotation = Quaternion(0.1f, 0.2f, 0.3f, 0.9f).normalized();

	PackedByteArray bytes = original.serialize();
	ZN_TEST_ASSERT(bytes.size() > 0);

	SubGridMetadata restored = SubGridMetadata::deserialize(bytes);

	ZN_TEST_ASSERT(memcmp(original.uuid, restored.uuid, 16) == 0);
	ZN_TEST_ASSERT(memcmp(original.parent_uuid, restored.parent_uuid, 16) == 0);
	ZN_TEST_ASSERT(restored.is_root == original.is_root);
	ZN_TEST_ASSERT(restored.pivot_in_parent_local == original.pivot_in_parent_local);
	ZN_TEST_ASSERT(restored.rotation_axis == original.rotation_axis);
	ZN_TEST_ASSERT(Math::is_equal_approx((float)restored.target_angle_rad, (float)original.target_angle_rad));
	ZN_TEST_ASSERT(restored.lock_mode == original.lock_mode);
	ZN_TEST_ASSERT(restored.world_position.is_equal_approx(original.world_position));
}

} // namespace zylann::voxel::tests