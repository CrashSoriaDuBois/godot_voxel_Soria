#ifndef VOXEL_STREAM_SQLITE_CONNECTION_H
#define VOXEL_STREAM_SQLITE_CONNECTION_H

#include "../../storage/voxel_buffer.h"
#include "../../util/string/std_string.h"
#include "../voxel_stream.h"
#include "block_location.h"
#include "entity_types.h"

struct sqlite3;
struct sqlite3_stmt;

namespace zylann::voxel::sqlite {

// One connection to the database, with our prepared statements
class Connection {
public:
	static constexpr int VERSION_V0 = 0;
	static constexpr int VERSION_V1 = 1;
	static constexpr int VERSION_LATEST = VERSION_V1;

	struct Meta {
		int version = -1;
		int block_size_po2 = 0;
		BlockLocation::CoordinateFormat coordinate_format =
				// Default as of V0
				BlockLocation::FORMAT_INT64_X16_Y16_Z16_L16;

		struct Channel {
			VoxelBuffer::Depth depth;
			bool used = false;
		};

		FixedArray<Channel, VoxelBuffer::MAX_CHANNELS> channels;
	};

	enum BlockType { //
		VOXELS,
		INSTANCES
	};

	Connection();
	~Connection();

	bool open(const char *fpath, const BlockLocation::CoordinateFormat preferred_coordinate_format);
	void close();

	bool is_open() const {
		return _db != nullptr;
	}

	// Returns the file path from SQLite
	const char *get_file_path() const;

	// Return the file path that was used to open the connection.
	// You may use this one if you want determinism, as SQLite seems to globalize its path.
	const char *get_opened_file_path() const {
		return _opened_path.c_str();
	}

	bool begin_transaction();
	bool end_transaction();

	bool save_block(const BlockLocation loc, const Span<const uint8_t> block_data, const BlockType type);

	VoxelStream::ResultCode load_block(
			const BlockLocation loc,
			StdVector<uint8_t> &out_block_data,
			const BlockType type
	);

	bool load_all_blocks(
			void *callback_data,
			void (*process_block_func)(
					void *callback_data,
					BlockLocation location,
					Span<const uint8_t> voxel_data,
					Span<const uint8_t> instances_data
			)
	);

	bool load_all_block_keys(
			void *callback_data,
			void (*process_block_func)(void *callback_data, BlockLocation location)
	);

	const Meta &get_meta() const {
		return _meta;
	}

	void migrate_to_latest_version();

	bool begin_write_transaction(); // BEGIN IMMEDIATE
	bool rollback_transaction();

	// Entities. All write operations are atomic: on any failure nothing is written.
	bool add_entity_groups(Span<const EntityGroup> groups);
	bool add_entity_children(Vector3i root, Span<const Vector3i> children);
	bool update_entity_data(Vector3i voxel, Span<const uint8_t> data);
	bool read_entities(Span<const Vector3i> voxels, StdVector<EntityRow> &out);
	bool read_entity_children(Vector3i root, StdVector<Vector3i> &out);
	bool remove_entities(Span<const Vector3i> voxels, StdVector<EntityPromotion> &out_promotions);
	bool list_entity_voxels_in_chunk(Vector3i chunk, StdVector<Vector3i> &out);


	bool save_chunk_last_modified(const BlockLocation loc, double timestamp);
	double load_chunk_last_modified(const BlockLocation loc); // returns -1 if not found

	bool save_chunk_last_modified_batch(Span<const BlockLocation> locs, double timestamp);

private:
	int load_version();
	Meta load_meta();
	void save_meta(Meta meta);
	bool migrate_to_next_version();
	bool migrate_from_v0_to_v1();

	StdString _opened_path;
	Meta _meta;
	sqlite3 *_db = nullptr;
	sqlite3_stmt *_load_version_statement = nullptr;
	sqlite3_stmt *_begin_statement = nullptr;
	sqlite3_stmt *_end_statement = nullptr;
	sqlite3_stmt *_update_voxel_block_statement = nullptr;
	sqlite3_stmt *_get_voxel_block_statement = nullptr;
	sqlite3_stmt *_update_instance_block_statement = nullptr;
	sqlite3_stmt *_get_instance_block_statement = nullptr;
	sqlite3_stmt *_load_meta_statement = nullptr;
	sqlite3_stmt *_save_meta_statement = nullptr;
	sqlite3_stmt *_load_channels_statement = nullptr;
	sqlite3_stmt *_save_channel_statement = nullptr;
	sqlite3_stmt *_load_all_blocks_statement = nullptr;
	sqlite3_stmt *_load_all_block_keys_statement = nullptr;

	sqlite3_stmt *_save_chunk_meta_statement = nullptr;
	sqlite3_stmt *_load_chunk_meta_statement = nullptr;

	bool entity_insert(Vector3i voxel, int action_type, Span<const uint8_t> data, bool has_parent, Vector3i parent);
	bool entity_select(Vector3i voxel, EntityRow &out);
	bool entity_delete(Vector3i voxel);
	bool entity_set_data(Vector3i voxel, Span<const uint8_t> data);
	bool entity_promote(Vector3i voxel, Span<const uint8_t> data);
	bool entity_reparent(Vector3i old_root, Vector3i new_root);
	bool entity_children(Vector3i root, StdVector<Vector3i> &out);

	sqlite3_stmt *_begin_immediate_statement = nullptr;
	sqlite3_stmt *_rollback_statement = nullptr;
	sqlite3_stmt *_entity_insert_statement = nullptr;
	sqlite3_stmt *_entity_select_statement = nullptr;
	sqlite3_stmt *_entity_delete_statement = nullptr;
	sqlite3_stmt *_entity_set_data_statement = nullptr;
	sqlite3_stmt *_entity_promote_statement = nullptr;
	sqlite3_stmt *_entity_reparent_statement = nullptr;
	sqlite3_stmt *_entity_children_statement = nullptr;
	sqlite3_stmt *_entity_chunk_list_statement = nullptr;
};

} // namespace zylann::voxel::sqlite

#endif // VOXEL_STREAM_SQLITE_CONNECTION_H
