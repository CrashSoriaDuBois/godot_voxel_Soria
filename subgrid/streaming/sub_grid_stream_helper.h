#pragma once
#include "core/string/ustring.h"
#include "streams/sqlite/voxel_stream_sqlite.h"

namespace zylann::voxel {

class SubGridStreamHelper {
public:
	// Opens (creates if needed) the SQLite file for this UUID.
	// saves_dir must already exist.
	// Returns the open stream - caller stores it as Ref<VoxelStreamSQLite>.
	static Ref<VoxelStreamSQLite> open(const String &saves_dir, const uint8_t *uuid_16bytes);

	// Cleanly closes the stream then deletes its file.
	// Call ONLY after all pending saves have flushed.
	// Pass the stream Ref by reference so it can be nulled after close.
	static void close_and_delete(Ref<VoxelStreamSQLite> &stream, const String &saves_dir, const uint8_t *uuid_16bytes);

	// Just close without deleting (ship is being unloaded, not destroyed).
	static void close(Ref<VoxelStreamSQLite> &stream);

	static String uuid_to_filename(const uint8_t *uuid_16bytes);
	static String uuid_to_path(const String &saves_dir, const uint8_t *uuid_16bytes);
};

} // namespace zylann::voxel