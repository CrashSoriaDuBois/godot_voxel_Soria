// subgrid/streaming/sub_grid_stream_helper.cpp

#include "sub_grid_stream_helper.h"
#include "core/io/dir_access.h"
#include "streams/sqlite/voxel_stream_sqlite.h"
#include "sub_grid_stream_helper.h"
#include "core/io/dir_access.h"
#include "core/config/project_settings.h"
#include "streams/sqlite/voxel_stream_sqlite.h"

namespace zylann::voxel {

String SubGridStreamHelper::uuid_to_filename(const uint8_t *u) {
	String s;
	for (int i = 0; i < 16; i++) {
		s += String::num_int64(u[i] >> 4, 16);
		s += String::num_int64(u[i] & 0xF, 16);
	}
	return s + ".sqlite";
}

String SubGridStreamHelper::uuid_to_path(const String &saves_dir, const uint8_t *uuid) {
	String saves_dir_abs = ProjectSettings::get_singleton()->globalize_path(saves_dir);
	return saves_dir_abs.path_join("ships").path_join(uuid_to_filename(uuid));
}

Ref<VoxelStreamSQLite> SubGridStreamHelper::open(const String &saves_dir, const uint8_t *uuid) {
	// Globalize once and use the same path for both dir creation and stream
	String saves_dir_abs = ProjectSettings::get_singleton()->globalize_path(saves_dir);
	String ships_dir_abs = saves_dir_abs.path_join("ships");

	Error err = DirAccess::make_dir_recursive_absolute(ships_dir_abs);
	if (err != OK) {
		print_line(String("Failed to create ships dir: ") + ships_dir_abs + String(" err=") + itos(err));
	} else {
		print_line(String("Ships dir ready: ") + ships_dir_abs);
	}

	// Use the globalized absolute path - not the virtual user:// path
	// This ensures SQLite and DirAccess use the exact same path
	String db_path = ships_dir_abs.path_join(uuid_to_filename(uuid));
	print_line(String("Opening DB at: ") + db_path);

	Ref<VoxelStreamSQLite> stream;
	stream.instantiate();
	stream->set_database_path(db_path);
	return stream;
}

void SubGridStreamHelper::close(Ref<VoxelStreamSQLite> &stream) {
	if (stream.is_valid()) {
		stream->set_database_path("");
		stream.unref();
	}
}

void SubGridStreamHelper::close_and_delete(
		Ref<VoxelStreamSQLite> &stream,
		const String &saves_dir,
		const uint8_t *uuid
) {
	String path = uuid_to_path(saves_dir, uuid);
	close(stream);
	DirAccess::remove_absolute(path);
}

} // namespace zylann::voxel