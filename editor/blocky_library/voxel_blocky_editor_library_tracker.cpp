#include "voxel_blocky_editor_library_tracker.h"
#include "../../util/godot/object_weak_ref.h"

namespace zylann::voxel {

namespace {
zylann::godot::ObjectWeakRef<VoxelBlockyLibraryBase> g_last_inspected_library;
}

void set_last_inspected_blocky_library(VoxelBlockyLibraryBase *library) {
	g_last_inspected_library.set(library);
}

Ref<VoxelBlockyLibraryBase> get_last_inspected_blocky_library() {
	return Ref<VoxelBlockyLibraryBase>(g_last_inspected_library.get());
}

} // namespace zylann::voxel
