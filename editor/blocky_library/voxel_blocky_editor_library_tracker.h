#ifndef VOXEL_BLOCKY_EDITOR_LIBRARY_TRACKER_H
#define VOXEL_BLOCKY_EDITOR_LIBRARY_TRACKER_H

#include "../../meshers/blocky/voxel_blocky_library_base.h"

namespace zylann::voxel {

// Editor-only helpers. Models don't know which library they belong to, but some of their editors need library-level settings (tile grid size, preview texture).
// The last library that was inspected is remembered here (weakly).
void set_last_inspected_blocky_library(VoxelBlockyLibraryBase *library);
Ref<VoxelBlockyLibraryBase> get_last_inspected_blocky_library();

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_EDITOR_LIBRARY_TRACKER_H
