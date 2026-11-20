#ifndef VOXEL_BLOCKY_LIBRARY_INDICES_DIALOG_H
#define VOXEL_BLOCKY_LIBRARY_INDICES_DIALOG_H

#include "../../meshers/blocky/voxel_blocky_library.h"
#include "../../util/godot/classes/accept_dialog.h"
#include "../../util/godot/macros.h"

ZN_GODOT_FORWARD_DECLARE(class ItemList);

namespace zylann::voxel {

// Lists the models of a VoxelBlockyLibrary by index, highlighting the indices occupied by rotation groups.
// The list is the same as the array of the library, it only adds information to it.
class VoxelBlockyLibraryIndicesDialog : public AcceptDialog {
	GDCLASS(VoxelBlockyLibraryIndicesDialog, AcceptDialog)
public:
	VoxelBlockyLibraryIndicesDialog();

	void set_library(Ref<VoxelBlockyLibrary> library);

private:
	// When compiling with GodotCpp, `_bind_methods` isn't optional.
	static void _bind_methods() {}

	ItemList *_item_list = nullptr;
};

} // namespace zylann::voxel

#endif // VOXEL_BLOCKY_LIBRARY_INDICES_DIALOG_H
