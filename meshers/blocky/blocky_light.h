#ifndef VOXEL_BLOCKY_LIGHT_H
#define VOXEL_BLOCKY_LIGHT_H

#include "../../util/containers/span.h"
#include "../../util/containers/std_vector.h"
#include "../../util/math/vector3i.h"
#include "blocky_baked_library.h"

namespace zylann::voxel::blocky {

// Declared here so mesh_block_task.cpp can call it.
// Implemented as explicit instantiations in blocky_light.cpp.

void flood_fill_light(
		Span<const uint8_t> type_buffer,
		Vector3i block_size,
		const BakedLibrary &library,
		StdVector<uint8_t> &out_light
);

void flood_fill_light(
		Span<const uint16_t> type_buffer,
		Vector3i block_size,
		const BakedLibrary &library,
		StdVector<uint8_t> &out_light
);

} // namespace zylann::voxel::blocky

#endif // VOXEL_BLOCKY_LIGHT_H
