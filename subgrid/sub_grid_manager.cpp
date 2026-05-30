#include "sub_grid_manager.h"
#include "core/config/project_settings.h"
#include "core/io/file_access.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include <chrono>
#include <vector>

namespace zylann::voxel {

// Must match VoxelSubGrid::LOD_DISTANCES.
const float SubGridManager::LOD_DISTANCES[4] = { 32.f, 64.f, 128.f, 256.f };

// ____________________________________________________________________________
// Godot hooks

void SubGridManager::_bind_methods() {
	ClassDB::bind_method(
			D_METHOD("initialize", "terrain", "saves_dir", "mesher", "library"), &SubGridManager::initialize
	);
	ClassDB::bind_method(D_METHOD("save_all"), &SubGridManager::save_all);
	ClassDB::bind_method(D_METHOD("load_all"), &SubGridManager::load_all);
}

void SubGridManager::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			set_process(true);
			break;

		case NOTIFICATION_PROCESS:
			_process_all_ships();
			break;

		case NOTIFICATION_EXIT_TREE:
			// Let in-flight futures finish so we don't leave dangling thread state.
			for (auto &f : _pending_futures) {
				f.wait();
			}
			_pending_futures.clear();
			break;
	}
}

// ____________________________________________________________________________
// Setup

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

// ____________________________________________________________________________
// Ship lifecycle

void SubGridManager::_register_single(VoxelSubGrid *sg) {
	ERR_FAIL_COND(sg == nullptr);
	String uuid = _uuid_to_string(sg->get_metadata().uuid);

	ShipState &state = _ships[uuid];
	state.node = sg;
	state.dirty_chunks.clear();
	state.in_flight_chunks.clear();
	state.current_lod.clear();
	_mark_all_dirty(state);
}

void SubGridManager::register_ship_tree(VoxelSubGrid *root) {
	ERR_FAIL_COND(root == nullptr);
	_register_single(root);

	// Register all child sub-contraptions too.
	for (int i = 0; i < root->get_child_count(); i++) {
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(root->get_child(i));
		if (child != nullptr) {
			register_ship_tree(child); // recursive
		}
	}
}

void SubGridManager::unregister_ship(const String &uuid_str) {
	_ships.erase(uuid_str);
}

void SubGridManager::mark_chunk_dirty(const String &uuid_str, Vector3i chunk_pos) {
	ShipState *state = _ships.getptr(uuid_str);
	if (state != nullptr && !state->in_flight_chunks.has(chunk_pos)) {
		state->dirty_chunks.insert(chunk_pos);
	}
}

void SubGridManager::_mark_all_dirty(ShipState &state) {
	if (state.node == nullptr) {
		return;
	}
	const HashSet<Vector3i> &positions = state.node->get_chunk_map().get_all_chunk_positions();
	for (const Vector3i &pos : positions) {
		if (!state.in_flight_chunks.has(pos)) {
			state.dirty_chunks.insert(pos);
		}
	}
}

// ____________________________________________________________________________
// _process pipeline

void SubGridManager::_process_all_ships() {
	// 1. Detect LOD changes and mark chunks dirty.
	for (auto &[uuid, state] : _ships) {
		_process_lod_for_ship(uuid, state);
	}

	// 2. Submit tasks for dirty chunks up to the concurrency limit.
	for (auto &[uuid, state] : _ships) {
		_submit_pending_tasks(uuid, state);
	}

	// 3. Poll finished futures and upload meshes.
	_poll_completed_tasks();
}

void SubGridManager::_process_lod_for_ship(const String &uuid, ShipState &state) {
	if (state.node == nullptr) {
		return;
	}
	const HashSet<Vector3i> &all_chunks = state.node->get_chunk_map().get_all_chunk_positions();

	for (const Vector3i &chunk_pos : all_chunks) {
		int desired_lod = _compute_lod(state.node, chunk_pos);

		int *current = state.current_lod.getptr(chunk_pos);
		if (current == nullptr || *current != desired_lod) {
			// LOD changed. remove stale meshes and mark dirty.
			if (current != nullptr) {
				state.node->remove_chunk_meshes_except(chunk_pos, desired_lod);
			}
			state.current_lod[chunk_pos] = desired_lod;
			if (!state.in_flight_chunks.has(chunk_pos)) {
				state.dirty_chunks.insert(chunk_pos);
			}
		}
	}
}

void SubGridManager::_submit_pending_tasks(const String &uuid, ShipState &state) {
	if (state.dirty_chunks.is_empty()) {
		return;
	}
	// Collect chunk positions first to avoid mutating the set while iterating.
	Vector<Vector3i> to_submit;
	for (const Vector3i &pos : state.dirty_chunks) {
		to_submit.push_back(pos);
	}
	for (const Vector3i &chunk_pos : to_submit) {
		if (_total_in_flight() >= MAX_CONCURRENT_TASKS) {
			break;
		}
		int lod = 0;
		int *l = state.current_lod.getptr(chunk_pos);
		if (l != nullptr) {
			lod = *l;
		}
		_submit_one_task(uuid, state, chunk_pos, lod);
	}
}

void SubGridManager::_poll_completed_tasks() {
	for (auto it = _pending_futures.begin(); it != _pending_futures.end();) {
		if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
			SubGridMeshTaskResult result = it->get();
			_apply_mesh_result(result);
			it = _pending_futures.erase(it);
		} else {
			++it;
		}
	}
}

// ____________________________________________________________________________
// Task submission and result application

void SubGridManager::_submit_one_task(const String &uuid, ShipState &state, Vector3i chunk_pos, int lod) {
	std::shared_ptr<VoxelBuffer> padded = _build_padded_buffer(state.node, chunk_pos);
	if (!padded) {
		// Chunk buffer doesn't exist yet (possible during streaming), skip.
		state.dirty_chunks.erase(chunk_pos);
		return;
	}

	state.dirty_chunks.erase(chunk_pos);
	state.in_flight_chunks.insert(chunk_pos);

	SubGridMeshTaskInput input;
	input.ship_uuid = uuid;
	input.chunk_pos = chunk_pos;
	input.lod = lod;
	input.padded_buffer = std::move(padded);
	input.mesher = _mesher;

	_pending_futures.push_back(std::async(std::launch::async, run_mesh_task, std::move(input)));
}

void SubGridManager::_apply_mesh_result(const SubGridMeshTaskResult &result) {
	ShipState *state = _ships.getptr(result.ship_uuid);
	if (state == nullptr || state->node == nullptr) {
		// Ship was destroyed while the task was in flight. discard.
		return;
	}

	// Mark chunk no longer in flight.
	state->in_flight_chunks.erase(result.chunk_pos);

	// Build ArrayMesh from surfaces on the main thread (Godot objects are not
	// safe to create on worker threads).
	Ref<ArrayMesh> mesh;
	if (!result.output.surfaces.empty()) {
		mesh.instantiate();
		for (size_t i = 0; i < result.output.surfaces.size(); i++) {
			if (result.output.surfaces[i].arrays.size() == 0) {
				continue;
			}
			mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, result.output.surfaces[i].arrays);
			uint16_t mat_idx = result.output.surfaces[i].material_index;
			Ref<Material> mat = _mesher->get_material_by_index(mat_idx);
			if (mat.is_valid()) {
				mesh->surface_set_material((int)i, mat);
			}
		}
	}

	// Hand the finished mesh to the node. runs on main thread, safe to touch
	// scene tree objects.
	state->node->apply_chunk_mesh(result.chunk_pos, result.lod, mesh);
}

// ____________________________________________________________________________
// Padded buffer construction (main thread)

std::shared_ptr<VoxelBuffer> SubGridManager::_build_padded_buffer(VoxelSubGrid *node, Vector3i chunk_pos) const {
	const SubGridChunkMap &chunk_map = node->get_chunk_map();
	std::shared_ptr<VoxelBuffer> buf = chunk_map.get_chunk_buffer(chunk_pos);
	if (!buf) {
		return nullptr;
	}

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const int pad = 1;
	const int ps = cs + pad * 2;

	auto padded = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_DEFAULT);
	padded->create(ps, ps, ps);
	padded->fill(0, VoxelBuffer::CHANNEL_TYPE);

	// Center chunk.
	for (int z = 0; z < cs; z++) {
		for (int y = 0; y < cs; y++) {
			for (int x = 0; x < cs; x++) {
				uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
				padded->set_voxel(v, x + pad, y + pad, z + pad, VoxelBuffer::CHANNEL_TYPE);
			}
		}
	}

	// One-voxel border from each of the 6 face neighbors.
	const Vector3i dirs[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };

	for (const Vector3i &dir : dirs) {
		std::shared_ptr<VoxelBuffer> nbuf = chunk_map.get_chunk_buffer(chunk_pos + dir);
		if (!nbuf) {
			continue; // stays air. correct for subgrid outer boundary
		}
		for (int a = 0; a < cs; a++) {
			for (int b = 0; b < cs; b++) {
				int nx, ny, nz, px, py, pz;
				// clang-format off
				if      (dir.x ==  1) { nx = 0;    ny = a;    nz = b;    px = cs+pad; py = a+pad;  pz = b+pad;  }
				else if (dir.x == -1) { nx = cs-1; ny = a;    nz = b;    px = 0;      py = a+pad;  pz = b+pad;  }
				else if (dir.y ==  1) { nx = a;    ny = 0;    nz = b;    px = a+pad;  py = cs+pad; pz = b+pad;  }
				else if (dir.y == -1) { nx = a;    ny = cs-1; nz = b;    px = a+pad;  py = 0;      pz = b+pad;  }
				else if (dir.z ==  1) { nx = a;    ny = b;    nz = 0;    px = a+pad;  py = b+pad;  pz = cs+pad; }
				else                  { nx = a;    ny = b;    nz = cs-1; px = a+pad;  py = b+pad;  pz = 0;      }
				// clang-format on
				uint32_t v = nbuf->get_voxel(nx, ny, nz, VoxelBuffer::CHANNEL_TYPE);
				padded->set_voxel(v, px, py, pz, VoxelBuffer::CHANNEL_TYPE);
			}
		}
	}

	return padded;
}

// ____________________________________________________________________________
// LOD

int SubGridManager::_compute_lod(VoxelSubGrid *node, Vector3i chunk_pos) const {
	Node3D *viewer = node->get_viewer();
	if (viewer == nullptr) {
		return 0;
	}
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	Vector3 viewer_local = node->get_global_transform().affine_inverse().xform(viewer->get_global_position());
	Vector3 chunk_center = Vector3(chunk_pos * cs) + Vector3(cs * 0.5f, cs * 0.5f, cs * 0.5f);
	float dist = viewer_local.distance_to(chunk_center);
	for (int i = 0; i < 4; i++) {
		if (dist < LOD_DISTANCES[i]) {
			return i;
		}
	}
	return 3;
}

int SubGridManager::_total_in_flight() const {
	int total = 0;
	for (const auto &[_, state] : _ships) {
		total += (int)state.in_flight_chunks.size();
	}
	return total;
}

// ____________________________________________________________________________
// Persistence (unchanged logic, kept here for completeness)

void SubGridManager::save_all() {
	Vector<SubGridMetadata> metas;
	Node *parent = get_parent();
	if (parent == nullptr) {
		return;
	}
	for (int i = 0; i < parent->get_child_count(); i++) {
		VoxelSubGrid *sg = Object::cast_to<VoxelSubGrid>(parent->get_child(i));
		if (sg != nullptr && sg->is_root()) {
			sg->flush_dirty_chunks();
			sg->get_metadata_mut().world_position = sg->get_global_position();
			sg->get_metadata_mut().world_rotation = sg->get_global_basis().get_rotation_quaternion();
			_collect_metadata_recursive(sg, metas);
		}
	}
	_save_metadata_index(metas);
}

void SubGridManager::_collect_metadata_recursive(VoxelSubGrid *sg, Vector<SubGridMetadata> &out) {
	sg->flush_dirty_chunks();
	out.push_back(sg->get_metadata());
	for (int i = 0; i < sg->get_child_count(); i++) {
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(sg->get_child(i));
		if (child != nullptr) {
			_collect_metadata_recursive(child, out);
		}
	}
}

void SubGridManager::load_all() {
	Vector<SubGridMetadata> metas = _load_metadata_index();
	if (metas.is_empty()) {
		return;
	}

	HashMap<String, VoxelSubGrid *> uuid_to_node;
	Node *parent = get_parent();

	// Roots first.
	for (const SubGridMetadata &meta : metas) {
		if (!meta.is_root) {
			continue;
		}
		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		parent->add_child(sg);
		// initialize_root_from_disk no longer calls rebuild_all_meshes.
		// SubGridManager will handle it via register_ship_tree below.
		sg->initialize_root_from_disk(meta, _saves_dir, _mesher, _library);

		register_ship_tree(sg);

		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

	// Children.
	for (const SubGridMetadata &meta : metas) {
		if (meta.is_root) {
			continue;
		}
		String parent_uuid = _uuid_to_string(meta.parent_uuid);
		VoxelSubGrid **parent_sg = uuid_to_node.getptr(parent_uuid);
		ERR_CONTINUE_MSG(parent_sg == nullptr, "Parent not found for child subgrid");

		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		(*parent_sg)->add_child(sg);
		sg->initialize_child(meta, SubGridChunkMap(), _saves_dir);
		sg->load_chunks_from_stream();
		// No rebuild_all_meshes here either.
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

	// Register every root (which recurses into children) to kick off meshing.
	for (const SubGridMetadata &meta : metas) {
		if (!meta.is_root) {
			continue;
		}
		VoxelSubGrid **sg = uuid_to_node.getptr(_uuid_to_string(meta.uuid));
		if (sg != nullptr) {
			register_ship_tree(*sg);
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
		write4(bsize);
		const uint8_t *bptr = blob.ptr();
		buf.insert(buf.end(), bptr, bptr + bsize);
	}

	Ref<FileAccess> f = FileAccess::open(abs_path, FileAccess::WRITE);
	ERR_FAIL_COND_MSG(!f.is_valid(), String("Failed to open for write: ") + abs_path);
	f->store_buffer(buf.data(), buf.size());
	f->flush();
}

Vector<SubGridMetadata> SubGridManager::_load_metadata_index() {
	Vector<SubGridMetadata> result;
	String abs_path = ProjectSettings::get_singleton()->globalize_path(_saves_dir + "/ships_index.bin");

	Ref<FileAccess> f = FileAccess::open(abs_path, FileAccess::READ);
	if (!f.is_valid()) {
		return result;
	}

	int count = f->get_32();
	for (int i = 0; i < count; i++) {
		int blob_size = f->get_32();
		PackedByteArray bytes = f->get_buffer(blob_size);
		result.push_back(SubGridMetadata::deserialize(bytes));
	}
	return result;
}

} // namespace zylann::voxel