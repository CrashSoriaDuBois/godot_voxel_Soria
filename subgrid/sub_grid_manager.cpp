#include "sub_grid_manager.h"
#include "core/config/project_settings.h"
#include "core/io/file_access.h"
#include "engine/voxel_engine.h"
#include "scene/3d/physics/animatable_body_3d.h"
#include "../util/godot/classes/physics_server_3d.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "voxel_sub_grid.h"
#include <chrono>
#include <vector>

// RenderingServer include. use whichever path exists in your Godot build.
// Try "servers/rendering_server.h" or the util wrapper if present.
#include "../util/godot/classes/rendering_server.h"
#include "../util/godot/classes/viewport.h"
#include "../util/godot/classes/world_3d.h"


namespace zylann::voxel {

const float SubGridManager::LOD_DISTANCES[4] = { 32.f, 64.f, 128.f, 256.f };

// ____________________________________________________________________________
// Godot hooks

void SubGridManager::_bind_methods() {
	ClassDB::bind_method(
			D_METHOD("initialize", "terrain", "saves_dir", "mesher", "library"), &SubGridManager::initialize
	);
	ClassDB::bind_method(D_METHOD("save_all"), &SubGridManager::save_all);
	ClassDB::bind_method(D_METHOD("load_all"), &SubGridManager::load_all);
	ClassDB::bind_method(D_METHOD("register_ship_tree", "root"), &SubGridManager::register_ship_tree);
	ClassDB::bind_method(D_METHOD("set_block_mass", "voxel_id", "mass"), &SubGridManager::set_block_mass);
	ClassDB::bind_method(D_METHOD("set_default_block_mass", "mass"), &SubGridManager::set_default_block_mass);
}

void SubGridManager::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			set_process(true);
			set_physics_process(true);
			break;

		case NOTIFICATION_PROCESS:
			_process_mesh(get_process_delta_time());
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			_process_physics(get_physics_process_delta_time());
			break;

		case NOTIFICATION_EXIT_TREE:
			for (auto &f : _pending_futures) {
				f.wait();
			}
			_pending_futures.clear();
			for (auto &[uuid, state] : _ships) {
				_free_chunk_renders(state);
				_destroy_body(state);
			}
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

void SubGridManager::set_block_mass(int voxel_id, float mass) {
	_weight_table.weights[(uint32_t)voxel_id] = mass;
}

void SubGridManager::set_default_block_mass(float mass) {
	_weight_table.default_mass = mass;
}

// ____________________________________________________________________________
// Ship lifecycle

void SubGridManager::_register_single(VoxelSubGrid *sg, const String &parent_uuid) {
	ERR_FAIL_COND(sg == nullptr);
	String uuid = _uuid_to_string(sg->get_metadata().uuid);

	ShipState &state = _ships[uuid];
	state.node = sg;
	state.parent_uuid = parent_uuid;
	state.load_state = LOADED;
	state.dirty_chunks.clear();
	state.in_flight_chunks.clear();
	state.current_lod.clear();

	_mark_all_dirty(state);
	sg->set_manager(this);

	// Create physics body
	if (sg->is_root()) {
		if (sg->get_metadata().is_terrain_anchored) {
			_create_child_body(uuid, state); // AnimatableBody3D, kinematic
		} 
		else {
			_create_root_body(uuid, state); // RigidBody3D, simulated
		}
	} 
	else {
		_create_child_body(uuid, state); // AnimatableBody3D, kinematic
	}
}

void SubGridManager::register_ship_tree(VoxelSubGrid *root) {
	ERR_FAIL_COND(root == nullptr);
	_register_single(root, String());
	String root_uuid = _uuid_to_string(root->get_metadata().uuid);
	for (int i = 0; i < root->get_child_count(); i++) {
		VoxelSubGrid *child = Object::cast_to<VoxelSubGrid>(root->get_child(i));
		if (child != nullptr) {
			// Sub-contraptions are registered with their parent's uuid
			_register_single(child, root_uuid);
		}
	}
}

void SubGridManager::unregister_ship(const String &uuid_str) {
	ShipState *state = _ships.getptr(uuid_str);
	if (state != nullptr) {
		_free_chunk_renders(*state);
		_destroy_body(*state);
	}
	_ships.erase(uuid_str);
}

void SubGridManager::mark_chunk_dirty(const String &uuid_str, Vector3i chunk_pos) {
	ShipState *state = _ships.getptr(uuid_str);
	if (state != nullptr && state->load_state == LOADED && !state->in_flight_chunks.has(chunk_pos)) {
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

String SubGridManager::uuid_for_node(VoxelSubGrid *node) const {
	for (const auto &[uuid, state] : _ships) {
		if (state.node == node)
			return uuid;
	}
	return String();
}

void SubGridManager::mark_all_dirty(const String &uuid) {
	ShipState *state = _ships.getptr(uuid);
	if (state != nullptr) {
		_mark_all_dirty(*state);
	}
}

// ____________________________________________________________________________
// Physics body management

void SubGridManager::_create_root_body(const String &uuid, ShipState &state) {
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

    RID body = ps->body_create();
    ps->body_set_mode(body, PhysicsServer3D::BODY_MODE_RIGID);
    ps->body_set_space(body, get_viewport()->get_world_3d()->get_space());
    ps->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, state.node->get_global_transform());

	// Default mass. will be updated when collision chunks arrive
    ps->body_set_param(body, PhysicsServer3D::BODY_PARAM_MASS, 1.f);

    // Point to the VoxelSubGrid node
	ps->body_attach_object_instance_id(body, state.node->get_instance_id());

    state.body_rid = body;
}

void SubGridManager::_create_child_body(const String &uuid, ShipState &state) {
	// AnimatableBody3D: a kinematic body we drive directly each physics frame.
	// Avoids joint constraint solving. we apply rotation math ourselves, which
	// is cheaper and more controllable than a HingeJoint for gameplay purposes.
	AnimatableBody3D *body = memnew(AnimatableBody3D);

	// Disable automatic sync. we set global_transform ourselves in _process_physics.
	body->set_as_top_level(true);

	// Add as child of the VoxelSubGrid node so it follows it in the scene tree.
	// The actual transform is overwritten every frame so the parent transform
	// doesn't matter here, but it keeps the scene tree tidy.
	state.node->add_child(body);
	body->set_owner(state.node->get_owner());

	state.animatable_body = body;
}

void SubGridManager::_destroy_body(ShipState &state) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	// Free all collision shapes
	for (auto &[chunk_pos, col_data] : state.chunk_collision) {
		if (state.body_rid.is_valid()) {
			for (RID shape_rid : col_data.shape_rids) {
				ps->free_rid(shape_rid);
			}
		} else if (state.animatable_body != nullptr) {
			for (RID shape_rid : col_data.shape_rids) {
				ps->free_rid(shape_rid);
			}
		}
	}
	state.chunk_collision.clear();

	if (state.body_rid.is_valid()) {
		ps->free_rid(state.body_rid);
		state.body_rid = RID();
	}

	if (state.animatable_body != nullptr) {
		state.animatable_body->queue_free();
		state.animatable_body = nullptr;
	}
}

void SubGridManager::_apply_collision_result(ShipState &state, Vector3i chunk_pos, const SubGridCollisionOutput &col) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	// Get the body RID to attach shapes to
	RID body_rid;
	if (state.body_rid.is_valid()) {
		body_rid = state.body_rid;
	} else if (state.animatable_body != nullptr) {
		body_rid = state.animatable_body->get_rid();
	} else {
		return;
	}

	// Remove old shapes for this chunk
	_remove_chunk_collision(state, chunk_pos);

	if (col.boxes.is_empty()) {
		return;
	}

	ChunkCollisionData &col_data = state.chunk_collision[chunk_pos];
	col_data.weighted_pos = col.center_of_mass * col.total_mass;
	col_data.mass = col.total_mass;

	// Add one BoxShape3D per greedy-merged box.
	// Shape data for PhysicsServer box = Vector3 of half-extents.
	for (const AABB &box : col.boxes) {
		RID shape = ps->box_shape_create();
		ps->shape_set_data(shape, box.size * 0.5f);

		// Shape transform: centered on the box in local subgrid space
		Transform3D shape_t;
		shape_t.origin = box.position + box.size * 0.5f;

		ps->body_add_shape(body_rid, shape, shape_t);
		col_data.shape_rids.push_back(shape);
	}
}

void SubGridManager::_remove_chunk_collision(ShipState &state, Vector3i chunk_pos) {
	ChunkCollisionData *col_data = state.chunk_collision.getptr(chunk_pos);
	if (col_data == nullptr)
		return;

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	RID body_rid;
	if (state.body_rid.is_valid()) {
		body_rid = state.body_rid;
	} else if (state.animatable_body != nullptr) {
		body_rid = state.animatable_body->get_rid();
	}

	for (RID shape_rid : col_data->shape_rids) {
		int shape_count = ps->body_get_shape_count(body_rid);
		for (int i = 0; i < shape_count; i++) {
			if (ps->body_get_shape(body_rid, i) == shape_rid) {
				ps->body_remove_shape(body_rid, i);
				break;
			}
		}
		ps->free_rid(shape_rid);
	}
	state.chunk_collision.erase(chunk_pos);
}

void SubGridManager::_recalculate_com(const String &uuid, ShipState &state) {
	if (!state.body_rid.is_valid()) {
		return; // only roots have rigid body CoM
	}

	Vector3 weighted_sum;
	float total_mass = 0.f;

	for (const auto &[chunk_pos, col_data] : state.chunk_collision) {
		weighted_sum += col_data.weighted_pos;
		total_mass += col_data.mass;
	}

	if (total_mass <= 0.f) {
		return;
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	ps->body_set_param(state.body_rid, PhysicsServer3D::BODY_PARAM_MASS, total_mass);
	ps->body_set_param(state.body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS, weighted_sum / total_mass);
}

// ____________________________________________________________________________
// Process. mesh, LOD, streaming

void SubGridManager::_process_mesh(double delta) {
	_process_streaming();

	for (auto &[uuid, state] : _ships) {
		if (state.load_state == LOADED) {
			_process_lod_for_ship(uuid, state);
		}
	}
	for (auto &[uuid, state] : _ships) {
		if (state.load_state == LOADED) {
			_submit_pending_tasks(uuid, state);
		}
	}
	_poll_completed_tasks();
}

// ____________________________________________________________________________
// Physics process. rotation update + transform sync

void SubGridManager::_process_physics(double delta) {
	_update_rotations(delta);
	_sync_all_transforms();
}

void SubGridManager::_update_rotations(double delta) {
	for (auto &[uuid, state] : _ships) {
		if (state.load_state != LOADED || state.node == nullptr) {
			continue;
		}
		if (state.node->is_root()) {
			continue; // roots are driven by PhysicsServer, not rotation
		}
		// Advance sub-contraption angle and apply to AnimatableBody3D.
		// advance_rotation() updates _target_angle_rad and returns the new transform.
		if (state.animatable_body == nullptr) {
			continue;
		}

		state.node->advance_rotation(delta);
		Transform3D local_t = state.node->compute_local_transform();

		// World transform = parent's world transform * this contraption's local transform
		Transform3D world_t;
		if (!state.parent_uuid.is_empty()) {
			ShipState *parent_state = _ships.getptr(state.parent_uuid);
			if (parent_state != nullptr && parent_state->node != nullptr) {
				world_t = parent_state->node->get_global_transform() * local_t;
			}
		}

		// Drive AnimatableBody3D. this makes it push other physics objects
		state.animatable_body->set_global_transform(world_t);

		// Keep VoxelSubGrid node in sync so rendering follows
		state.node->set_global_transform(world_t);
	}
}

void SubGridManager::_sync_all_transforms() {
	RenderingServer *rs = RenderingServer::get_singleton();
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	for (auto &[uuid, state] : _ships) {
		if (state.load_state != LOADED || state.node == nullptr) {
			continue;
		}

		Transform3D world_t;

		if (state.node->is_root() && state.body_rid.is_valid()) {
			// Read authoritative transform from physics engine
			world_t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
			// Push back to the Node3D so GDScript can read position/rotation
			state.node->set_global_transform(world_t);
		} else {
			// Sub-contraption: transform was already set in _update_rotations
			world_t = state.node->get_global_transform();
		}

		// Update all mesh instance RIDs for this ship
		for (auto &[key, render] : state.chunk_renders) {
			Vector3i chunk_pos = _key_to_chunk_pos(key);
			int lod = _key_to_lod(key);
			int cs_world = (1 << SubGridChunkMap::CHUNK_SIZE_PO2) << lod;
			Vector3 local_offset = Vector3(chunk_pos * cs_world);
			rs->instance_set_transform(render.instance_rid, world_t * Transform3D(Basis(), local_offset));
		}
	}
}

// ____________________________________________________________________________
// Streaming

void SubGridManager::_process_streaming() {
	Vector3 viewer_pos;
	bool has_viewer = false;
	VoxelEngine::get_singleton().for_each_viewer([&viewer_pos,
												  &has_viewer](ViewerID id, const VoxelEngine::Viewer &viewer) {
		viewer_pos = viewer.world_position;
		has_viewer = true;
	});
	if (!has_viewer) {
		return;
	}

	for (auto &[uuid, state] : _ships) {
		if (state.node == nullptr) {
			continue;
		}
		float dist = state.node->get_global_position().distance_to(viewer_pos);
		if (state.load_state == SLEEPING && dist < _load_distance) {
			_load_ship(uuid, state);
		} else if (state.load_state == LOADED && dist > _unload_distance) {
			_unload_ship(uuid, state);
		}
	}
}

void SubGridManager::_load_ship(const String &uuid, ShipState &state) {
	if (state.node == nullptr) {
		return;
	}
	state.node->load_chunks_from_stream();
	state.load_state = LOADED;
	_mark_all_dirty(state);

	// Recreate physics body
	if (state.node->is_root()) {
		_create_root_body(uuid, state);
	} else {
		_create_child_body(uuid, state);
	}
}

void SubGridManager::_unload_ship(const String &uuid, ShipState &state) {
	if (state.node == nullptr) {
		return;
	}
	state.dirty_chunks.clear();
	state.node->flush_dirty_chunks();
	_free_chunk_renders(state);
	_destroy_body(state);
	state.node->clear_chunk_buffers();
	state.current_lod.clear();
	state.chunk_collision.clear();
	state.load_state = SLEEPING;
}

void SubGridManager::_free_chunk_renders(ShipState &state) {
	RenderingServer *rs = RenderingServer::get_singleton();
	for (auto &[key, render] : state.chunk_renders) {
		rs->free_rid(render.instance_rid);
	}
	state.chunk_renders.clear();
}

// ____________________________________________________________________________
// LOD

void SubGridManager::_process_lod_for_ship(const String &uuid, ShipState &state) {
	if (state.node == nullptr) {
		return;
	}
	const HashSet<Vector3i> &all_chunks = state.node->get_chunk_map().get_all_chunk_positions();
	for (const Vector3i &chunk_pos : all_chunks) {
		int desired_lod = _compute_lod(state.node, chunk_pos);
		int *current = state.current_lod.getptr(chunk_pos);
		if (current == nullptr || *current != desired_lod) {
			for (int old_lod = 0; old_lod < 4; old_lod++) {
				if (old_lod == desired_lod) {
					continue;
				}
				uint64_t old_key = _chunk_mesh_key(chunk_pos, old_lod);
				ChunkRenderData *old = state.chunk_renders.getptr(old_key);
				if (old != nullptr) {
					RenderingServer::get_singleton()->free_rid(old->instance_rid);
					state.chunk_renders.erase(old_key);
				}
			}
			state.current_lod[chunk_pos] = desired_lod;
			if (!state.in_flight_chunks.has(chunk_pos)) {
				state.dirty_chunks.insert(chunk_pos);
			}
		}
	}
}

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

// ____________________________________________________________________________
// Task submission

void SubGridManager::_submit_pending_tasks(const String &uuid, ShipState &state) {
	if (state.dirty_chunks.is_empty()) {
		return;
	}
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

void SubGridManager::_submit_one_task(const String &uuid, ShipState &state, Vector3i chunk_pos, int lod) {
	std::shared_ptr<VoxelBuffer> padded = _build_padded_buffer(state.node, chunk_pos);
	if (!padded) {
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
	input.build_collision = true; // collision only runs at lod==0 inside run_mesh_task
	input.weight_table = _weight_table; // cheap copy

	_pending_futures.push_back(std::async(std::launch::async, run_mesh_task, std::move(input)));
}

void SubGridManager::_poll_completed_tasks() {
	for (auto it = _pending_futures.begin(); it != _pending_futures.end();) {
		if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
			_apply_mesh_result(it->get());
			it = _pending_futures.erase(it);
		} else {
			++it;
		}
	}
}

// ____________________________________________________________________________
// Mesh + collision result application (main thread)

void SubGridManager::_apply_mesh_result(const SubGridMeshTaskResult &result) {
	ShipState *state = _ships.getptr(result.ship_uuid);
	if (state == nullptr || state->node == nullptr || state->load_state != LOADED) {
		return;
	}

	state->in_flight_chunks.erase(result.chunk_pos);

	// --- Mesh ---
	uint64_t key = _chunk_mesh_key(result.chunk_pos, result.lod);
	ChunkRenderData *existing = state->chunk_renders.getptr(key);
	if (existing != nullptr) {
		RenderingServer::get_singleton()->free_rid(existing->instance_rid);
		state->chunk_renders.erase(key);
	}

	if (!result.output.surfaces.empty()) {
		Ref<ArrayMesh> mesh;
		mesh.instantiate();
		bool has_surface = false;
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
			has_surface = true;
		}

		if (has_surface) {
			RenderingServer *rs = RenderingServer::get_singleton();
			RID scenario = get_viewport()->get_world_3d()->get_scenario();
			RID instance = rs->instance_create();
			rs->instance_set_base(instance, mesh->get_rid());
			rs->instance_set_scenario(instance, scenario);

			int cs_world = (1 << SubGridChunkMap::CHUNK_SIZE_PO2) << result.lod;
			Transform3D world_t = state->node->get_global_transform();
			rs->instance_set_transform(instance, world_t * Transform3D(Basis(), Vector3(result.chunk_pos * cs_world)));

			ChunkRenderData render_data;
			render_data.instance_rid = instance;
			render_data.mesh = mesh;
			state->chunk_renders[key] = std::move(render_data);
		}
	}

	// --- Collision (lod 0 only) ---
	if (result.has_collision) {
		_apply_collision_result(*state, result.chunk_pos, result.collision);
		_recalculate_com(result.ship_uuid, *state);
	}
}

// ____________________________________________________________________________
// Padded buffer

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

	for (int z = 0; z < cs; z++) {
		for (int y = 0; y < cs; y++) {
			for (int x = 0; x < cs; x++) {
				uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
				padded->set_voxel(v, x + pad, y + pad, z + pad, VoxelBuffer::CHANNEL_TYPE);
			}
		}
	}

	const Vector3i dirs[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	for (const Vector3i &dir : dirs) {
		std::shared_ptr<VoxelBuffer> nbuf = chunk_map.get_chunk_buffer(chunk_pos + dir);
		if (!nbuf) {
			continue;
		}
		for (int a = 0; a < cs; a++) {
			for (int b = 0; b < cs; b++) {
				int nx, ny, nz, px, py, pz;
				// clang-format off
				if      (dir.x ==  1) { nx=0;    ny=a;    nz=b;    px=cs+pad; py=a+pad;  pz=b+pad;  }
				else if (dir.x == -1) { nx=cs-1; ny=a;    nz=b;    px=0;      py=a+pad;  pz=b+pad;  }
				else if (dir.y ==  1) { nx=a;    ny=0;    nz=b;    px=a+pad;  py=cs+pad; pz=b+pad;  }
				else if (dir.y == -1) { nx=a;    ny=cs-1; nz=b;    px=a+pad;  py=0;      pz=b+pad;  }
				else if (dir.z ==  1) { nx=a;    ny=b;    nz=0;    px=a+pad;  py=b+pad;  pz=cs+pad; }
				else                  { nx=a;    ny=b;    nz=cs-1; px=a+pad;  py=b+pad;  pz=0;      }
				// clang-format on
				uint32_t v = nbuf->get_voxel(nx, ny, nz, VoxelBuffer::CHANNEL_TYPE);
				padded->set_voxel(v, px, py, pz, VoxelBuffer::CHANNEL_TYPE);
			}
		}
	}
	return padded;
}

int SubGridManager::_total_in_flight() const {
	int total = 0;
	for (const auto &[_, state] : _ships) {
		total += (int)state.in_flight_chunks.size();
	}
	return total;
}

// ____________________________________________________________________________
// Persistence

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

	for (const SubGridMetadata &meta : metas) {
		if (!meta.is_root) {
			continue;
		}
		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		parent->add_child(sg);
		sg->initialize_root_from_disk(meta, _saves_dir, _mesher, _library);
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

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
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

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
	ERR_FAIL_COND_MSG(!f.is_valid(), String("Failed to open: ") + abs_path);
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