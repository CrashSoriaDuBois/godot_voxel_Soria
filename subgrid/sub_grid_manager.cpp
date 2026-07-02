#include "sub_grid_manager.h"
#include "../util/godot/classes/physics_server_3d.h"
#include "core/config/project_settings.h"
#include "core/io/file_access.h"
#include "engine/voxel_engine.h"
#include "scene/3d/physics/animatable_body_3d.h"
#include "terrain/variable_lod/voxel_lod_terrain.h"
#include "voxel_sub_grid.h"
#include <chrono>
#include <vector>

#include "../util/godot/classes/rendering_server.h"
#include "../util/godot/classes/viewport.h"
#include "../util/godot/classes/world_3d.h"
#include "lod/sub_grid_lod_streaming.h"

namespace zylann::voxel {

// ____________________________________________________________________________
// Godot hooks

void SubGridManager::_bind_methods() {
	ClassDB::bind_method(
			D_METHOD(
					"initialize",
					"terrain",
					"saves_dir",
					"mesher",
					"library",
					"view_distance",
					"lod_count",
					"lod_distance",
					"secondary_lod_distance"
			),
			&SubGridManager::initialize,
			DEFVAL(512.f),
			DEFVAL(4),
			DEFVAL(48.f),
			DEFVAL(48.f)
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
			if (Engine::get_singleton()->is_editor_hint())
				break;
			set_process(true);
			set_physics_process(true);
			_save_thread_data.running = true;
			_save_thread = std::thread(&SubGridManager::_save_thread_func, this);
			break;

		case NOTIFICATION_PROCESS:
			_process_mesh(get_process_delta_time());
			break;

		case NOTIFICATION_PHYSICS_PROCESS:
			_process_physics(get_physics_process_delta_time());
			break;

		case NOTIFICATION_EXIT_TREE: {
			// Save all ships before shutdown so nothing is lost even if the user didn't call save_all explicitly.
			for (auto &[uuid, state] : _ships) {
				if (state.node == nullptr || state.load_state != LOADED) {
					continue;
				}
				const HashSet<Vector3i> &positions = state.node->get_chunk_map().get_all_chunk_positions();
				for (const Vector3i &pos : positions) {
					state.node->get_chunk_map_mut().mark_chunk_dirty_for_save(pos);
				}
				state.node->flush_dirty_chunks();
			}
			// Now wait for save thread to finish before shutting it down
			{
				std::unique_lock<std::mutex> lock(_save_thread_data.mutex);
				_save_thread_data.running = false;
				_save_thread_data.cv.notify_all();
			}
			if (_save_thread.joinable()) {
				_save_thread.join();
			}
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
}

// ____________________________________________________________________________
// Setup

void SubGridManager::initialize(
		VoxelLodTerrain *terrain,
		const String &saves_dir,
		Ref<VoxelMesherBlocky> mesher,
		Ref<VoxelBlockyLibrary> library,
		float view_distance,
		int lod_count,
		float lod_distance,
		float secondary_lod_distance
) {
	_terrain = terrain;
	_saves_dir = saves_dir;
	_mesher = mesher;
	_library = library;

	_lod_count = CLAMP(lod_count, 1, SUBGRID_MAX_LODS);
	_lod_distance = MAX(lod_distance, 1.f);
	_secondary_lod_distance = MAX(secondary_lod_distance, 0.f);
	_recompute_lod_distances();

	const float min_view_distance = _lod_distances[_lod_count - 1];
	if (view_distance < min_view_distance) {
		WARN_PRINT(
				String("SubGridManager: view_distance ") + rtos(view_distance) +
				" is smaller than the outermost LOD distance " + rtos(min_view_distance) +
				" - clamping up to avoid flushing ship data from RAM while it's still meant to be rendered."
		);
		view_distance = min_view_distance;
	}

	// Preserve the same 50-voxel hysteresis gap the old hardcoded 200.f/250.f pair had, now anchored to the configurable view_distance instead.
	_unload_distance = view_distance;
	_load_distance = MAX(view_distance - 50.f, 0.f);

	// Same margin reasoning as before, just derived now instead of read from a static array.
	_viewer_pairing_distance = _lod_distances[_lod_count - 1] + 32.f;
}

void SubGridManager::_recompute_lod_distances() {
	_lod_distances[0] = _lod_distance;
	for (int lod = 1; lod < _lod_count; lod++) {
		_lod_distances[lod] = _lod_distance + _secondary_lod_distance * static_cast<float>(1 << lod);
	}
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
	state.min_chunk_y = INT32_MAX;
	for (const Vector3i &pos : sg->get_chunk_map().get_all_chunk_positions()) {
		if (pos.y < state.min_chunk_y) {
			state.min_chunk_y = pos.y;
		}
	}

	for (int lod = 0; lod < SUBGRID_MAX_LODS; lod++) {
		ShipLod &ship_lod = state.lods[lod];
		ship_lod.mesh_state.clear();
		ship_lod.pending_update.clear();
		ship_lod.to_activate_visuals.clear();
		ship_lod.to_deactivate_visuals.clear();
		ship_lod.to_unload.clear();
	}
	state.paired_viewers.clear();
	state.pending_loaded_chunks.clear();

	//mesh_state is empty for every LOD until a viewer's box first reaches this ship in _process_lod_for_ship (next frame), at which
	// point view_mesh_box schedules a remesh for every chunk it views automatically - there's nothing to "mark dirty" before any chunk is being viewed yet.
	sg->set_manager(this);

	if (sg->_needs_initial_save) {
		sg->_needs_initial_save = false;
		// Force all chunks dirty so flush saves them even if set_block_buffer was used (which doesn't mark dirty). This
		// is the initial persistence of a newly assembled ship.
		const HashSet<Vector3i> &positions = sg->get_chunk_map().get_all_chunk_positions();
		for (const Vector3i &pos : positions) {
			sg->get_chunk_map_mut().mark_chunk_dirty_for_save(pos);
		}

		print_line(String("_register_single: dirty after mark=") + itos(sg->get_chunk_map().get_dirty_chunks().size()));

		sg->flush_dirty_chunks();
	}

	if (sg->is_root()) {
		if (sg->get_metadata().is_terrain_anchored) {
			_create_child_body(uuid, state);
		} else {
			_create_root_body(uuid, state);
		}
	} else {
		_create_child_body(uuid, state);
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

void SubGridManager::mark_chunk_dirty(const String &uuid_str, Vector3i lod0_chunk_pos) {
	ShipState *state = _ships.getptr(uuid_str);
	if (state == nullptr || state->load_state != LOADED) {
		return;
	}

	// Invalidate collision for this LOD0 chunk
	state->collision_built_chunks.erase(lod0_chunk_pos);

	// Propagate LOD downsampling in the chunk map and collect affected
	// LOD-space positions
	FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> affected;
	state->node->get_chunk_map_mut().update_lods_for_chunk(lod0_chunk_pos, affected);

	//re-trigger meshing for every affected position that is currently being viewed (has a ChunkMeshBlockState already). Positions nobody is
	// viewing right now are skipped - schedule_chunk_remesh() is a no-op for them, and view_mesh_box() will read fresh voxel data automatically once a viewer's box reaches
	// them, so there's nothing lost by not tracking them here.

	// LOD0
	notify_chunk_edited(*state, lod0_chunk_pos, 0);

	// LOD1+: affected[lod] are LOD-space positions
	for (int lod = 1; lod < _lod_count; lod++) {
		for (const Vector3i &lod_pos : affected[lod]) {
			notify_chunk_edited(*state, lod_pos, lod);
		}
	}
}

void SubGridManager::_mark_all_dirty(ShipState &state) {
	if (state.node == nullptr) {
		return;
	}
	// Re-trigger meshing for every chunk currently being viewed, at every LOD. Mirrors the old "mark every chunk dirty regardless of whether it's viewed" intent, adapted: a chunk
	// nobody is viewing doesn't have a ChunkMeshBlockState to mark, and doesn't need one, it'll get fresh voxel data the moment a viewer's box reaches it anyway.
	for (int lod = 0; lod < _lod_count; lod++) {
		ShipLod &ship_lod = state.lods[lod];
		// Collect positions first: schedule_chunk_remesh() doesn't mutate mesh_state's key set
		// (only pending_update / per-block fields), so iterating mesh_state directly while
		// calling it is safe, but a HashMap doesn't expose a "keys" view here - copy them out.
		Vector<Vector3i> positions;
		for (const KeyValue<Vector3i, ChunkMeshBlockState> &kv : ship_lod.mesh_state) {
			positions.push_back(kv.key);
		}
		for (const Vector3i &pos : positions) {
			schedule_chunk_remesh(ship_lod, pos);
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
// Ship lifecycle Threaded

void SubGridManager::_save_thread_func() {
	auto &d = _save_thread_data;

	while (true) {
		std::vector<SubGridManager::SaveRequest> batch;
		{
			std::unique_lock<std::mutex> lock(d.mutex);
			d.cv.wait(lock, [&d] { return !d.queue.empty() || !d.running; });
			if (!d.running && d.queue.empty())
				break;
			batch = std::move(d.queue);
			d.queue.clear();
		}

		for (auto &req : batch) {
			struct Decrement {
				std::atomic<int> &counter;
				bool skip;
				~Decrement() {
					if (!skip)
						--counter;
				}
			};
			Decrement dec{ d.items_in_flight,
						   req.close_stream }; // don't decrement for close_stream (wasn't incremented)

			String uuid = String(req.uuid.c_str());
			if (req.close_stream) {
				auto it = d.streams.find(uuid);
				if (it != d.streams.end()) {
					it->value->set_database_path(""); // closes SQLite
					d.streams.erase(uuid);
				}
				continue;
			}

			// Open stream lazily on save thread if not yet open
			if (!d.streams.has(uuid)) {
				String saves_dir = String(req.saves_dir.c_str());
				// Parse uuid bytes from hex string for SubGridStreamHelper
				// We store the path directly instead
				Ref<VoxelStreamSQLite> stream;
				stream.instantiate();
				// Build path same way SubGridStreamHelper does
				String saves_dir_abs = String(req.saves_dir.c_str()); // already absolute
				String db_path = saves_dir_abs.path_join("ships").path_join(uuid + ".sqlite");
				stream->set_database_path(db_path);
				d.streams[uuid] = stream;
			}

			Ref<VoxelStreamSQLite> &stream = d.streams[uuid];
			if (!stream.is_valid())
				continue;

			VoxelStream::VoxelQueryData q{ *req.buffer, req.chunk_pos, 0, VoxelStream::RESULT_BLOCK_NOT_FOUND };
			stream->save_voxel_block(q);
			_save_thread_data.items_in_flight--;
		}
	}

	// Drain: close all streams cleanly
	for (auto &kv : d.streams) {
		kv.value->set_database_path("");
	}
	d.streams.clear();
}

void SubGridManager::push_save(const SaveRequest &req) {
	std::unique_lock<std::mutex> lock(_save_thread_data.mutex);
	_save_thread_data.items_in_flight++;
	_save_thread_data.queue.push_back(req);
	_save_thread_data.cv.notify_one();
}

void SubGridManager::wait_save_queue() {
	const int max_wait_ms = 5000;
	int waited = 0;
	while (_save_thread_data.items_in_flight > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		waited++;
		if (waited > max_wait_ms) {
			ERR_PRINT("wait_save_queue timed out! items_in_flight=" + itos(_save_thread_data.items_in_flight));
			break;
		}
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

	// Collision layer/mask must be set BEFORE the sleep call below, not after: changing a
	// body's collision_layer/collision_mask forces Godot's physics server to re-evaluate its
	// broadphase pairs, which wakes a sleeping body as a side effect.
	
	_apply_collision_layers(state, state.node->get_metadata().is_terrain_anchored);

	// Start asleep: at this exact moment there are zero collision shapes built for this ship
	// (collision_built_chunks is empty, meshing/collision is async and hasn't had a chance to
	// run yet), so gravity must not act on it until _update_collision_suspension positively
	// confirms real collision exists nearby. Setting this explicitly here, rather than relying
	// on _update_collision_suspension's first call landing before the next physics step,
	// removes any dependency on call ordering within the frame.
	ps->body_set_state(body, PhysicsServer3D::BODY_STATE_SLEEPING, true);
	state.collision_suspended = true;
}

void SubGridManager::_create_child_body(const String &uuid, ShipState &state) {
	// AnimatableBody3D: a kinematic body we drive directly each physics frame.
	// Avoids joint constraint solving. we apply rotation math ourselves, which is cheaper and more controllable than a HingeJoint for gameplay purposes.
	AnimatableBody3D *body = memnew(AnimatableBody3D);

	// Disable automatic sync. we set global_transform ourselves in _process_physics.
	body->set_as_top_level(true);

	// Add as child of the VoxelSubGrid node so it follows it in the scene tree.
	// The actual transform is overwritten every frame so the parent transform doesn't matter here, but it keeps the scene tree tidy.
	state.node->add_child(body);
	body->set_owner(state.node->get_owner());

	state.animatable_body = body;

	// Unlike _create_root_body, this path is shared by both normal sub-contraptions AND terrain-anchored promoted roots
	// (see _register_single's branching), the metadata check here is what actually distinguishes them, not which function got called.
	_apply_collision_layers(state, state.node->get_metadata().is_terrain_anchored);
}

void SubGridManager::_destroy_body(ShipState &state) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	for (auto &[chunk_pos, col_data] : state.chunk_collision) {
		for (RID shape_rid : col_data.shape_rids) {
			ps->free_rid(shape_rid);
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

void SubGridManager::_apply_collision_layers(ShipState &state, bool is_terrain_anchored) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	RID body_rid;
	bool is_root_rigidbody = false;
	if (state.body_rid.is_valid()) {
		body_rid = state.body_rid;
		is_root_rigidbody = true;
	} else if (state.animatable_body != nullptr) {
		body_rid = state.animatable_body->get_rid();
	} else {
		return;
	}

	const uint32_t subgrid_part_bit = 1u << (SUBGRID_PART_LAYER_BIT - 1);
	const uint32_t terrain_anchored_bit = 1u << (TERRAIN_ANCHORED_LAYER_BIT - 1);
	const uint32_t ship_hull_bit = 1u << (SHIP_HULL_LAYER_BIT - 1);

	// A real root rigidbody is never terrain-anchored in practice, but checking state.body_rid directly here rather 
	// than trusting is_terrain_anchored to always be false for roots means this stays correct even if that branching ever changes.
	uint32_t layer;
	if (is_root_rigidbody) {
		layer = ship_hull_bit;
	} else {
		layer = is_terrain_anchored ? terrain_anchored_bit : subgrid_part_bit;
	}
	ps->body_set_collision_layer(body_rid, layer);

	// Hulls (root rigidbodies AND terrain-anchored statics) collide with everything except
	// turrets, that includes each other, terrain, and the player, via the OR-rule (A and B
	// collide if A.layer & B.mask OR B.layer & A.mask is nonzero), without needing to know which bits terrain/player actually use.
	const uint32_t mask = is_root_rigidbody || is_terrain_anchored ? ~uint32_t(0) & ~subgrid_part_bit
																   : ~uint32_t(0) & ~subgrid_part_bit & ~ship_hull_bit;
	ps->body_set_collision_mask(body_rid, mask);
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
			_apply_lod_visibility_changes(state);
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
	_drive_grabbed_ships(delta);
	_update_rotations(delta);
	_sync_all_transforms();
}

void SubGridManager::_update_rotations(double delta) {
	for (auto &[uuid, state] : _ships) {
		if (state.load_state != LOADED || state.node == nullptr) {
			continue;
		}
		if (state.node->is_root() && !state.node->get_metadata().is_terrain_anchored) {
			continue; // only skip simulated roots, not terrain-anchored ones
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
		} else {
			if (!state.node->_is_world_anchored) { // Not yet promoted/loaded correctly, skip
				continue;
			}
			float rpm = state.node->get_angular_speed_rpm();
			if (rpm == 0.0f) {
				state.animatable_body->set_global_transform(state.node->get_global_transform());
				continue;
			}
			Vector3 facing = Vector3(state.node->get_metadata().rotation_axis).normalized();
			Basis rotation_basis = Basis(facing, (real_t)state.node->get_target_angle_rad());

			// pivot_in_child_local: same as compute_local_transform
			Vector3 pivot_in_child_local = Vector3(0.5f, 0.5f, 0.5f) - facing * 0.5f;

			// Rotate child around the world pivot point
			world_t.basis = rotation_basis;
			world_t.origin = state.node->_promoted_pivot_world + facing * 0.5f // bearing face center offset
					- rotation_basis.xform(pivot_in_child_local);
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
		if (!state.node->is_inside_tree())
			continue;
		if (state.load_state != LOADED || state.node == nullptr)
			continue;

		Transform3D world_t;

		if (state.node->is_root() && state.body_rid.is_valid() && !state.grabbed) {
			// Only read from physics server if NOT grabbed
			// (grabbed bodies are positioned by _drive_grabbed_ships directly)
			world_t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
			state.node->set_global_transform(world_t);
		} else {
			// Use whatever transform the node already has
			world_t = state.node->get_global_transform();
		}

		for (auto &[key, render] : state.chunk_renders) {
			Vector3i chunk_pos = _key_to_chunk_pos(key);
			int lod = _key_to_lod(key);
			Vector3 local_offset = _lod_chunk_local_offset(chunk_pos, lod);
			rs->instance_set_transform(render.instance_rid, world_t * Transform3D(Basis(), local_offset));
		}
	}
}

void SubGridManager::grab_subgrid(VoxelSubGrid *sg, Vector3 grab_point_local, bool rotate, float strength) {
	String uuid = uuid_for_node(sg);
	ShipState *state = _ships.getptr(uuid);
	ERR_FAIL_COND_MSG(state == nullptr, "VoxelSubGrid not registered in manager.");
	ERR_FAIL_COND_MSG(!state->body_rid.is_valid(), "Cannot grab a non-rigid subgrid.");

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	// Zero velocity so it doesn't carry momentum into the grab
	ps->body_set_mode(state->body_rid, PhysicsServer3D::BODY_MODE_RIGID);
	ps->body_set_state(state->body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, Vector3());
	ps->body_set_state(state->body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, Vector3());

	state->grabbed = true;
	state->grab_rotate = rotate;
	state->grab_point_local = grab_point_local;
	state->grab_target = sg->get_global_transform();

	state->grab_strength = strength;
}

void SubGridManager::release_subgrid(VoxelSubGrid *sg) {
	String uuid = uuid_for_node(sg);
	ShipState *state = _ships.getptr(uuid);
	ERR_FAIL_COND_MSG(state == nullptr, "VoxelSubGrid not registered in manager.");

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if (state->grabbed && state->body_rid.is_valid()) {
		// Clear velocity so the last frame doesn't launch it
		ps->body_set_state(state->body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, Vector3());
		ps->body_set_state(state->body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, Vector3());

		ps->body_set_param(state->body_rid, PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE, 1.0f);
	}
	state->grabbed = false;
	if (state->collision_suspended && state->body_rid.is_valid()) {
		ps->body_set_mode(state->body_rid, PhysicsServer3D::BODY_MODE_STATIC);
	}
}

void SubGridManager::set_grab_target(VoxelSubGrid *sg, Transform3D target) {
	String uuid = uuid_for_node(sg);
	ShipState *state = _ships.getptr(uuid);
	ERR_FAIL_COND_MSG(state == nullptr, "VoxelSubGrid not registered in manager.");
	state->grab_target = target;
}

void SubGridManager::apply_impulse(VoxelSubGrid *sg, Vector3 impulse, Vector3 world_point) {
	String uuid = uuid_for_node(sg);
	ShipState *state = _ships.getptr(uuid);
	ERR_FAIL_COND_MSG(state == nullptr, "VoxelSubGrid not registered in manager.");
	ERR_FAIL_COND_MSG(!state->body_rid.is_valid(), "Cannot apply impulse to a non-rigid subgrid.");

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Transform3D t = ps->body_get_state(state->body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);

	Variant com_variant = ps->body_get_param(state->body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS);
	Vector3 com_local = com_variant;

	// Pure linear push
	ps->body_apply_central_impulse(state->body_rid, impulse);

	// Controlled tumble from hit offset, scale down to avoid launching
	Vector3 local_point = t.affine_inverse().xform(world_point) - com_local;
	Vector3 torque = local_point.cross(impulse) * 0.1f;
	ps->body_apply_torque_impulse(state->body_rid, torque);
}

void SubGridManager::apply_central_impulse(VoxelSubGrid *sg, Vector3 impulse) {
	String uuid = uuid_for_node(sg);
	ShipState *state = _ships.getptr(uuid);
	ERR_FAIL_COND_MSG(state == nullptr, "VoxelSubGrid not registered in manager.");
	ERR_FAIL_COND_MSG(!state->body_rid.is_valid(), "Cannot apply impulse to a non-rigid subgrid.");

	PhysicsServer3D::get_singleton()->body_apply_central_impulse(state->body_rid, impulse);
}

void SubGridManager::_drive_grabbed_ships(double delta) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	for (auto &[uuid, state] : _ships) {
		if (!state.grabbed || !state.body_rid.is_valid()) {
			continue;
		}

		ps->body_set_param(state.body_rid, PhysicsServer3D::BODY_PARAM_GRAVITY_SCALE, 0.0f);

		Transform3D current = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);

		// Linear drive (existing)
		Vector3 grab_local_world = current.basis.xform(state.grab_point_local);
		Vector3 desired_origin = state.grab_target.origin - grab_local_world;
		Vector3 linear_error = desired_origin - current.origin;

		// Mass aware speed cap :base speed * strength / mass, so heavier objects need more strength to move at the same
		// speed
		float mass = ps->body_get_param(state.body_rid, PhysicsServer3D::BODY_PARAM_MASS);

		float max_linear_speed = (15.f * state.grab_strength) / mass;
		max_linear_speed = CLAMP(max_linear_speed, 0.5f, 35.f);

		Vector3 new_linear_vel = linear_error / (float)delta;
		if (new_linear_vel.length() > max_linear_speed) {
			new_linear_vel = new_linear_vel.normalized() * max_linear_speed;
		}

		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, new_linear_vel);

		// Angular drive (new)
		if (state.grab_rotate) {
			// Rotation error: how much do we need to rotate current to reach target
			Basis current_basis = current.basis.orthonormalized();
			Basis target_basis = state.grab_target.basis.orthonormalized();

			// delta rotation = target * current_inverse
			Basis rotation_error = target_basis * current_basis.inverse();
			// Convert to axis/angle
			Vector3 axis;
			real_t angle;
			rotation_error.get_axis_angle(axis, angle);

			// Wrap angle to [-PI, PI]
			if (angle > 3.14159265f)
				angle -= 6.28318530f;

			float max_angular_speed = (2.f * state.grab_strength) / mass;
			max_angular_speed = CLAMP(max_angular_speed, 0.1f, 20.f);

			Vector3 new_angular_vel = axis * (angle / (float)delta);
			if (new_angular_vel.length() > max_angular_speed) {
				new_angular_vel = new_angular_vel.normalized() * max_angular_speed;
			}
			ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, new_angular_vel);
		} else {
			ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, Vector3());
		}

		state.node->set_global_transform(current);
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
		if (state.load_state == SLEEPING) {
			print_line(String("streaming: SLEEPING ship dist=") + rtos(dist) + " load_dist=" + rtos(_load_distance));
		}
		if (state.load_state == SLEEPING && dist < _load_distance) {
			_load_ship(uuid, state);
		} else if (state.load_state == LOADED && dist > _unload_distance) {
			_unload_ship(uuid, state);
		}
	}
}

void SubGridManager::_load_ship(const String &uuid, ShipState &state) {
	print_line(String("_load_ship called for uuid=") + uuid);
	if (state.node == nullptr) {
		return;
	}
	state.node->load_chunks_from_stream();

	// Only proceed if chunks actually loaded
	if (state.node->get_chunk_map().get_all_chunk_positions().is_empty()) {
		print_line(String("_load_ship: no chunks loaded, staying SLEEPING"));
		// Don't change load_state, don't create body
		// The ship will retry next time it comes in range
		return;
	}

	state.load_state = LOADED;
	_mark_all_dirty(state);

	if (state.node->is_root() && !state.node->get_metadata().is_terrain_anchored) {
		_create_root_body(uuid, state);
	} else {
		_create_child_body(uuid, state);
	}
}

void SubGridManager::_unload_ship(const String &uuid, ShipState &state) {
	for (int lod = 0; lod < SUBGRID_MAX_LODS; lod++) {
		ShipLod &ship_lod = state.lods[lod];
		ship_lod.mesh_state.clear();
		ship_lod.pending_update.clear();
		ship_lod.to_activate_visuals.clear();
		ship_lod.to_deactivate_visuals.clear();
		ship_lod.to_unload.clear();
	}
	state.paired_viewers.clear();
	state.pending_loaded_chunks.clear();

	// Force all chunks dirty for save
	const HashSet<Vector3i> &positions = state.node->get_chunk_map().get_all_chunk_positions();
	for (const Vector3i &pos : positions) {
		state.node->get_chunk_map_mut().mark_chunk_dirty_for_save(pos);
	}
	state.node->flush_dirty_chunks();

	// Wait for save thread to finish writing
	wait_save_queue();

	// Close the SQLite stream so WAL is checkpointed before reload reads it, without this, a new connection may not see
	// data written by the save thread.
	state.node->_close_stream();

	// Wait for close request to be processed (close doesn't increment items_in_flight)
	// Poll until the save thread's queue is empty
	const int max_wait_ms = 2000;
	int waited = 0;
	while (waited < max_wait_ms) {
		{
			std::unique_lock<std::mutex> lock(_save_thread_data.mutex);
			if (_save_thread_data.queue.empty()) {
				break;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		waited++;
	}
	// Wait for the close request to be processed too
	wait_save_queue();

	_free_chunk_renders(state);
	_destroy_body(state);
	state.node->clear_chunk_buffers();
	state.chunk_collision.clear();
	state.collision_built_chunks.clear();
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

	process_ship_lod_streaming(
			state,
			state.node->get_chunk_map(),
			Span<const float>(_lod_distances.data(), _lod_count),
			_viewer_pairing_distance
	);
}

void SubGridManager::_apply_lod_visibility_changes(ShipState &state) {
	RenderingServer *rs = RenderingServer::get_singleton();

	for (int lod = 0; lod < _lod_count; lod++) {
		ShipLod &ship_lod = state.lods[lod];
		const bool is_coarse = lod > _collision_safe_lod_max;

		for (const Vector3i &lod_pos : ship_lod.to_activate_visuals) {
			ChunkRenderData *render = state.chunk_renders.getptr(_chunk_mesh_key(lod_pos, lod));
			if (render != nullptr) {
				rs->instance_set_visible(render->instance_rid, true);
			}
			// Else: mesh task for this chunk hasn't completed yet. _apply_mesh_result will
			// read the (already true) visual_active flag and create the instance visible.
			if (is_coarse) {
				state.coarse_lod_active_count += 1;
			}
		}
		ship_lod.to_activate_visuals.clear();

		for (const Vector3i &lod_pos : ship_lod.to_deactivate_visuals) {
			ChunkRenderData *render = state.chunk_renders.getptr(_chunk_mesh_key(lod_pos, lod));
			if (render != nullptr) {
				rs->instance_set_visible(render->instance_rid, false);
			}
			if (is_coarse) {
				state.coarse_lod_active_count -= 1;
			}
		}
		ship_lod.to_deactivate_visuals.clear();

		for (const Vector3i &lod_pos : ship_lod.to_unload) {
			uint64_t key = _chunk_mesh_key(lod_pos, lod);
			ChunkRenderData *render = state.chunk_renders.getptr(key);
			if (render != nullptr) {
				rs->free_rid(render->instance_rid);
				state.chunk_renders.erase(key);
			}
			// Note: collision is intentionally untouched here, matching the old behavior, collision lifecycle is independent of visual LOD activity (see
			// collision_built_chunks: only mark_chunk_dirty's edit path and whole-ship teardown ever invalidate it).
		}
		ship_lod.to_unload.clear();
	}

	ERR_FAIL_COND_MSG(
			state.coarse_lod_active_count < 0,
			"coarse_lod_active_count went negative - activate/deactivate events are unbalanced somewhere"
	);

	_update_collision_suspension(state);
}

void SubGridManager::_update_collision_suspension(ShipState &state) {
	if (!state.body_rid.is_valid()) {
		// Sub-contraption (AnimatableBody3D, kinematic) or not yet given a body, never gravity-simulated in the first place, nothing to suspend.
		return;
	}

	// coarse_lod_active_count == 0 is ambiguous on its own: it's true both when the ship has
	// resolved down to fine LOD (safe) AND when nothing has loaded at all yet, e.g. right
	// after _load_ship/_create_root_body, before any async mesh/collision task has had time
	// to complete (NOT safe, terrain underneath may have no confirmed collision yet either).
	// Require actual built collision as positive evidence before ever waking, on top of the
	// coarse-chunk check.
	const bool should_be_suspended = state.coarse_lod_active_count > 0 || state.collision_built_chunks.size() == 0;
	if (should_be_suspended == state.collision_suspended) {
		return;
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	ps->body_set_mode(
			state.body_rid, should_be_suspended ? PhysicsServer3D::BODY_MODE_STATIC : PhysicsServer3D::BODY_MODE_RIGID
	);
	if (!should_be_suspended) {
		// Coming out of STATIC: make sure no stale velocity survives the mode switch.
		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, Vector3());
		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, Vector3());
	}
	state.collision_suspended = should_be_suspended;

	// Waking up: nothing else to do, the body resumes integrating next physics step.
	// Going to sleep: if it was grabbed, _drive_grabbed_ships will set
	// BODY_STATE_LINEAR_VELOCITY on it next physics frame regardless, which wakes a sleeping
	// body as a side effect in Godot, grabbing always wins, no special-casing needed.
}

// ____________________________________________________________________________
// Task submission

void SubGridManager::_submit_pending_tasks(const String &uuid, ShipState &state) {
	// MIRROR of send_mesh_requests()'s state transition (MESH_UPDATE_NOT_SENT ->
	// MESH_UPDATE_SENT, update_list_index reset to -1), throttled by MAX_CONCURRENT_TASKS
	// since this module fires tasks via std::async instead of upstream's managed worker pool.
	for (int lod = 0; lod < _lod_count; lod++) {
		ShipLod &ship_lod = state.lods[lod];
		if (ship_lod.pending_update.is_empty()) {
			continue;
		}

		// Submit from the front; anything left over (because the budget ran out) stays
		// queued for next frame. Rebuilt in one pass rather than erased-as-we-go, since
		// erasing from the middle of a Vector would require shifting every later
		// update_list_index anyway.
		Vector<Vector3i> still_pending;
		bool budget_exhausted = false;

		for (const Vector3i &lod_pos : ship_lod.pending_update) {
			if (budget_exhausted || _total_in_flight() >= MAX_CONCURRENT_TASKS) {
				budget_exhausted = true;
				still_pending.push_back(lod_pos);
				continue;
			}

			ChunkMeshBlockState *block = ship_lod.mesh_state.getptr(lod_pos);
			if (block == nullptr) {
				// Was unviewed since being queued; nothing to submit.
				continue;
			}
			ERR_CONTINUE_MSG(
					block->state != MESH_UPDATE_NOT_SENT, "Chunk in pending_update was not MESH_UPDATE_NOT_SENT"
			);

			_submit_one_task(uuid, state, lod_pos, lod);

			block->state = MESH_UPDATE_SENT;
			block->update_list_index = -1;
		}

		// Repair update_list_index for whatever is left, then swap in.
		for (int i = 0; i < still_pending.size(); ++i) {
			ChunkMeshBlockState *block = ship_lod.mesh_state.getptr(still_pending[i]);
			if (block != nullptr) {
				block->update_list_index = i;
			}
		}
		ship_lod.pending_update = std::move(still_pending);
	}
}

void SubGridManager::_submit_one_task(const String &uuid, ShipState &state, Vector3i lod_pos, int lod) {
	std::shared_ptr<VoxelBuffer> padded = _build_padded_buffer(state.node, lod_pos, lod);

	if (!padded) {
		// Caller (_submit_pending_tasks) already set state/update_list_index for this chunk;
		// it'll simply have no task in flight and stay MESH_UPDATE_SENT with nothing to
		// deliver. Acceptable: the next box re-entry or edit will re-schedule it.
		return;
	}

	SubGridMeshTaskInput input;
	input.ship_uuid = uuid;
	input.chunk_pos = lod_pos; // LOD-space
	input.lod = lod;
	input.padded_buffer = std::move(padded);
	input.mesher = _mesher;
	// Collision only at LOD0, only if not yet built
	// chunk_pos at lod0 == lod_pos when lod==0
	input.build_collision = (lod == 0) && !state.collision_built_chunks.has(lod_pos);
	input.weight_table = _weight_table;

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

	const int lod = result.lod;
	const Vector3i lod_pos = result.chunk_pos; // LOD-space
	ShipLod &ship_lod = state->lods[lod];

	// MIRROR of the staleness check in apply_mesh_update(): the chunk may have been unviewed
	// (refcount hit zero, erased from mesh_state) while this task was in flight. Discard.
	ChunkMeshBlockState *block = ship_lod.mesh_state.getptr(lod_pos);
	if (block == nullptr) {
		return;
	}
	block->state = MESH_UP_TO_DATE;

	uint64_t key = _chunk_mesh_key(lod_pos, lod);

	// Remove existing render for this key (re-mesh of an already-loaded chunk, e.g. after an edit, voxels changed so the old geometry is stale).
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

			// LOD-space world offset: chunk_pos * (chunk_size << lod)
			Vector3 local_offset = _lod_chunk_local_offset(lod_pos, lod);
			Transform3D world_t = state->node->get_global_transform();
			rs->instance_set_transform(instance, world_t * Transform3D(Basis(), local_offset));

			// Read the CURRENT visual_active flag rather than assuming hidden: most of the
			// time this is a brand-new chunk and visual_active is still false (nothing could
			// have activated it before its mesh existed), but unview_mesh_box's "show parent
			// immediately when children are removed" branch can mark a chunk active before
			// its own mesh task has completed, in which case this must come up visible.
			rs->instance_set_visible(instance, block->visual_active);

			ChunkRenderData render_data;
			render_data.instance_rid = instance;
			render_data.mesh = mesh;
			state->chunk_renders[key] = std::move(render_data);
		}
	}

	// marks visual_loaded and notifyies the streaming system (ClipboxStreamingState::loaded_mesh_blocks there, state->pending_loaded_chunks here)
	// so the subdivision rule (update_mesh_block_load) can react next frame.
	if (!block->visual_loaded) {
		block->visual_loaded = true;
		state->pending_loaded_chunks.push_back(LoadedChunkEvent{ lod_pos, (uint8_t)lod });
	}

	// Collision: LOD0 only, once per chunk lifetime
	if (result.has_collision && !state->collision_built_chunks.has(lod_pos)) {
		_apply_collision_result(*state, lod_pos, result.collision);
		_recalculate_com(result.ship_uuid, *state);
		state->collision_built_chunks.insert(lod_pos);
	}
}

// ____________________________________________________________________________
// Padded buffer

std::shared_ptr<VoxelBuffer> SubGridManager::_build_padded_buffer(VoxelSubGrid *node, Vector3i lod_pos, int lod) const {
	const SubGridChunkMap &chunk_map = node->get_chunk_map();
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const int pad = 1;
	const int ps = cs + pad * 2;

	// Get the center chunk from the appropriate LOD map
	std::shared_ptr<VoxelBuffer> buf = chunk_map.get_lod_chunk_buffer(lod_pos, lod);
	if (!buf) {
		return nullptr;
	}

	auto padded = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_DEFAULT);
	padded->create(ps, ps, ps);
	padded->fill(0, VoxelBuffer::CHANNEL_TYPE);

	// Copy center chunk
	for (int z = 0; z < cs; z++) {
		for (int y = 0; y < cs; y++) {
			for (int x = 0; x < cs; x++) {
				uint32_t v = buf->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
				padded->set_voxel(v, x + pad, y + pad, z + pad, VoxelBuffer::CHANNEL_TYPE);
			}
		}
	}

	// Pad from LOD-space neighbors (same lod level)
	const Vector3i dirs[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	for (const Vector3i &dir : dirs) {
		std::shared_ptr<VoxelBuffer> nbuf = chunk_map.get_lod_chunk_buffer(lod_pos + dir, lod);
		if (!nbuf) {
			continue;
		}
		for (int a = 0; a < cs; a++) {
			for (int b = 0; b < cs; b++) {
				int nx, ny, nz, px, py, pz;
				// clang-format off
                if      (dir.x ==  1) { nx=0;    ny=a;    nz=b; px=cs+pad; py=a+pad;  pz=b+pad;  }
                else if (dir.x == -1) { nx=cs-1; ny=a;    nz=b; px=0;      py=a+pad;  pz=b+pad;  }
                else if (dir.y ==  1) { nx=a;    ny=0;    nz=b; px=a+pad;  py=cs+pad; pz=b+pad;  }
                else if (dir.y == -1) { nx=a;    ny=cs-1; nz=b; px=a+pad;  py=0;      pz=b+pad;  }
                else if (dir.z ==  1) { nx=a;    ny=b;    nz=0; px=a+pad;  py=b+pad;  pz=cs+pad; }
                else                  { nx=a;    ny=b; nz=cs-1; px=a+pad;  py=b+pad;  pz=0;      }
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
		for (int lod = 0; lod < _lod_count; lod++) {
			for (const KeyValue<Vector3i, ChunkMeshBlockState> &kv : state.lods[lod].mesh_state) {
				if (kv.value.state == MESH_UPDATE_SENT) {
					total += 1;
				}
			}
		}
	}
	return total;
}

// ____________________________________________________________________________
// Persistence

void SubGridManager::save_all() {
	Vector<SubGridMetadata> metas;

	for (auto &[uuid, state] : _ships) {
		if (state.node == nullptr)
			continue;

		print_line(
				String("save_all: saving uuid=") + uuid + " pos=" + String(state.node->get_metadata().world_position) +
				" rot=(" + rtos(state.node->get_metadata().world_rotation.x) + "," +
				rtos(state.node->get_metadata().world_rotation.y) + "," +
				rtos(state.node->get_metadata().world_rotation.z) + "," +
				rtos(state.node->get_metadata().world_rotation.w) + ")" +
				" pivot=" + String(state.node->_promoted_pivot_world) +
				" is_terrain_anchored=" + (state.node->get_metadata().is_terrain_anchored ? "true" : "false") +
				" is_root=" + (state.node->get_metadata().is_root ? "true" : "false")
		);

		if (state.body_rid.is_valid()) {
			Transform3D t = PhysicsServer3D::get_singleton()->body_get_state(
					state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM
			);
			state.node->get_metadata_mut().world_position = t.origin;
			state.node->get_metadata_mut().world_rotation = t.basis.get_rotation_quaternion();
		} else {
			state.node->get_metadata_mut().world_position = state.node->get_global_position();
			state.node->get_metadata_mut().world_rotation = state.node->get_global_basis().get_rotation_quaternion();
		}
		metas.push_back(state.node->get_metadata());
	}
	// Wait for chunk saves to complete before writing index
	// (save thread is still running at this point)
	wait_save_queue();
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

Vector3 SubGridManager::_get_viewer_world_pos(VoxelSubGrid *node) const {
	Vector3 viewer_world;
	bool has_viewer = false;

	VoxelEngine::get_singleton().for_each_viewer([&viewer_world, &has_viewer](ViewerID, const VoxelEngine::Viewer &v) {
		if (!has_viewer) {
			viewer_world = v.world_position;
			has_viewer = true;
		}
	});

	if (!has_viewer && node != nullptr) {
		Node3D *viewer = node->get_viewer();
		if (viewer != nullptr) {
			viewer_world = viewer->get_global_position();
		}
	}
	return viewer_world;
}

void SubGridManager::load_all() {
	Vector<SubGridMetadata> metas = _load_metadata_index();
	print_line(String("load_all: found ") + itos(metas.size()) + " entries in index");
	if (metas.is_empty())
		return;

	HashMap<String, VoxelSubGrid *> uuid_to_node;
	Node *parent = get_parent();

	// Pass 1: roots (both rigid and terrain-anchored)
	for (const SubGridMetadata &meta : metas) {
		if (!meta.is_root)
			continue;

		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		parent->add_child(sg);
		sg->initialize_root_from_disk(meta, _saves_dir, _mesher, _library);
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

	// Pass 2: children (have a valid parent_uuid)
	for (const SubGridMetadata &meta : metas) {
		if (meta.is_root)
			continue;

		String parent_uuid = _uuid_to_string(meta.parent_uuid);
		VoxelSubGrid **parent_sg = uuid_to_node.getptr(parent_uuid);
		ERR_CONTINUE_MSG(
				parent_sg == nullptr,
				"Parent not found for child subgrid — was it saved as is_root=false with no parent?"
		);

		VoxelSubGrid *sg = memnew(VoxelSubGrid);
		(*parent_sg)->add_child(sg);
		sg->initialize_child(meta, SubGridChunkMap(), _saves_dir, false);
		sg->load_chunks_from_stream();
		uuid_to_node[_uuid_to_string(meta.uuid)] = sg;
	}

	// Pass 3: register all roots (register_ship_tree walks children itself)
	for (const SubGridMetadata &meta : metas) {
		if (!meta.is_root)
			continue;
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