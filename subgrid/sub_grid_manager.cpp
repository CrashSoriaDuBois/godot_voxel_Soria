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
#include <cmath>

#include "../util/godot/classes/rendering_server.h"
#include "../util/godot/classes/viewport.h"
#include "../util/godot/classes/world_3d.h"
#include "lod/sub_grid_lod_streaming.h"
#include "../util/godot/classes/time.h"

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
			if (_terrain != nullptr) {
				_terrain->remove_mesh_block_lod_listener(this);
			}
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

			if (_chunk_probe_shape.is_valid()) {
				PhysicsServer3D::get_singleton()->free_rid(_chunk_probe_shape);
				_chunk_probe_shape = RID();
			}

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

	// Mirror the terrain's actual configured channel depths, rather than guessing - this is
	// the single source of truth so a subgrid's color/data5 always agrees with whatever
	// format the terrain (and its stream/generator) actually use.
	Ref<godot::VoxelFormat> terrain_format = _terrain->get_format();
	if (terrain_format.is_valid()) {
		_voxel_format = terrain_format->get_internal();
	} else {
		_voxel_format = VoxelFormat(); // class default (see voxel_format.cpp) as a fallback
	}

	_physics_space = get_viewport()->get_world_3d()->get_space();

	_lod_count = CLAMP(lod_count, 1, SUBGRID_MAX_LODS);
	_lod_distance = MAX(lod_distance, 1.f);
	_secondary_lod_distance = MAX(secondary_lod_distance, 0.f);
	_recompute_lod_distances();

	_viewer_pairing_distance = _lod_distances[_lod_count - 1] + 32.f;
	_terrain->add_mesh_block_lod_listener(this);

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

	// Must happen before anything reads/creates chunk buffers for this ship - see
	// SubGridChunkMap::set_format's comment.
	sg->get_chunk_map_mut().set_format(_voxel_format);

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
	state.coarse_lod_active_count = 0;

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
	_remove_ship_from_cell_index(uuid_str);
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

	ShipLod &lod0 = state->lods[0];

	// Inner box (mesh padding = 1): full remesh, own light_dirty.
	{
		ChunkMeshBlockState *block = lod0.mesh_state.getptr(lod0_chunk_pos);
		if (block != nullptr) {
			block->light_dirty = true;
			block->requires_geometry = true;
		}
	}
	notify_chunk_edited(*state, lod0_chunk_pos, 0);

	// LIGHT_PADDING (15) < chunk size (16): radius-1 chunk neighborhood always covers it.
	// Deliberately LOD0-only (per the earlier scope decision) - subgrid doesn't chase
	// terrain's full per-LOD flood-only neighbor propagation.
	for (int dz = -1; dz <= 1; dz++) {
		for (int dy = -1; dy <= 1; dy++) {
			for (int dx = -1; dx <= 1; dx++) {
				if (dx == 0 && dy == 0 && dz == 0)
					continue;
				const Vector3i npos = lod0_chunk_pos + Vector3i(dx, dy, dz);
				ChunkMeshBlockState *nblock = lod0.mesh_state.getptr(npos);
				if (nblock == nullptr)
					continue;
				nblock->light_dirty = true;
				// Only downgrade to flood-only if nothing else already needs (or is currently mid-flight
				// for) a real geometry rebuild on this chunk. Blindly overwriting requires_geometry=false
				// here was clobbering genuinely-pending full rebuilds scheduled moments earlier but not yet
				// submitted/completed - exactly what caused rapidly-edited new chunks to lose their mesh.
				if (nblock->update_list_index == -1 && nblock->state != MESH_UPDATE_SENT) {
					nblock->requires_geometry = false;
				}
				schedule_chunk_remesh(lod0, npos);
			}
		}
	}

	for (int lod = 1; lod < _lod_count; lod++) {
		for (const Vector3i &lod_pos : affected[lod]) {
			ChunkMeshBlockState *ablock = state->lods[lod].mesh_state.getptr(lod_pos);
			if (ablock != nullptr) {
				// Force a real geometry rebuild for this edit - never let a stale
				// requires_geometry=false from a PREVIOUS build cause this fresh edit's
				// mandatory geometry update to be misrouted into a flood-only task.
				ablock->requires_geometry = true;
				// light_dirty deliberately left untouched here: LOD1+ never floods on its
				// own regardless, so this flag is irrelevant to it either way. The rebuild's
				// EXTRACT_FROM_DATA5 branch will read whatever CHANNEL_DATA5 currently holds -
				// which may still be stale relative to LOD0's not-yet-finished flood. That's
				// fine: this is a geometry-correctness rebuild, and _apply_mesh_result's
				// forced re-extract (once the flood truly completes) is what guarantees the
				// LIGHT ends up correct afterward, using the pending_light_retry mechanism to
				// avoid racing this exact task if it's still in flight.
			}
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

	//Static mode makes it immovable regardless of what touches it, until _apply_combined_suspension explicitly lifts it.
	ps->body_set_mode(body, PhysicsServer3D::BODY_MODE_STATIC);
	state.collision_suspended = true;
	state.ground_confirmed = false;
	state.awaiting_ground_confirmation = false;
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
	_drive_homing_disassembles(delta);
	_update_rotations(delta);
	_sync_all_transforms();
	_process_ground_confirmations();
}

void SubGridManager::_update_rotations(double delta) {
	static thread_local Vector<VoxelSubGrid *> pending_disassembles;
	pending_disassembles.clear();

	for (auto &[uuid, state] : _ships) {
		if (state.load_state != LOADED || state.node == nullptr) {
			continue;
		}
		if (state.node->is_root() && !state.node->get_metadata().is_terrain_anchored) {
			continue;
		}
		if (state.animatable_body == nullptr) {
			continue;
		}

		state.node->advance_rotation(delta);

		if (state.node->check_auto_align_disassemble_ready()) {
			pending_disassembles.push_back(state.node);
		}

		Transform3D local_t = state.node->compute_local_transform();

		Transform3D world_t;
		if (!state.parent_uuid.is_empty()) {
			ShipState *parent_state = _ships.getptr(state.parent_uuid);
			if (parent_state != nullptr && parent_state->node != nullptr) {
				world_t = parent_state->node->get_global_transform() * local_t;
			}
		} else {
			if (!state.node->_is_world_anchored) {
				continue;
			}
			const float rpm = state.node->get_angular_speed_rpm();
			const bool auto_aligning = state.node->is_auto_align_active();
			if (rpm == 0.0f && !auto_aligning) {
				state.animatable_body->set_global_transform(state.node->get_global_transform());
				continue;
			}

			Vector3 facing = Vector3(state.node->get_metadata().rotation_axis).normalized();
			Basis rotation_basis =
					Basis(Vector3(state.node->get_spin_axis()).normalized(),
						  (real_t)state.node->get_target_angle_rad());

			Vector3 pivot_in_child_local = Vector3(0.5f, 0.5f, 0.5f) - facing * 0.5f;

			world_t.basis = rotation_basis;
			world_t.origin =
					state.node->_promoted_pivot_world + facing * 0.5f - rotation_basis.xform(pivot_in_child_local);
		}

		state.animatable_body->set_global_transform(world_t);
		state.node->set_global_transform(world_t);
	}

	for (VoxelSubGrid *sg : pending_disassembles) {
		if (sg->is_auto_align_pending_disassemble()) {
			VoxelLodTerrain *terrain = sg->consume_auto_align_disassemble_terrain();
			if (terrain != nullptr) {
				sg->try_disassemble(terrain);
			}
		}
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
		_update_ship_cell_index(uuid, world_t.origin);
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
	state.coarse_lod_active_count = 0;

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
	_remove_ship_from_cell_index(uuid);
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
		return;
	}
	state.own_collision_unsafe = state.coarse_lod_active_count > 0 || state.collision_built_chunks.size() == 0;
	_apply_combined_suspension(state);
}

void SubGridManager::_apply_combined_suspension(ShipState &state) {
	const bool should_be_suspended = state.own_collision_unsafe || !state.ground_confirmed;

	if (should_be_suspended == state.collision_suspended) {
		return;
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	ps->body_set_mode(
			state.body_rid, should_be_suspended ? PhysicsServer3D::BODY_MODE_STATIC : PhysicsServer3D::BODY_MODE_RIGID
	);
	if (!should_be_suspended) {
		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, Vector3());
		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, Vector3());
	}
	state.collision_suspended = should_be_suspended;
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
				print_line(
						String("QUEUE_FULL lod=") + itos(lod) + " pos=" + String(lod_pos) +
						" total_in_flight=" + itos(_total_in_flight())
				);
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
	ChunkMeshBlockState *block = state.lods[lod].mesh_state.getptr(lod_pos);
	const bool requires_geometry = (block == nullptr) ? true : block->requires_geometry;
	bool light_dirty = false;
	if (block != nullptr) {
		light_dirty = block->light_dirty;
		block->light_dirty = false;
	}

	std::shared_ptr<VoxelBuffer> padded;
	if (requires_geometry) {
		padded = _build_padded_buffer(state.node, lod_pos, lod);
		if (!padded) {
			/*print_line(String("SUBMIT lod=") + itos(lod) + " pos=" + String(lod_pos) + " ABORTED: no padded buffer");*/
			return;
		}
	}

	const bool should_flood = !requires_geometry || (lod == 0 && light_dirty);

	/*print_line(
			String("SUBMIT lod=") + itos(lod) + " pos=" + String(lod_pos) + " requires_geometry=" +
			(requires_geometry ? "true" : "false") + " light_dirty=" + (light_dirty ? "true" : "false") +
			" should_flood=" + (should_flood ? "true" : "false") + " -> " +
			(should_flood ? "REAL_FLOOD" : (requires_geometry && lod > 0 ? "EXTRACT_FROM_DATA5" : "NO_LIGHT_WORK"))
	);*/

	std::shared_ptr<VoxelBuffer> light_padded;
	std::shared_ptr<VoxelBuffer> data5_extract;
	if (should_flood) {
		light_padded = _build_light_padded_buffer(state.node, lod_pos, lod);
	} else if (requires_geometry && lod > 0) {
		data5_extract = _build_data5_extract_buffer(state.node, lod_pos, lod);
		/*print_line(
				String("SUBMIT lod=") + itos(lod) + " pos=" + String(lod_pos) +
				" data5_extract_buffer=" + (data5_extract ? "built" : "NULL (neighbor chunk missing?)")
		);*/
	}

	SubGridMeshTaskInput input;
	input.ship_uuid = uuid;
	input.chunk_pos = lod_pos;
	input.lod = lod;
	input.padded_buffer = std::move(padded);
	input.light_padded_buffer = std::move(light_padded);
	input.data5_extract_buffer = std::move(data5_extract);
	input.should_flood = should_flood;
	input.requires_geometry = requires_geometry;
	input.mesher = _mesher;
	input.build_collision = requires_geometry && (lod == 0) && !state.collision_built_chunks.has(lod_pos);
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
	if (state == nullptr || state->node == nullptr || state->load_state != LOADED){
		return;
	}

	const int lod = result.lod;
	const Vector3i lod_pos = result.chunk_pos;
	uint64_t key = _chunk_mesh_key(lod_pos, lod);

	// -------- Flood-only: update just the light texture, touch nothing else. --------
	if (result.light_only) {
		ChunkRenderData *render = state->chunk_renders.getptr(key);
		if (render != nullptr && render->chunk_material.is_valid() && result.output.light_surface.was_computed) {
			_update_chunk_light_texture(*render, result.output.light_surface.texture_data, lod);
		}
		return;
	}

	// -------- Normal path (unchanged from before, plus texture assignment at creation) --------
	ShipLod &ship_lod = state->lods[lod];

	// MIRROR of the staleness check in apply_mesh_update(): the chunk may have been unviewed
	// (refcount hit zero, erased from mesh_state) while this task was in flight. Discard.
	ChunkMeshBlockState *block = ship_lod.mesh_state.getptr(lod_pos);
	if (block == nullptr)
		return;
	block->state = MESH_UP_TO_DATE;
	block->requires_geometry = false; // up to date now; future unrelated neighbor edits may flood-only it

	if (block->pending_light_retry) {
		block->pending_light_retry = false;
		block->requires_geometry = true;
		notify_chunk_edited(*state, lod_pos, lod);
	}

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
			if (result.output.surfaces[i].arrays.size() == 0)
				continue;
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
			rs->instance_set_visible(instance, block->visual_active);

			ChunkRenderData render_data;
			render_data.instance_rid = instance;
			render_data.mesh = mesh;

			const int chunk_size = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
			const int surface_count = mesh->get_surface_count();
			for (int i = 0; i < surface_count; i++) {
				Ref<Material> base_mat = mesh->surface_get_material(i);
				Ref<ShaderMaterial> base_shader_mat = base_mat;
				if (base_shader_mat.is_valid()) {
					Ref<ShaderMaterial> override_mat = base_shader_mat->duplicate();
					override_mat->set_shader_parameter("u_block_size", chunk_size << lod);
					override_mat->set_shader_parameter("u_texture_border", TEXTURE_BORDER << lod);
					mesh->surface_set_material(i, override_mat);
					render_data.chunk_material = override_mat;
				}
			}

			if (render_data.chunk_material.is_valid() && result.output.light_surface.was_computed) {
				/*print_line(
						String("APPLY lod=") + itos(lod) + " pos=" + String(lod_pos) +
						" UPLOADING texture, texture_data.size=" + itos(result.output.light_surface.texture_data.size())
				);*/
				_update_chunk_light_texture(render_data, result.output.light_surface.texture_data, lod);
			} /*else {
				print_line(
						String("APPLY lod=") + itos(lod) + " pos=" + String(lod_pos) +
						" SKIPPED texture upload: chunk_material_valid=" +
						(render_data.chunk_material.is_valid() ? "true" : "false") +
						" was_computed=" + (result.output.light_surface.was_computed ? "true" : "false")
				);
			}*/

			state->chunk_renders[key] = std::move(render_data);
		}
	}

	// marks visual_loaded and notifyies the streaming system (ClipboxStreamingState::loaded_mesh_blocks there, state->pending_loaded_chunks here)
	// so the subdivision rule (update_mesh_block_load) can react next frame.
	if (!block->visual_loaded) {
		block->visual_loaded = true;
		state->pending_loaded_chunks.push_back(LoadedChunkEvent{ lod_pos, (uint8_t)lod });
	}

	// Persist the LOD0 flood result into the real chunk buffer, then re-downsample, so LOD1+
	// (which never floods on its own) has correct CHANNEL_DATA5 to extract from next time it rebuilds.
	if (lod == 0 && result.output.light_surface.was_computed && !result.output.light_surface.data.empty()) {
		/*print_line(String("APPLY lod=0 pos=") + String(lod_pos) + " entering CHANNEL_DATA5 persist+repropagate")*/;
		std::shared_ptr<VoxelBuffer> real_buf = state->node->get_chunk_map_mut().get_chunk_buffer(lod_pos);
		if (real_buf) {
			const StdVector<uint8_t> &raw = result.output.light_surface.data;
			/*print_line(
					String("APPLY lod=0 pos=") + String(lod_pos) + " raw.size=" + itos(raw.size()) +
					" real_buf->get_volume()=" + itos(real_buf->get_volume())
			);*/
			if (raw.size() == real_buf->get_volume()) {
				real_buf->decompress_channel(VoxelBuffer::CHANNEL_DATA5);
				Span<uint8_t> dst;
				if (real_buf->get_channel_as_bytes(VoxelBuffer::CHANNEL_DATA5, dst)) {
					memcpy(dst.data(), raw.data(), raw.size());

					FixedArray<HashSet<Vector3i>, SUBGRID_MAX_LODS> affected;
					state->node->get_chunk_map_mut().update_lods_for_chunk(lod_pos, affected);

					for (int alod = 1; alod < _lod_count; alod++) {
						for (const Vector3i &apos : affected[alod]) {
							ChunkMeshBlockState *ablock = state->lods[alod].mesh_state.getptr(apos);
							if (ablock == nullptr)
								continue;

							if (ablock->state == MESH_UPDATE_SENT) {
								// A task for this exact chunk is already running right now (possibly reading
								// stale CHANNEL_DATA5, since it may have started before this flood completed).
								// Don't submit a second, racing task for the same chunk - instead flag it so
								// its OWN completion (which is guaranteed to happen after this point) triggers
								// a guaranteed-correct follow-up.
								ablock->pending_light_retry = true;
							} else {
								ablock->requires_geometry = true;
								notify_chunk_edited(*state, apos, alod);
							}
						}
					}
				} /*else {
					print_line(
							String("APPLY lod=0 pos=") + String(lod_pos) +
							" FAILED to get_channel_as_bytes for CHANNEL_DATA5!"
					);
				}*/
			} /*else {
				print_line(String("APPLY lod=0 pos=") + String(lod_pos) + " SIZE MISMATCH, skipping persist entirely");
			}*/
		} /*else {
			print_line(
					String("APPLY lod=0 pos=") + String(lod_pos) + " real_buf is NULL (chunk not found in chunk map)!"
			);
		}*/
	}

	if (result.has_collision && !state->collision_built_chunks.has(lod_pos)) {
		_apply_collision_result(*state, lod_pos, result.collision);
		_recalculate_com(result.ship_uuid, *state);
		state->collision_built_chunks.insert(lod_pos);
	}
}


void SubGridManager::_update_chunk_light_texture(
		ChunkRenderData &render_data,
		const StdVector<uint8_t> &light_data,
		int lod
) {
	const int chunk_size = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const int block_size = chunk_size + 2 * TEXTURE_BORDER;
	const int voxel_count = block_size * block_size * block_size;
	if (static_cast<int>(light_data.size()) != voxel_count)
		return;



	PackedByteArray image_data;
	image_data.resize(voxel_count * 4);
	uint8_t *w = image_data.ptrw();
	for (int i = 0; i < voxel_count; ++i) {
		const uint8_t packed = light_data[i];
		w[i * 4 + 0] = uint8_t((packed & 0xF) * 17);
		w[i * 4 + 1] = uint8_t(((packed >> 4) & 0xF) * 17);
		w[i * 4 + 2] = 0;
		w[i * 4 + 3] = 255;
	}

	Vector<Ref<Image>> layers;
	layers.resize(block_size);
	for (int z = 0; z < block_size; ++z) {
		PackedByteArray layer_data;
		layer_data.resize(block_size * block_size * 4);
		uint8_t *lw = layer_data.ptrw();
		for (int x = 0; x < block_size; ++x) {
			for (int y = 0; y < block_size; ++y) {
				const int src = (y + x * block_size + z * block_size * block_size) * 4;
				const int dst = (x + y * block_size) * 4;
				lw[dst + 0] = w[src + 0];
				lw[dst + 1] = w[src + 1];
				lw[dst + 2] = w[src + 2];
				lw[dst + 3] = w[src + 3];
			}
		}
		layers.write[z] = Image::create_from_data(block_size, block_size, false, Image::FORMAT_RGBA8, layer_data);
	}

	if (render_data.light_texture.is_null()) {
		render_data.light_texture.instantiate();
		render_data.light_texture->create(Image::FORMAT_RGBA8, block_size, block_size, block_size, false, layers);
	} else {
		render_data.light_texture->update(layers);
	}

	if (render_data.chunk_material.is_valid()) {
		render_data.chunk_material->set_shader_parameter("u_light_texture", Variant(render_data.light_texture));
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
	padded->create(ps, ps, ps, &chunk_map.get_format());
	for (int c = 0; c < SubGridChunkMap::SUBGRID_CHANNEL_COUNT; c++) {
		padded->fill(0, SubGridChunkMap::SUBGRID_CHANNELS[c]);
	}

	// Copy center chunk - every tracked channel, not just TYPE, so the mesher actually
	// receives color/data5 alongside shape instead of it being silently dropped here.
	for (int z = 0; z < cs; z++) {
		for (int y = 0; y < cs; y++) {
			for (int x = 0; x < cs; x++) {
				for (int c = 0; c < SubGridChunkMap::SUBGRID_CHANNEL_COUNT; c++) {
					const VoxelBuffer::ChannelId channel = SubGridChunkMap::SUBGRID_CHANNELS[c];
					uint32_t v = buf->get_voxel(x, y, z, channel);
					padded->set_voxel(v, x + pad, y + pad, z + pad, channel);
				}
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
				for (int c = 0; c < SubGridChunkMap::SUBGRID_CHANNEL_COUNT; c++) {
					const VoxelBuffer::ChannelId channel = SubGridChunkMap::SUBGRID_CHANNELS[c];
					uint32_t v = nbuf->get_voxel(nx, ny, nz, channel);
					padded->set_voxel(v, px, py, pz, channel);
				}
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
		sg->initialize_root_from_disk(meta, _saves_dir, _mesher, _library, _voxel_format);
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
		sg->initialize_child(meta, SubGridChunkMap(), _saves_dir, false, _voxel_format);
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

void SubGridManager::_check_terrain_snap_correction(
		ShipState &state,
		Vector3i render_grid_position,
		unsigned int lod_index
) {
	int col_vertex_max = -1;
	int col_index_max = -1;
	Array surface = _terrain->get_mesh_block_surface(render_grid_position, lod_index, col_vertex_max, col_index_max);
	if (surface.is_empty()) {
		return;
	}

	PackedVector3Array vertices = surface[Mesh::ARRAY_VERTEX];
	if (vertices.is_empty()) {
		return;
	}

	float min_local_y = vertices[0].y;
	for (const Vector3 &v : vertices) {
		if (v.y < min_local_y) {
			min_local_y = v.y;
		}
	}

	const int mesh_block_size = _terrain->get_mesh_block_size() << lod_index;
	const Vector3 chunk_local_origin = Vector3(render_grid_position * mesh_block_size);
	const float terrain_h_local = chunk_local_origin.y + min_local_y;
	const float terrain_h_world = _terrain->get_global_transform().xform(Vector3(0, terrain_h_local, 0)).y;

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Transform3D t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);

	Variant com_variant = ps->body_get_param(state.body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS);
	Vector3 com_local = com_variant;
	Vector3 com_world = t.xform(com_local);

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	float com_to_keel = com_local.y - static_cast<float>(state.min_chunk_y * cs);
	float hull_bottom_world_y = com_world.y - com_to_keel;

	float correction = terrain_h_world - hull_bottom_world_y;
	if (correction > _snap_correction_threshold) {
		t.origin.y += correction;
		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM, t);
	}
}

// ____________________________________________________________________________
// LOAD SAFETY
void SubGridManager::_update_ship_cell_index(const String &uuid, Vector3 world_pos) {
	const Vector3i new_cell = _world_pos_to_cell(world_pos);

	Vector3i *last_cell = _ship_last_cell.getptr(uuid);
	if (last_cell != nullptr && *last_cell == new_cell) {
		return;
	}

	if (last_cell != nullptr) {
		Vector<String> *old_bucket = _ship_cell_index.getptr(*last_cell);
		if (old_bucket != nullptr) {
			old_bucket->erase(uuid);
			if (old_bucket->is_empty()) {
				_ship_cell_index.erase(*last_cell);
			}
		}
	}

	_ship_cell_index[new_cell].push_back(uuid);
	_ship_last_cell[uuid] = new_cell;
}

void SubGridManager::_remove_ship_from_cell_index(const String &uuid) {
	Vector3i *last_cell = _ship_last_cell.getptr(uuid);
	if (last_cell == nullptr) {
		return;
	}
	Vector<String> *bucket = _ship_cell_index.getptr(*last_cell);
	if (bucket != nullptr) {
		bucket->erase(uuid);
		if (bucket->is_empty()) {
			_ship_cell_index.erase(*last_cell);
		}
	}
	_ship_last_cell.erase(uuid);
}
// ____________________________________________________________________________
// load TerrainLod signals
void SubGridManager::on_terrain_mesh_block_entered(Vector3i render_grid_position, unsigned int lod_index) {
	_on_terrain_ground_lod_event(render_grid_position, lod_index, /*entered=*/true);
}

void SubGridManager::on_terrain_mesh_block_exited(Vector3i render_grid_position, unsigned int lod_index) {
	_on_terrain_ground_lod_event(render_grid_position, lod_index, /*entered=*/false);
}

void SubGridManager::_on_terrain_ground_lod_event(Vector3i render_grid_position, unsigned int lod_index, bool entered) {
	if (_terrain == nullptr) {
		return;
	}
	const int mesh_block_size = _terrain->get_mesh_block_size() << lod_index;
	const Vector3 chunk_origin_world =
			_terrain->get_global_transform().xform(Vector3(render_grid_position * mesh_block_size));
	const Vector3i min_cell = _world_pos_to_cell(chunk_origin_world) - Vector3i(1, 1, 1);
	const Vector3i max_cell =
			_world_pos_to_cell(chunk_origin_world + Vector3(mesh_block_size, mesh_block_size, mesh_block_size)) +
			Vector3i(1, 1, 1);

	Vector3i cell;
	for (cell.x = min_cell.x; cell.x <= max_cell.x; ++cell.x) {
		for (cell.y = min_cell.y; cell.y <= max_cell.y; ++cell.y) {
			for (cell.z = min_cell.z; cell.z <= max_cell.z; ++cell.z) {
				Vector<String> *bucket = _ship_cell_index.getptr(cell);
				if (bucket == nullptr) {
					continue;
				}
				for (const String &uuid : *bucket) {
					ShipState *state = _ships.getptr(uuid);
					if (state == nullptr || state->load_state != LOADED || !state->body_rid.is_valid()) {
						continue;
					}

					const Vector3i ship_keel_block = _terrain_block_pos_for_ship_keel(*state, lod_index);
					if (ship_keel_block != render_grid_position) {
						continue;
					}

					//if (entered) {
					//	_check_terrain_snap_correction(*state, render_grid_position, lod_index); //snap logic i forget to mention in commit. dosent work
					//}
					else {
						state->ground_confirmed = false;
						_apply_combined_suspension(*state);
					}
				}
			}
		}
	}
}

Vector3i SubGridManager::_terrain_block_pos_for_ship_keel(const ShipState &state, unsigned int lod_index) const {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Transform3D t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
	Variant com_variant = ps->body_get_param(state.body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS);
	Vector3 com_local = com_variant;
	Vector3 com_world = t.xform(com_local);

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	float com_to_keel = com_local.y - static_cast<float>(state.min_chunk_y * cs);
	Vector3 keel_world = com_world;
	keel_world.y -= com_to_keel;

	Vector3 local = _terrain->get_global_transform().affine_inverse().xform(keel_world);
	return _terrain->voxel_to_mesh_block_position(local, lod_index);
}

// ____________________________________________________________________________
// load safety checks

void SubGridManager::_process_ground_confirmations() { //main safety loop callen from _process_physics
	const uint64_t now = Time::get_singleton()->get_ticks_msec();
	constexpr uint64_t RETRY_INTERVAL_MSEC = 200;

	for (auto &[uuid, state] : _ships) {
		
		if (state.load_state != LOADED || !state.body_rid.is_valid()) {
//			print_line(String("!!_ship not LOADED"));
			continue;
		}
		// Any ship whose own hull is safe but ground isn't confirmed yet is a candidate,
		// regardless of whether a terrain event ever told us to start looking.
		if (state.own_collision_unsafe || state.ground_confirmed) {
//			print_line(
//					String("!!own_collision_unsafe: ") + (state.own_collision_unsafe ? "true" : "false") +
//					String(". !!ground_confirmed: ") + (state.ground_confirmed ? "true" : "false")
//			);
			continue;
		}
		if (now < state.next_ground_check_msec) {
			continue;
		}
		state.next_ground_check_msec = now + RETRY_INTERVAL_MSEC;

		if (!_is_ship_footprint_terrain_loaded(state)) {
//			print_line(String("!!!_is_ship_footprint_terrain_loaded(state)"));
			continue; // cheap guard: don't bother raycasting if data isn't even loaded yet
		}
		if (_confirm_ground(state)) {
//			print_line(String("!!!state ground and rigibody active"));
			state.ground_confirmed = true;
			_apply_combined_suspension(state);
		}
	}
}

bool SubGridManager::_raycast_confirms_ground(const ShipState &state) const {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Transform3D t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
	Variant com_variant = ps->body_get_param(state.body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS);
	Vector3 com_local = com_variant;
	Vector3 com_world = t.xform(com_local);

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const float com_to_keel = com_local.y - static_cast<float>(state.min_chunk_y * cs);
	Vector3 keel_world = com_world;
	keel_world.y -= com_to_keel;

	const float margin = cs * 0.5f;

	PhysicsDirectSpaceState3D::RayParameters params;
	params.from = com_world; // per earlier fix, starts at COM not keel
	params.to = keel_world - Vector3(0, margin, 0);
	params.exclude.insert(state.body_rid); // keep as a safety net
	params.collide_with_bodies = true;
	params.collide_with_areas = false;
	params.collision_mask = 1; // terrain layer/mask bit 1, excludes all ship bodies (bits 10-12

	RID space = ps->body_get_space(state.body_rid);
	PhysicsDirectSpaceState3D *space_state = ps->space_get_direct_state(space);
	if (space_state == nullptr) {
		return false;
	}

	PhysicsDirectSpaceState3D::RayResult result;
	return space_state->intersect_ray(params, result);
}



bool SubGridManager::_is_ship_footprint_terrain_loaded(const ShipState &state) const {
	if (_terrain == nullptr || state.node == nullptr) {
		return false; // no terrain reference, conservatively treat as unsafe
	}
//	print_line(String("!!_is_ship_footprint_terrain_loaded"));

	const HashSet<Vector3i> &positions = state.node->get_chunk_map().get_all_chunk_positions();
	if (positions.is_empty()) {
		return false; // nothing to check against, but also nothing to stand on. unsafe
	}

	// Compute the LOD0 chunk bounds directly from known chunk positions, rather than relying
	// on a bounding-box accessor that doesn't exist on SubGridChunkMap.
	Vector3i min_pos = *positions.begin();
	Vector3i max_pos = min_pos;
	for (const Vector3i &pos : positions) {
		min_pos = min_pos.min(pos);
		max_pos = max_pos.max(pos);
	}

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const AABB ship_local_aabb(Vector3(min_pos * cs), Vector3((max_pos - min_pos + Vector3i(1, 1, 1)) * cs));

	const Transform3D t = state.node->get_global_transform();
	const AABB ship_world_aabb = t.xform(ship_local_aabb).grow(4.f);

	const Transform3D terrain_to_local = _terrain->get_global_transform().affine_inverse();
	const AABB local_aabb = terrain_to_local.xform(ship_world_aabb);

	const Box3i voxel_box = Box3i::from_min_max(
			math::floor_to_int(local_aabb.position), math::ceil_to_int(local_aabb.position + local_aabb.size)
	);

	return _terrain->get_storage().is_area_loaded(voxel_box);
}

bool SubGridManager::_voxel_footprint_is_solid(const ShipState &state) const {
	if (_terrain == nullptr || !state.body_rid.is_valid()) {
		return false;
	}

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Transform3D t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
	Variant com_variant = ps->body_get_param(state.body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS);
	Vector3 com_local = com_variant;
	Vector3 com_world = t.xform(com_local);

	Transform3D terrain_to_local = _terrain->get_global_transform().affine_inverse();
	Vector3 sample_local = terrain_to_local.xform(com_world);

	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const float com_to_keel = com_local.y - static_cast<float>(state.min_chunk_y * cs);
	const int sample_range_voxels = static_cast<int>(std::ceil(com_to_keel)) + 4; // correct: std::ceil takes/returns float

	const Vector3i top = math::floor_to_int(sample_local);
	const Vector3i bottom = top - Vector3i(0, sample_range_voxels, 0);
	const Box3i probe_box = Box3i::from_min_max(bottom, top + Vector3i(1, 1, 1));

	if (!_terrain->get_storage().is_area_loaded(probe_box)) {
		return false; // don't trust unloaded data either way; treat as "not confirmed yet"
	}

	for (int dy = 0; dy <= sample_range_voxels; dy++) {
		Vector3i probe_pos = top - Vector3i(0, dy, 0);
		VoxelSingleValue v;
		v.i = 0;
		v = _terrain->get_storage().get_voxel(probe_pos, VoxelBuffer::CHANNEL_TYPE, v);
		if (v.i != 0) {
			return true;
		}
	}
	return false;
}

bool SubGridManager::_chunk_buffer_is_pure_air(const std::shared_ptr<VoxelBuffer> &voxels) const {
	if (!voxels) {
//		print_line("_chunk_buffer_is_pure_air: voxels == nullptr");
		return false;
	}
	if (!voxels->is_uniform(VoxelBuffer::CHANNEL_TYPE)) {
//		print_line("_chunk_buffer_is_pure_air: channel not uniform (real per-voxel array)");
		return false;
	}
	uint64_t v = voxels->get_voxel(0, 0, 0, VoxelBuffer::CHANNEL_TYPE);
//	print_line(String("_chunk_buffer_is_pure_air: uniform value = ") + itos(v));
	return v == 0;
}

bool SubGridManager::_chunk_has_terrain_collision(Vector3i chunk_bpos, int chunk_size) const {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	// Shrink slightly so the probe can't clip into a neighboring chunk's collision shapes across a
	// shared seam (epsilon overlap at chunk boundaries).
	const float margin = 0.1f;
	const float box_size = static_cast<float>(chunk_size) - margin * 2.f;

	if (!_chunk_probe_shape.is_valid() || _chunk_probe_shape_size != box_size) {
		if (_chunk_probe_shape.is_valid()) {
			ps->free_rid(_chunk_probe_shape);
		}
		_chunk_probe_shape = ps->box_shape_create();
		ps->shape_set_data(_chunk_probe_shape, Vector3(box_size, box_size, box_size) * 0.5f);
		_chunk_probe_shape_size = box_size;
	}

	Transform3D probe_t;
	probe_t.origin = _terrain->get_global_transform().xform(
			Vector3(chunk_bpos * chunk_size) + Vector3(chunk_size, chunk_size, chunk_size) * 0.5f
	);

	PhysicsDirectSpaceState3D::ShapeParameters params;
	params.shape_rid = _chunk_probe_shape;
	params.transform = probe_t;
	params.collision_mask = 1; // terrain layer only, same convention as _raycast_confirms_ground
	params.collide_with_bodies = true;
	params.collide_with_areas = false;

	PhysicsDirectSpaceState3D *space_state = ps->space_get_direct_state(_physics_space);
	if (space_state == nullptr) {
		return false;
	}

	// max_results = 1: we only care whether anything is there, not what or how much. Lets the
	// physics engine early-out on first hit instead of enumerating every overlapping shape.
	PhysicsDirectSpaceState3D::ShapeResult result;
	int hit_count = space_state->intersect_shape(params, &result, 1);
	return hit_count > 0;
}

bool SubGridManager::_check_open_air_column(ShipState &state, Vector3i first_below_chunk, Vector3 com_world) {
	print_line(String("!!_check_open_air_column"));
	VoxelEngine::Viewer::Distances view_distances;
	Vector3 viewer_world_pos;
	bool has_viewer = false;
	VoxelEngine::get_singleton().for_each_viewer(
			[&view_distances, &viewer_world_pos, &has_viewer](ViewerID, const VoxelEngine::Viewer &viewer) {
				if (!has_viewer) {
					view_distances = viewer.view_distances;
					viewer_world_pos = viewer.world_position;
					has_viewer = true;
				}
			}
	);
	if (!has_viewer) {
		return false;
	}

	const int cs = 1 << _terrain->get_data_block_size_pow2();
	// Convert viewer's world position to terrain/local chunk space, so it's directly comparable to first_below_chunk
	const Transform3D terrain_to_local = _terrain->get_global_transform().affine_inverse();
	const Vector3 viewer_local = terrain_to_local.xform(viewer_world_pos);
	const Vector3i viewer_chunk = _terrain->voxel_to_data_block_position(viewer_local, 0);

	const int vertical_chunk_offset = std::abs(viewer_chunk.y - first_below_chunk.y);

	const int lod0_chunks_total = static_cast<int>(view_distances.vertical) / cs;
	const int lod0_chunks_available = lod0_chunks_total - vertical_chunk_offset;
//	print_line(String("!!lod0_chunks_available =") + itos(lod0_chunks_available));
	if (lod0_chunks_available < 6) {
//		print_line(String("!!lod0_chunks_available less than 6"));
		// Not enough guaranteed-LOD0 vertical range to trust a multi-chunk walk; fall back to the simple single-ray check.
		return _raycast_confirms_ground(state);
	}

	Vector3i chunk_bpos = first_below_chunk;
	for (int i = 0; i < lod0_chunks_available; ++i, chunk_bpos.y -= 1) {
		const Box3i voxel_box(chunk_bpos * cs, Vector3i(cs, cs, cs));
		if (!_terrain->get_storage().is_area_loaded(voxel_box)) {
			return false;
		}

		std::shared_ptr<VoxelBuffer> voxels = _terrain->get_storage().try_get_block_voxels(chunk_bpos);

		if (voxels) {
//			print_line(String("!!buffer saved data avalible"));
			if (_chunk_buffer_is_pure_air(voxels)) {
//				print_line(String("!!buffer avalible is all air"));
				continue;
			}
			return _chunk_has_terrain_collision(chunk_bpos, cs);
		}

//		print_line(String("!!no resident buffer"));
		// No resident buffer, cheap generator-only estimate before paying for a physics query.
		const Vector3i chunk_top_voxel = chunk_bpos * cs + Vector3i(0, cs - 1, 0);
		if (!_generator_column_is_solid(chunk_top_voxel, cs)) {
//			print_line(String("!!generated buffer all air"));
			continue; // generator says air, keep walking down without touching physics
		}
//		print_line(String("!!generated buffer not all air, _check_open_air_column returned false"));
		return _chunk_has_terrain_collision(chunk_bpos, cs);
	}
//	print_line(String("!!_check_open_air_column returned true"));
	return true;
}

bool SubGridManager::_confirm_ground(ShipState &state) {
	if (_terrain == nullptr || !state.body_rid.is_valid()) {
		return false;
	}
//	print_line(String("!!_confirm_ground"));

	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Transform3D t = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
	Variant com_variant = ps->body_get_param(state.body_rid, PhysicsServer3D::BODY_PARAM_CENTER_OF_MASS);
	Vector3 com_local = com_variant;
	Vector3 com_world = t.xform(com_local);

	const Transform3D terrain_to_local = _terrain->get_global_transform().affine_inverse();
	const Vector3 com_terrain_local = terrain_to_local.xform(com_world);

	const Vector3i com_chunk = _terrain->voxel_to_data_block_position(com_terrain_local, 0);
	const Vector3i below_chunk = com_chunk - Vector3i(0, 1, 0);

	const int cs = 1 << _terrain->get_data_block_size_pow2();
	const Box3i below_voxel_box(below_chunk * cs, Vector3i(cs, cs, cs));

	if (!_terrain->get_storage().is_area_loaded(below_voxel_box)) {
		// The chunk directly below the ship might be outside the footprint area the caller already
		// verified is loaded (_is_ship_footprint_terrain_loaded checks the ship's own footprint, not
		// necessarily one chunk further down). Fall back to the plain raycast rather than assume.
		return _raycast_confirms_ground(state);
	}

	std::shared_ptr<VoxelBuffer> below_voxels = _terrain->get_storage().try_get_block_voxels(below_chunk);

	if (below_voxels) {
		// We have real resident data (possibly edited), trust it directly.
		if (!_chunk_buffer_is_pure_air(below_voxels)) {
			return _raycast_confirms_ground(state);
		}
		return _check_open_air_column(state, below_chunk, com_world);
	}

	// No resident buffer, fall back to a cheap generator only estimate, explicitly a different,
	// weaker source of truth than confirmed resident data (won't reflect edits).
	const Vector3i below_chunk_top_voxel = below_chunk * cs + Vector3i(0, cs - 1, 0);
	if (!_generator_column_is_solid(below_chunk_top_voxel, cs)) {
		return _check_open_air_column(state, below_chunk, com_world);
	}
	return _raycast_confirms_ground(state);
}

bool SubGridManager::_generator_column_is_solid(Vector3i top_voxel, int sample_range_voxels) const {
	const Box3i box =
			Box3i::from_min_max(top_voxel - Vector3i(0, sample_range_voxels, 0), top_voxel + Vector3i(1, 1, 1));

	VoxelBuffer column(VoxelBuffer::ALLOCATOR_POOL);
	_terrain->get_storage().get_voxels_batch(box, VoxelBuffer::CHANNEL_TYPE, column);

	if (column.is_uniform(VoxelBuffer::CHANNEL_TYPE)) {
		return column.get_voxel(0, 0, 0, VoxelBuffer::CHANNEL_TYPE) != 0;
	}
	for (int y = 0; y < box.size.y; ++y) {
		if (column.get_voxel(0, y, 0, VoxelBuffer::CHANNEL_TYPE) != 0) {
			return true;
		}
	}
	return false;
}

//________________________
//disasembly
void SubGridManager::begin_homing_disassemble(
		VoxelSubGrid *sg,
		const Transform3D &target_t,
		VoxelLodTerrain *terrain,
		float speed
) {
	String uuid = uuid_for_node(sg);
	ShipState *state = _ships.getptr(uuid);
	ERR_FAIL_COND_MSG(state == nullptr, "VoxelSubGrid not registered.");
	ERR_FAIL_COND_MSG(!state->body_rid.is_valid(), "Cannot home a non-rigid subgrid.");

	state->homing_to_disassemble = true;
	state->homing_target_t = target_t;
	state->homing_terrain = terrain;
	state->homing_speed = MAX(speed, 0.1f);
	state->homing_start_msec = Time::get_singleton()->get_ticks_msec();


	//state->homing_speed = Math::max(speed, 0.1f);

	// Force RIGID regardless of current suspension state, same reasoning as grab_subgrid: this sequence needs to actively drive the 
	// body, suspension logic resumes control only if the sequence is cancelled/fails.
	PhysicsServer3D::get_singleton()->body_set_mode(state->body_rid, PhysicsServer3D::BODY_MODE_RIGID);
}


void SubGridManager::_drive_homing_disassembles(double delta) {
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	const uint64_t now = Time::get_singleton()->get_ticks_msec();

	for (auto &[uuid, state] : _ships) {
		if (!state.homing_to_disassemble || !state.body_rid.is_valid()) {
			continue;
		}

		Transform3D current = ps->body_get_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
		Vector3 linear_error = state.homing_target_t.origin - current.origin;
		Basis rotation_error_basis = state.homing_target_t.basis * current.basis.orthonormalized().inverse();
		Vector3 axis; real_t angle;
		rotation_error_basis.get_axis_angle(axis, angle);

		const float linear_dist = linear_error.length();
		const float angular_dist = Math::abs((float)angle);
		const bool arrived = linear_dist < 0.02f && angular_dist < Math::deg_to_rad(1.0f);
		const bool timed_out = (now - state.homing_start_msec) > state.homing_timeout_msec;

		if (arrived || timed_out) {
			// Snap is purely cosmetic here, the transform used for actual voxel placement is always homing_target_t (already verified 
			// clear at command time), never wherever  physics actually left the body, even if it got stuck and never reached this spot.
			ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM, state.homing_target_t);
			ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, Vector3());
			ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, Vector3());

			state.homing_to_disassemble = false;
			VoxelLodTerrain *terrain = state.homing_terrain;
			state.homing_terrain = nullptr;
			Transform3D committed_t = state.homing_target_t;

			if (terrain != nullptr && state.node != nullptr) {
				state.node->try_disassemble_at(terrain, committed_t);
			}
			continue;
		}

		Vector3 linear_vel = linear_dist > 0.0001f ? (linear_error / linear_dist) * state.homing_speed : Vector3();
		Vector3 angular_vel = angular_dist > 0.0001f ? axis.normalized() * (angle / MAX(delta, 0.001)) : Vector3();

		// Cap angular speed to something reasonable relative to homing_speed rather than an unbounded division-by-delta value.
		const float max_angular = Math::deg_to_rad(90.0) * state.homing_speed;
		if (angular_vel.length() > max_angular) {
			angular_vel = angular_vel.normalized() * max_angular;
		}

		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, linear_vel);
		ps->body_set_state(state.body_rid, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, angular_vel);
	}
}
std::shared_ptr<VoxelBuffer> SubGridManager::_build_light_padded_buffer(VoxelSubGrid *node, Vector3i lod_pos, int lod)
		const {
	const SubGridChunkMap &chunk_map = node->get_chunk_map();
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const int pad = SubGridChunkMap::LIGHT_PADDING;
	const int bs = cs + pad * 2;

	auto big = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_DEFAULT);
	big->create(bs, bs, bs);
	big->fill(0, VoxelBuffer::CHANNEL_TYPE);

	// LIGHT_PADDING (15) is smaller than chunk size (16), so a single layer of the 26 surrounding chunks (27 including center) always fully covers the padded region, light from this chunk can never need to reach two chunks out.
	for (int dz = -1; dz <= 1; dz++) {
		for (int dy = -1; dy <= 1; dy++) {
			for (int dx = -1; dx <= 1; dx++) {
				const Vector3i offset(dx, dy, dz);
				std::shared_ptr<VoxelBuffer> src = chunk_map.get_lod_chunk_buffer(lod_pos + offset, lod);
				if (!src)
					continue;

				// This neighbor chunk's voxel range [0,cs) maps into the big buffer at offset*cs + pad. Clip to the big buffer's own bounds.
				const Vector3i chunk_origin_in_big = offset * cs + Vector3i(pad, pad, pad);

				const int x0 = MAX(0, -chunk_origin_in_big.x);
				const int y0 = MAX(0, -chunk_origin_in_big.y);
				const int z0 = MAX(0, -chunk_origin_in_big.z);
				const int x1 = MIN(cs, bs - chunk_origin_in_big.x);
				const int y1 = MIN(cs, bs - chunk_origin_in_big.y);
				const int z1 = MIN(cs, bs - chunk_origin_in_big.z);

				for (int z = z0; z < z1; z++) {
					for (int x = x0; x < x1; x++) {
						for (int y = y0; y < y1; y++) {
							const uint32_t v = src->get_voxel(x, y, z, VoxelBuffer::CHANNEL_TYPE);
							big->set_voxel(
									v,
									chunk_origin_in_big.x + x,
									chunk_origin_in_big.y + y,
									chunk_origin_in_big.z + z,
									VoxelBuffer::CHANNEL_TYPE
							);
						}
					}
				}
			}
		}
	}
	return big;
}

std::shared_ptr<VoxelBuffer> SubGridManager::_build_data5_extract_buffer(VoxelSubGrid *node, Vector3i lod_pos, int lod)
		const {
	const SubGridChunkMap &chunk_map = node->get_chunk_map();
	const int cs = 1 << SubGridChunkMap::CHUNK_SIZE_PO2;
	const int pad = SubGridChunkMap::TEXTURE_BORDER;
	const int bs = cs + pad * 2;

	std::shared_ptr<VoxelBuffer> center = chunk_map.get_lod_chunk_buffer(lod_pos, lod);
	if (!center) {
		return nullptr;
	}

	auto padded = std::make_shared<VoxelBuffer>(VoxelBuffer::ALLOCATOR_DEFAULT);
	padded->create(bs, bs, bs);
	padded->fill(0, VoxelBuffer::CHANNEL_DATA5);

	for (int z = 0; z < cs; z++) {
		for (int x = 0; x < cs; x++) {
			for (int y = 0; y < cs; y++) {
				uint32_t v = center->get_voxel(x, y, z, VoxelBuffer::CHANNEL_DATA5);
				padded->set_voxel(v, x + pad, y + pad, z + pad, VoxelBuffer::CHANNEL_DATA5);
			}
		}
	}

	const Vector3i dirs[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	for (const Vector3i &dir : dirs) {
		std::shared_ptr<VoxelBuffer> nbuf = chunk_map.get_lod_chunk_buffer(lod_pos + dir, lod);
		if (!nbuf)
			continue;
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
				uint32_t v = nbuf->get_voxel(nx, ny, nz, VoxelBuffer::CHANNEL_DATA5);
				padded->set_voxel(v, px, py, pz, VoxelBuffer::CHANNEL_DATA5);
			}
		}
	}
	return padded;
}

} // namespace zylann::voxel
