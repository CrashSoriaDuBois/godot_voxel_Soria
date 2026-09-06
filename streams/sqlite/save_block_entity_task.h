#ifndef VOXEL_SAVE_BLOCK_ENTITY_TASK_H
#define VOXEL_SAVE_BLOCK_ENTITY_TASK_H

#include "../../constants/voxel_constants.h"
#include "../../util/containers/span.h"
#include "../../util/containers/std_vector.h"
#include "../../util/tasks/threaded_task.h"
#include "voxel_stream_sqlite.h"

namespace zylann::voxel {

class SaveBlockEntityTask : public IThreadedTask {
public:
	SaveBlockEntityTask(
			Ref<VoxelStreamSQLite> stream,
			int64_t request_id,
			Vector3i chunk_pos,
			int action_type,
			StdVector<uint8_t> data
	) :
			_stream(stream),
			_request_id(request_id),
			_chunk_pos(chunk_pos),
			_action_type(action_type),
			_data(std::move(data)) {}

	const char *get_debug_name() const override {
		return "SaveBlockEntity";
	}

	void run(zylann::ThreadedTaskContext &ctx) override {
		ZN_PROFILE_SCOPE();
		ZN_ASSERT_RETURN(_stream.is_valid());
		int local_key = -1;
		_ok = _stream->save_new_block_entity_internal(_chunk_pos, _action_type, to_span(_data), local_key);
		_assigned_key = local_key;
	}

	TaskPriority get_priority() override {
		TaskPriority p;
		p.band2 = constants::TASK_PRIORITY_SAVE_BAND2;
		p.band3 = constants::TASK_PRIORITY_BAND3_DEFAULT;
		return p;
	}

	bool is_cancelled() override {
		return false;
	}

	void apply_result() override {
		if (_stream.is_valid()) {
			_stream->emit_signal("block_entity_saved", _request_id, _chunk_pos, _assigned_key, _ok);
		}
	}

private:
	Ref<VoxelStreamSQLite> _stream;
	int64_t _request_id;
	Vector3i _chunk_pos;
	int _action_type;
	StdVector<uint8_t> _data;
	int _assigned_key = -1;
	bool _ok = false;
};

} // namespace zylann::voxel

#endif // VOXEL_SAVE_BLOCK_ENTITY_TASK_H
