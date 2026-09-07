#ifndef VOXEL_DELETE_BLOCK_ENTITY_TASK_H
#define VOXEL_DELETE_BLOCK_ENTITY_TASK_H

#include "../../constants/voxel_constants.h"
#include "../../util/tasks/threaded_task.h"
#include "voxel_stream_sqlite.h"

namespace zylann::voxel {

class DeleteBlockEntityTask : public IThreadedTask {
public:
	DeleteBlockEntityTask(Ref<VoxelStreamSQLite> stream, int64_t request_id, Vector3i chunk_pos, int local_key) :
			_stream(stream), _request_id(request_id), _chunk_pos(chunk_pos), _local_key(local_key) {}

	const char *get_debug_name() const override {
		return "DeleteBlockEntity";
	}

	void run(zylann::ThreadedTaskContext &ctx) override {
		ZN_ASSERT_RETURN(_stream.is_valid());
		_ok = _stream->delete_block_entity_internal(_chunk_pos, _local_key);
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
			_stream->emit_signal("block_entity_deleted", _request_id, _chunk_pos, _local_key, _ok);
		}
	}

private:
	Ref<VoxelStreamSQLite> _stream;
	int64_t _request_id;
	Vector3i _chunk_pos;
	int _local_key;
	bool _ok = false;
};

} // namespace zylann::voxel

#endif // VOXEL_DELETE_BLOCK_ENTITY_TASK_H
