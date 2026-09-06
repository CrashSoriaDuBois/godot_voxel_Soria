#ifndef VOXEL_SAVE_CHUNK_TIMESTAMPS_TASK_H
#define VOXEL_SAVE_CHUNK_TIMESTAMPS_TASK_H

#include "../../constants/voxel_constants.h"
#include "../../util/containers/std_vector.h"
#include "../../util/tasks/threaded_task.h"
#include "voxel_stream_sqlite.h"

namespace zylann::voxel {

// Saves a batch of chunk "last modified" timestamps in a single SQLite transaction, off the main thread. Used for online sync bookkeeping (host authoritative time).
class SaveChunkTimestampsTask : public IThreadedTask {
public:
	SaveChunkTimestampsTask(Ref<VoxelStreamSQLite> stream, StdVector<Vector3i> chunk_positions, double timestamp) :
			_stream(stream), _chunk_positions(std::move(chunk_positions)), _timestamp(timestamp) {}

	const char *get_debug_name() const override {
		return "SaveChunkTimestamps";
	}

	void run(zylann::ThreadedTaskContext &ctx) override {
		ZN_PROFILE_SCOPE();
		ZN_ASSERT_RETURN(_stream.is_valid());
		_ok = _stream->save_chunk_last_modified_batch_internal(to_span(_chunk_positions), _timestamp);
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
		// Runs back on the main thread after the IO task completes. Nothing required by default; the caller doesn't need a per-chunk callback for
		// this kind of write. If you want failure visibility, emit a signal here,
		// e.g.: if (!_ok && _stream.is_valid()) {_stream->emit_signal("chunk_timestamps_save_failed", _chunk_positions.size());}
	}

private:
	Ref<VoxelStreamSQLite> _stream;
	StdVector<Vector3i> _chunk_positions;
	double _timestamp;
	bool _ok = false;
};

} // namespace zylann::voxel

#endif // VOXEL_SAVE_CHUNK_TIMESTAMPS_TASK_H
