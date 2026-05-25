#include "sub_grid_metadata.h"
#include "core/io/marshalls.h"

namespace zylann::voxel {

PackedByteArray SubGridMetadata::serialize() const {
	print_line("NEW SERIALIZE RUNNING");

	PackedByteArray bytes;

	// Avoid lambdas entirely - use a simple helper
	struct Writer {
		PackedByteArray &b;
		void raw(const void *src, int size) {
			int n = b.size();
			b.resize(n + size);
			memcpy(b.ptrw() + n, src, size);
		}
		void i32(int32_t v) {
			raw(&v, 4);
		}
		void i8(uint8_t v) {
			raw(&v, 1);
		}
		void f64(double v) {
			raw(&v, 8);
		}
		void f32(float v) {
			raw(&v, 4);
		}
	} w{ bytes };

	w.raw(uuid, 16);
	w.raw(parent_uuid, 16);

	w.i32(pivot_in_parent_local.x);
	w.i32(pivot_in_parent_local.y);
	w.i32(pivot_in_parent_local.z);

	w.i32(rotation_axis.x);
	w.i32(rotation_axis.y);
	w.i32(rotation_axis.z);

	w.f64(target_angle_rad);

	w.i8(is_root ? 1 : 0);
	w.i8((uint8_t)lock_mode);

	print_line(String("about to write world_position: ") + String(world_position));
	w.f32(world_position.x);
	w.f32(world_position.y);
	w.f32(world_position.z);

	// Verify bytes were written
	const uint8_t *check = bytes.ptr() + 66;
	print_line(
			String("bytes[66..69] after write: ") + String::num_int64(check[0], 16) + " " +
			String::num_int64(check[1], 16) + " " + String::num_int64(check[2], 16) + " " +
			String::num_int64(check[3], 16)
	);

	w.f32(world_rotation.x);
	w.f32(world_rotation.y);
	w.f32(world_rotation.z);
	w.f32(world_rotation.w);

	w.i32((int32_t)chunk_positions.size());
	for (int i = 0; i < chunk_positions.size(); i++) {
		w.i32(chunk_positions[i].x);
		w.i32(chunk_positions[i].y);
		w.i32(chunk_positions[i].z);
	}

	print_line(
			String("serialize: total bytes = ") + itos(bytes.size()) + String(" world_position = ") +
			String(world_position)
	);
	return bytes;
}

SubGridMetadata SubGridMetadata::deserialize(const PackedByteArray &bytes) {
	SubGridMetadata m;
	const uint8_t *r = bytes.ptr();
	int offset = 0;

	auto read4 = [&]() -> int32_t {
		int32_t v;
		memcpy(&v, r + offset, 4);
		offset += 4;
		return v;
	};
	auto read8 = [&]() -> double {
		double v;
		memcpy(&v, r + offset, 8);
		offset += 8;
		return v;
	};
	auto read1 = [&]() -> uint8_t { return r[offset++]; };
	auto readf = [&]() -> float {
		float v;
		memcpy(&v, r + offset, 4);
		offset += 4;
		return v;
	};

	memcpy(m.uuid, r + offset, 16);
	offset += 16;
	memcpy(m.parent_uuid, r + offset, 16);
	offset += 16;

	m.pivot_in_parent_local.x = read4();
	m.pivot_in_parent_local.y = read4();
	m.pivot_in_parent_local.z = read4();

	m.rotation_axis.x = read4();
	m.rotation_axis.y = read4();
	m.rotation_axis.z = read4();

	m.target_angle_rad = read8();

	m.is_root = read1() != 0;
	m.lock_mode = (LockMode)read1();

	// Print offset and raw bytes before reading world_position
	print_line(String("offset before world_position: ") + itos(offset));
	print_line(
			String("next 12 bytes (hex): ") + String::num_int64(r[offset], 16) + " " +
			String::num_int64(r[offset + 1], 16) + " " + String::num_int64(r[offset + 2], 16) + " " +
			String::num_int64(r[offset + 3], 16) + " | " + String::num_int64(r[offset + 4], 16) + " " +
			String::num_int64(r[offset + 5], 16) + " " + String::num_int64(r[offset + 6], 16) + " " +
			String::num_int64(r[offset + 7], 16) + " | " + String::num_int64(r[offset + 8], 16) + " " +
			String::num_int64(r[offset + 9], 16) + " " + String::num_int64(r[offset + 10], 16) + " " +
			String::num_int64(r[offset + 11], 16)
	);

	m.world_position.x = readf();
	m.world_position.y = readf();
	m.world_position.z = readf();

	print_line(String("world_position after read: ") + String(m.world_position));

	//m.world_position.x = readf();
	//m.world_position.y = readf();
	//m.world_position.z = readf();

	m.world_rotation.x = readf();
	m.world_rotation.y = readf();
	m.world_rotation.z = readf();
	m.world_rotation.w = readf();

	int count = read4();
	m.chunk_positions.resize(count);
	for (int i = 0; i < count; i++) {
		m.chunk_positions.write[i].x = read4();
		m.chunk_positions.write[i].y = read4();
		m.chunk_positions.write[i].z = read4();
	}

	return m;
}

} // namespace zylann::voxel