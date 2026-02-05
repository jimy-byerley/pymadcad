"""
Tests for Python <-> Rust data conversions.

Tests roundtrip correctness of:
- single vector conversions (vec3, uvec2, uvec3)
- typedlist conversions (typedlist[vec3], typedlist[vec2], typedlist[uvec2], typedlist[uvec3], typedlist[float], typedlist[u32])
"""

from madcad.core import test
from madcad.mathutils import vec3, uvec2, uvec3, vec2
from arrex import typedlist


# --- single vector passthrough roundtrips ---

def test_passthrough_vec3_zero():
	v = vec3(0, 0, 0)
	r = test.passthrough_vec3(v)
	assert type(r) is vec3
	assert r == v

def test_passthrough_vec3_values():
	v = vec3(1.5, -2.25, 3.75)
	r = test.passthrough_vec3(v)
	assert r.x == v.x
	assert r.y == v.y
	assert r.z == v.z

def test_passthrough_vec3_large():
	v = vec3(1e15, -1e15, 1e-15)
	r = test.passthrough_vec3(v)
	assert r == v

def test_passthrough_uvec2_values():
	v = uvec2(10, 20)
	r = test.passthrough_uvec2(v)
	assert type(r) is uvec2
	assert r.x == 10
	assert r.y == 20

def test_passthrough_uvec3_values():
	v = uvec3(100, 200, 300)
	r = test.passthrough_uvec3(v)
	assert type(r) is uvec3
	assert r.x == 100
	assert r.y == 200
	assert r.z == 300

def test_passthrough_uvec3_zero():
	v = uvec3(0, 0, 0)
	r = test.passthrough_uvec3(v)
	assert r == v


# --- single vector sequence generators ---

def test_sequence_vec3():
	r = test.sequence_vec3(5)
	assert type(r) is vec3
	assert r.x == 5.0
	assert r.y == 5.5
	assert r.z == 5.25

def test_sequence_uvec3():
	r = test.sequence_uvec3(10)
	assert type(r) is uvec3
	assert r.x == 10
	assert r.y == 11
	assert r.z == 12

def test_sequence_uvec2():
	r = test.sequence_uvec2(7)
	assert type(r) is uvec2
	assert r.x == 7
	assert r.y == 8


# --- typedlist passthrough roundtrips ---

def test_passthrough_typedlist_vec3():
	data = typedlist([vec3(1, 2, 3), vec3(4, 5, 6), vec3(7, 8, 9)])
	r = test.passthrough_typedlist_vec3(data)
	assert type(r) is typedlist
	assert len(r) == 3
	for i in range(3):
		assert r[i] == data[i]

def test_passthrough_typedlist_vec3_empty():
	data = typedlist(dtype=vec3)
	r = test.passthrough_typedlist_vec3(data)
	assert len(r) == 0

def test_passthrough_typedlist_vec2():
	data = typedlist([vec2(1.5, 2.5), vec2(3.5, 4.5)])
	r = test.passthrough_typedlist_vec2(data)
	assert len(r) == 2
	for i in range(2):
		assert r[i] == data[i]

def test_passthrough_typedlist_uvec2():
	data = typedlist([uvec2(0, 1), uvec2(2, 3), uvec2(4, 5)])
	r = test.passthrough_typedlist_uvec2(data)
	assert len(r) == 3
	for i in range(3):
		assert r[i] == data[i]

def test_passthrough_typedlist_uvec3():
	data = typedlist([uvec3(0, 1, 2), uvec3(3, 4, 5)])
	r = test.passthrough_typedlist_uvec3(data)
	assert len(r) == 2
	for i in range(2):
		assert r[i] == data[i]

def test_passthrough_typedlist_uvec3_empty():
	data = typedlist(dtype=uvec3)
	r = test.passthrough_typedlist_uvec3(data)
	assert len(r) == 0

def test_passthrough_typedlist_index():
	data = typedlist([10, 20, 30, 40], dtype='I')
	r = test.passthrough_typedlist_index(data)
	assert len(r) == 4
	for i in range(4):
		assert r[i] == data[i]

def test_passthrough_typedlist_float():
	data = typedlist([1.1, 2.2, 3.3], dtype=float)
	r = test.passthrough_typedlist_float(data)
	assert len(r) == 3
	for i in range(3):
		assert r[i] == data[i]


# --- typedlist sequence generators ---

def test_sequence_typedlist_vec3():
	r = test.sequence_typedlist_vec3(4)
	assert len(r) == 4
	for i in range(4):
		v = r[i]
		assert type(v) is vec3
		assert v.x == float(i)
		assert v.y == float(i) + 0.5
		assert v.z == float(i) + 0.25

def test_sequence_typedlist_vec3_empty():
	r = test.sequence_typedlist_vec3(0)
	assert len(r) == 0

def test_sequence_typedlist_uvec3():
	r = test.sequence_typedlist_uvec3(5)
	assert len(r) == 5
	for i in range(5):
		v = r[i]
		assert type(v) is uvec3
		assert v.x == i
		assert v.y == i + 1
		assert v.z == i + 2

def test_sequence_typedlist_uvec2():
	r = test.sequence_typedlist_uvec2(3)
	assert len(r) == 3
	for i in range(3):
		v = r[i]
		assert type(v) is uvec2
		assert v.x == i
		assert v.y == i + 1

def test_sequence_typedlist_vec2():
	r = test.sequence_typedlist_vec2(3)
	assert len(r) == 3
	for i in range(3):
		v = r[i]
		assert type(v) is vec2
		assert v.x == float(i)
		assert v.y == float(i) + 0.5

def test_sequence_typedlist_index():
	r = test.sequence_typedlist_index(5)
	assert len(r) == 5
	for i in range(5):
		assert r[i] == i

def test_sequence_typedlist_float():
	r = test.sequence_typedlist_float(4)
	assert len(r) == 4
	for i in range(4):
		assert abs(r[i] - i * 0.1) < 1e-15


# --- data integrity checks ---

def test_typedlist_vec3_precision():
	"""verify f64 precision is preserved through roundtrip"""
	import math
	data = typedlist([vec3(math.pi, math.e, math.sqrt(2))], dtype=vec3)
	r = test.passthrough_typedlist_vec3(data)
	assert r[0].x == math.pi
	assert r[0].y == math.e
	assert r[0].z == math.sqrt(2)

def test_typedlist_uvec3_max_value():
	"""verify u32 max values survive roundtrip"""
	mx = 2**32 - 1
	data = typedlist([uvec3(mx, 0, mx)], dtype=uvec3)
	r = test.passthrough_typedlist_uvec3(data)
	assert r[0].x == mx
	assert r[0].y == 0
	assert r[0].z == mx

def test_passthrough_preserves_length():
	"""large buffer roundtrip preserves all elements"""
	n = 1000
	data = typedlist([vec3(i, i*2, i*3) for i in range(n)])
	r = test.passthrough_typedlist_vec3(data)
	assert len(r) == n
	assert r[0] == vec3(0, 0, 0)
	assert r[n-1] == vec3(n-1, (n-1)*2, (n-1)*3)

def test_sequence_then_passthrough():
	"""generate in rust, roundtrip through python, pass back to rust"""
	generated = test.sequence_typedlist_vec3(10)
	roundtripped = test.passthrough_typedlist_vec3(generated)
	assert len(roundtripped) == 10
	for i in range(10):
		assert roundtripped[i] == generated[i]
