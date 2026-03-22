from pnprint import nprint

from random import random, seed as random_seed
from madcad import *
from madcad.mesh import *
from . import visualcheck

def test_init():
	ico = icosahedron(vec3(0), 1)
	ico.check()

@visualcheck
def test_repr():
	ico = icosahedron(vec3(0), 1)
	nprint(ico)
	nprint(repr(ico))
	return [repr(ico)]

def test_concatenation():
	ico = icosahedron(vec3(0), 1)
	bri = brick(center=vec3(2,0,0), width=vec3(0.5))
	nprint('bri', bri)
	m = ico + bri
	m.check()

def test_measures():
	def closeto(x, ref, prec):	return abs(x-ref)/ref < prec
	r = 1
	s = icosphere(vec3(0), r)
	assert closeto(s.surface(), 4*pi*r**2, 0.05)
	assert closeto(s.volume(), 4/3*pi*r**3, 0.05)

def test_islands():
	m = (
		icosahedron(vec3(0), 1)
		+ brick(center=vec3(2,0,0), width=vec3(0.5))
		)
	l = m.islands()
	assert len(l) == 2
	l[0].check()
	l[1].check()

def test_frontiers():
	bri = brick(center=vec3(2,0,0), width=vec3(0.5))
	m = bri.frontiers(0,2,3)
	nprint(m)
	nprint(m.tracks, m.edges)
	assert {tuple(e) : m.groups[m.tracks[i]]	for i,e in enumerate(m.edges)} == {(0,1):(0,2), (1,5):(2,3), (1,2):(0,3)}
	m = bri.group({0,1})

	m = bri.group({0,2}).frontiers({2,None})
	assert set(m.edges) == {uvec2( 0, 4 ), uvec2( 4, 5 ), uvec2( 1, 5 )}

@visualcheck
def test_transform():
	m1 = Mesh([vec3(0,0,0), vec3(1,0,0), vec3(0,1,0)], [(0,1,2)])
	m2 = m1.transform(vec3(0,0,-5))
	m1.check()
	m2.check()
	return [mat4(), m1, m2]

def test_distance():
	ico = icosahedron(vec3(0), 1)
	m = Mesh([vec3(0,0,0), vec3(1,0,0), vec3(0,1,0)], [(0,1,2)]).transform(vec3(0,0,-5))
	assert abs(mesh_distance(m, ico)[0] - 4) < 0.2

def test_orientation():
	random_seed(7)
	ico = icosphere(vec3(0), 1)
	ico.faces = typedlist((f if random()>0.5 else (f[0],f[2],f[1])	for f in ico.faces), dtype=uvec3)

	o = ico.own(faces=True).orient()
	o.check()
	assert len(o.outlines_unoriented()) == 0

	o = ico.own(faces=True).orient(vec3(1,0,0))
	o.check()
	assert len(o.outlines_unoriented()) == 0

def test_groups():
	# test group qualification
	bri = brick(center=vec3(2,0,0), width=vec3(0.5))
	bri.qualify('truc', select=0)
	assert bri.groups[0] == {'truc':None}
	assert bri.groups[1] == None
	bri.qualify('machin')
	assert bri.groups[0] == {'truc':None, 'machin':None}
	assert bri.groups[1] == {'machin':None}
	bri.qualify('bidule', select='truc')
	assert bri.groups[0] == {'truc':None, 'machin':None, 'bidule':None}
	assert bri.groups[1] == {'machin':None}
	assert len(bri.group(['truc', 'machin']).faces) == 2
	nprint('groups', bri.groups)

def test_split():
	# 5x2 grid strip: 6 points across, 2 rows, 10 triangles, 2 groups (top/bottom row)
	#   3---4---5---6---7---8
	#   |\ | \ | \ | \ | \ |
	#   | \|  \|  \|  \|  \|
	#   0---1---2---9--10--11
	pts = [vec3(i, 0, 0) for i in range(3)] + [vec3(i, 1, 0) for i in range(3)]
	pts += [vec3(i, 0, 0) for i in range(3, 6)] + [vec3(i, 1, 0) for i in range(3, 6)]
	faces = [
		uvec3(0,1,4), uvec3(0,4,3),   # group 0
		uvec3(1,2,5), uvec3(1,5,4),   # group 0
		uvec3(2,9,5),                   # frontier
		uvec3(9,10,8), uvec3(9,8,5),  # group 1
		uvec3(10,11,8), uvec3(10,8,7), # group 1 (dummy reuse)
		uvec3(5,8,6),                   # group 1
	]
	tracks = [0,0, 0,0, 0, 1,1, 1,1, 1]
	m = Mesh(pts, faces, tracks)
	assert len(m.faces) == 10

	# split along the frontier edge between group 0 and group 1
	frontier = m.frontiers().edges
	assert len(frontier) > 0
	original_npoints = len(m.points)
	m.split(frontier)

	# points on the frontier should have been duplicated
	assert len(m.points) > original_npoints
	assert len(m.faces) == 10
	# all face indices should be valid
	for face in m.faces:
		for p in face:
			assert p < len(m.points)

def test_vertexnormals():
	# subdivide a triangle diagonal on 2 axes, project onto a unit sphere,
	# then check that vertex normals are approximately radial
	center = vec3(0)
	m = Mesh(
		[vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)],
		[uvec3(0,1,2)],
		)
	m = m.subdivide(6)
	# project points onto the unit sphere (like icosphere does)
	for i, p in enumerate(m.points):
		m.points[i] = normalize(p - center)
	m.mergeclose()

	normals = m.vertexnormals()
	assert len(normals) == len(m.points)
	for i, n in enumerate(normals):
		expected = normalize(m.points[i] - center)
		# normal should be roughly radial (dot product close to 1)
		assert dot(n, expected) > 0.95, f"point {i}: normal {n} not radial to {expected}"

def test_display_buffers():
	import math
	from madcad import core

	# brick has 6 groups (one per face), 12 faces total
	m = brick(center=vec3(0), width=vec3(1))
	assert len(m.faces) == 12

	sharp_angle = math.radians(60)
	points, normals, faces, edges, idents = core.display_buffers_surface(m, sharp_angle)

	# must have buffers of matching lengths
	assert len(points) == len(normals)
	assert len(points) == len(idents)
	# faces and edges must be non-empty
	assert len(faces) > 0
	assert len(edges) > 0
	# after splitting at group frontiers and sharp edges, a cube should have
	# more points than the original 8 (each corner is shared by 3 groups)
	assert len(points) >= 24  # 6 faces * 4 corners each
	# all face indices must be valid
	for face in faces:
		for p in face:
			assert p < len(points), f"face index {p} out of range {len(points)}"
	# all edge indices must be valid
	for edge in edges:
		for p in edge:
			assert p < len(points), f"edge index {p} out of range {len(points)}"
	# idents should contain group ids that exist in the original mesh
	ngroups = len(m.groups)
	for ident in idents:
		assert ident < ngroups, f"ident {ident} out of range {ngroups}"
	# normals for referenced points should be unit length
	referenced = set()
	for face in faces:
		for p in face:
			referenced.add(int(p))
	for i in referenced:
		l = length(normals[i])
		assert abs(l - 1) < 0.01, f"normal {i} not unit length: {l}"
