from madcad.mathutils import vec3
from madcad.mesh import Web
from madcad.generation import extrusion
from madcad.io import *

def test_io():
	original = extrusion(
			Web(
				[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0)],
				[(0,1), (1,2), (2,3), (3,0)],
				[0,1,2,3],
				),
			vec3(0,0,0.5))
	original.check()

	# test ply
	write(original, 'tests/test_io.ply')
	ply = read('tests/test_io.ply')
	ply.check()
	assert ply.issurface()

	# test stl
	write(original, 'tests/test_io.stl')
	stl = read('tests/test_io.stl')
	stl.check()
	assert stl.issurface()
