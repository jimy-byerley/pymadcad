from pnprint import nprint
from copy import deepcopy
from pytest import approx

from madcad.mathutils import *
from madcad import vec3, saddle, tube, ArcThrough, Web, Wire, web, wire, filet, chamfer, show, brick
from madcad.bevel import edgecut
from . import visualcheck

@visualcheck
def test_bevel():
	mesh = saddle(
			Web(
				[vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], 
				[(0,1), (1,2), (2,3), (3,4)],
				[0,1,2,3]),
			#web(vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)),
			#web(ArcThrough(vec3(0,1,-1),vec3(0,1.5,0),vec3(0,1,1))),
			web(
				ArcThrough(vec3(0,1,-1),vec3(0,1.3,-0.5),vec3(0,1,0)), 
				ArcThrough(vec3(0,1,0),vec3(0,0.7,0.5),vec3(0,1,1))),
			)
	box = mesh.box()
	edge = mesh.frontiers((1,2)) + mesh.frontiers((5,6))

	results = {}
	for i, operation in enumerate([edgecut, chamfer, filet]):
		for j, dim in enumerate(['width', 'depth', 'radius', 'distance']):
			print('testing  {} {}  ... '.format(operation.__name__, dim), end='')
			try:
				operated = deepcopy(mesh)
				operated.check()
				operation(operated, edge, **{dim: 0.6})
				operated.check()
				assert operated.issurface()
			except Exception as err:
				print('failed', err)
				raise
			else:
				results[(operation, dim)] = operated.transform(box.size * 1.2 * vec3(i,0,j))
				print('ok')

	return results

@visualcheck
def test_bevel_line_1():
	line = wire([
		vec3(0,0,0),
		vec3(1,0,0),
		vec3(1,1,0),
		vec3(3,3,0),
		vec3(2,4,1),
		vec3(0,2,2),
		vec3(0,0,0),
		])
		
	filet(line, [1,2,5], width=0.6)
	line.check()
	return [line]

@visualcheck
def test_bevel_line_2():
	line = web([
		vec3(0,0,0),
		vec3(1,0,0),
		vec3(1,1,0),
		vec3(3,3,0),
		vec3(2,4,1),
		vec3(0,2,2),
		vec3(0,0,0),
		])
	
	filet(line, [1,2,5], width=0.6)
	line.check()
	return [line]

@visualcheck
def test_bevel_cube():
	results = []
	def check(func, width):
		mesh = brick(Box(center=vec3(0), width=vec3(2))) #.transform(scaledir(normalize(vec3(1)), 0.5))
		func(mesh, [(0,1),(1,2),(2,3),(0,3),(1,5),(0,4)], width=width)
		mesh.check()
		mesh.issurface()
		mesh.mergeclose()
		mesh.check()
		results.append(mesh)
	
	check(edgecut, 0.3)
	check(chamfer, 0.3)
	check(filet, 0.3)
	return results



def arc_wire() -> Wire:
	P0 = vec3(0.08051, -2.024e-08, 0.1698)
	w = wire(
		[
			ArcThrough(
				vec3(-0.1988, 1.5e-10, -0.001258), vec3(-0.06793, -1.35e-08, 0.1132), P0
				),
			ArcThrough(
				P0, vec3(0.08932, -3.899e-09, 0.03271), vec3(0.2327, 1.335e-08, -0.112)
				),
			]
		)
	# w.finish()
	return w

def test_cut_curve():
	w = arc_wire()
	filet(w, [4], width=0.15)
	w.check()


def square_wire(b=2.0) -> Wire:
	"""
	A closed wire, shaped like a square
	"""
	points = [
		vec3(0, 0, 0),
		vec3(b, 0, 0),
		vec3(b, b, 0),
		vec3(0, b, 0),
		]
	inds = [0, 1, 2, 3, 0]
	return Wire(points, inds)


def chamfer_square(b, d) -> Wire:
	"""
	A square profile with rounded corners
	"""
	# low_res = resolution=("div", 1)
	wire = square_wire(b)
	chamfer(wire, [0, 1, 2, 3], distance=d)
	return wire


def test_chamfer():
	profile = chamfer_square(
		b := 2.0,
		d := 0.5,
	)
	expected_len = 4 * (b - d * 2) + 4 * sqrt(d)
	profile.finish()
	for p in profile.points:
		print(p)

	profile.check()
	assert profile.length() == approx(expected_len)


def bevel_square(b, r) -> Wire:
	"""
	A square profile with rounded corners
	"""
	res = ("div", 100)
	wire = square_wire(b)
	filet(wire, [0, 1, 2, 3], distance=r, resolution=res)
	return wire


def test_bevel():
	profile = bevel_square(b := 2.0, r := 0.5)
	d = r * 2
	expected_len = 4 * (b - d) + pi * d
	profile.finish()

	profile.check()
	assert profile.length() == approx(expected_len, abs=0.05)  # large tolerance because the bevel isn't an actual arc, and is discretized
