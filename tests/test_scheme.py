from pnprint import nprint
				
from madcad import *
from madcad.scheme import *
from . import visualcheck

@visualcheck
def test_annotations():
	part = extrusion(flatsurface(wire(Circle((vec3(1), vec3(0,0,1)), 1))), vec3(-1,-3,-5))
	cube = brick(center=vec3(-4,0,0), width=vec3(1))
	part.mergeclose()
	
	results = []
	def check(create):
		first = create()
		second = create()
		try:
			assert first == first
			assert first == second
		except Exception as err:
			if isinstance(first, Scheme):
				nprint(vars(first))
				nprint(vars(second))
			raise
		
		results.append(first)
	
	# schemes must be comparable
	check(lambda: note_floating(vec3(-2,2,-2), text="break all sharp edges\n\nsurfaces must be Ra <= 1.6 unless otherwise indicated"))
	check(lambda: note_distance(vec3(4,0,1), vec3(4,2,-5), project=Y, tol=0.05))
	check(lambda: note_leading(part.group(0), text="ø5 ± 0.05"))
	check(lambda: note_label(part.group(2), text='A'))
	check(lambda: note_distance_planes(part.group(1), part.group(2), tol='H7'))
	check(lambda: note_distance_set(part.group(1), part.group(2), offset=2))
	check(lambda: note_angle_edge(part, (49,50)))
	check(lambda: note_angle_planes(cube.group(0), cube.group(2), offset=-1))

	return [part, cube] + results

@visualcheck
def test_annotation_radius():
	def test(line, mesh):
		return [
			line, mesh,
			note_radius(mesh),
			note_radius(line),
			note_bounds(mesh),
			]
			
	results = []

	increment = 4*X
	line = web(ArcCentered((1*increment,Z), 1*increment+X, 1*increment+Y))
	mesh = extrusion(line, Z)
	results.append(test(line, mesh))

	line = web(Circle((2*increment,Z), 1, resolution=('rad',0.2)))
	mesh = extrusion(line, Z)
	results.append(test(line, mesh))

	line = web(Circle((3*increment,Z), 1, resolution=('rad',0.2)))
	mesh = icosphere(3*increment, 1, resolution=('rad',0.2))
	results.append(test(line, mesh))

	return results
