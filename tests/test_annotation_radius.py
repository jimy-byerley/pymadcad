from madcad import *
from madcad.scheme import *

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

show(results)
