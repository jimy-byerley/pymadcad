from madcad.nprint import nprint
from math import pi
from madcad import *
from madcad.generation import *

results = []
absciss = 0
def place(shape):
	global absciss
	bounds = shape.box()
	absciss -= bounds.min.x
	p = absciss*X
	results.append([p, shape.transform(p).option(color=0.1+0.1*abs(vec3(sin(absciss), sin(absciss*1.5), sin(absciss*1.3))))])
	absciss += bounds.max.x + bounds.width.x*0.2

def test(shape):
	shape.check()
	#assert shape.issurface()
	place(shape)

# extransion

test(extrans(
	[vec3(6,1,0), vec3(4,1,0), vec3(4,-1,0), vec3(6,-1,0)], 
	[mat4(), translate(2*Z)*rotate(0.2, X), translate(4*Z)*rotate(0.2, Y)],
	))
test(extrusion(vec3(0,0,0.5), Web(
	[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0)],
	[(0,1), (1,2), (2,3), (3,0)],
	)))
test(revolution(pi, (vec3(0,0,0), vec3(0,1,0)), web(
	Segment(vec3(1,1,0), vec3(0.9,0.5,0)), 
	ArcThrough(vec3(0.9,0.5,0), vec3(0.7,0,0), vec3(0.9,-0.5,0)), 
	Segment(vec3(0.9,-0.5,0), vec3(1,-1,0)),
	)))

# test saddle
test(saddle(
	Web(
		[vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], 
		[(0,1), (1,2), (2,3), (3,4)],
		[0,1,2,3]),
	web(vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)),
	))
test(saddle(
	Web(
		[vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], 
		[(0,1), (1,2), (2,3), (3,4)],
		[0,1,2,3]),
	web(ArcThrough(vec3(0,1,-1), vec3(0,1.5,0), vec3(0,1,1))),
	))

# tubes	
test(tube(
	flatsurface(wire(Circle((vec3(0),vec3(0,0,-1)), 1))),
	[vec3(0,0,0), vec3(0,0,2), vec3(1,0,3), vec3(4,0,3)],
	))
test(tube(
	Web(
		[vec3(1,0,0), vec3(0,1,0), vec3(-1,0,0), vec3(0,-1,0), vec3(1,0,0)],
		[(0,1),(1,2),(2,3),(3,0)],
		[0,1,2,3],
		),
	[vec3(0,0,0), vec3(0,0,2), vec3(1,0,3), vec3(4,0,3)],
	))
test(tube(
	flatsurface(wire(Circle((vec3(0),vec3(0,0,-1)), 1))),
	ArcThrough(vec3(0,0,0), vec3(4,1,4), vec3(6,0,3)).mesh(),
	))
test(tube(
	Mesh([vec3(1,0,0), vec3(0,1,0), vec3(-1,0,0), vec3(0,-1,0), vec3(1,0,0)], [(0,2,1),(2,0,3)]),
	[vec3(0,0,0), vec3(0,0,2), vec3(1,0,3), vec3(4,0,3)],
	))

# icosurface
test(icosurface(
	[vec3(0.5,-1,0), vec3(0,1,0), vec3(0,0,1)], 
	[normalize(vec3(0.5,-1,0)), vec3(0,1,0), vec3(0,0,1)], 
	))
test(icosurface(
	[vec3(0.5,-1,0), vec3(0,1,0), vec3(0,0,1)], 
	[normalize(vec3(0.5,-1,0)), vec3(0,1,0), vec3(0,0,1)], 
	))
test(icosurface(
	[vec3(0.5,-1,0), vec3(0,1,0), vec3(0,0,1)], 
	[vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)], 
	))
test(icosurface(
	[vec3(0.5,-1,0), vec3(0,1,0), vec3(0,0,1)], 
	[1.57*vec3(1,0,0), 1.57*vec3(0,1,0), 1.57*vec3(0,0,1)],  
	))

# classic volumes
test(icosphere(vec3(0,3,0), 1))
test(pyramid(vec3(-3,-2,3), flatsurface(regon(Axis(vec3(-3,-3,0), Z), 2, 5))))
test(cone(vec3(3,0,2), vec3(3,0,0), 1))
test(cylinder(vec3(3,-3,2), vec3(3,-3,0), 1))
test(brick(vec3(1), vec3(1,2,3)))
test(parallelogram(vec3(2,-1,1), vec3(1,1,1)))
test(parallelogram(vec3(2,-1,1), vec3(1,1,1), vec3(0,0,1)))
test(parallelogram(vec3(2,-1,1), vec3(1,1,1), fill=False))
test(parallelogram(vec3(2,-1,1), vec3(1,1,1), vec3(0,0,1), fill=False))

show(results)
