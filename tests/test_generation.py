from nprint import nprint
from math import pi
from madcad import vec3, normalize, web, Web, Wire, ArcThrough, Segment, extrusion, revolution, saddle, tube, junction, icosurface, Mesh, wire, Circle
from madcad.generation import *
from madcad import show


# test extrusion
m1 = extrusion(vec3(0,0,0.5), Web(
		[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0)],
		[(0,1), (1,2), (2,3), (3,0)],
		[0,1,2,3],
		))
m1.check()
assert m1.issurface()

	
# test revolution
m2 = revolution(pi, (vec3(0,0,0), vec3(0,1,0)), web(
		Segment(vec3(1,1,0), vec3(0.9,0.5,0)), 
		ArcThrough(vec3(0.9,0.5,0), vec3(0.7,0,0), vec3(0.9,-0.5,0)), 
		Segment(vec3(0.9,-0.5,0), vec3(1,-1,0)),
		))
m2.check()
assert m2.issurface()

# test saddle
m3 = saddle(
		Web(
			[vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], 
			[(0,1), (1,2), (2,3), (3,4)],
			[0,1,2,3]),
		web(vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)),
		#web(Arc(vec3(0,1,-1),vec3(0,1.5,0),vec3(0,1,1))),
		)
m3.check()
assert m3.issurface()
#m.options.update({'debug_display': True, 'debug_points': False })

# test tubes
m4 = tube(
		#Web(
			#[vec3(1,0,0), vec3(0,1,0), vec3(-1,0,0), vec3(0,-1,0), vec3(1,0,0)],
			#[(0,1),(1,2),(2,3),(3,0)],
			#[0,1,2,3],
			#),
		#Mesh([vec3(1,0,0), vec3(0,1,0), vec3(-1,0,0), vec3(0,-1,0), vec3(1,0,0)], [(0,1,2),(2,3,0)]),
		flatsurface(wire(Circle((vec3(0),vec3(0,0,-1)), 1))),
		#Arc(vec3(0,0,0), vec3(4,1,4), vec3(6,0,3)).mesh()[0],
		[vec3(0,0,0), vec3(0,0,2), vec3(1,0,3), vec3(4,0,3)],
		)
m4.mergeclose()
m4.check()
assert m4.issurface()
#m4.options.update({'debug_display': True, 'debug_points': True})
m4 = m4.transform(vec3(-4,0,0))


# test junction
m5 = junction(
	dividematch(
	matchclosest(
		Wire([vec3(-2,0,0), vec3(-1,0,0), vec3(0,0,0), vec3(1,0,0), vec3(2,0,0)]),
		Wire([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)]),
	)))
m5.check()
assert m5.issurface()
#m4.options.update({'debug_display': True, 'debug_points': True})

# test icosurface
m6 = icosurface(
	[vec3(0.5,-1,0), vec3(0,1,0), vec3(0,0,1)], 
	[normalize(vec3(0.5,-1,0)), vec3(0,1,0), vec3(0,0,1)], 
	#[vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)], 
	#[1.57*vec3(1,0,0), 1.57*vec3(0,1,0), 1.57*vec3(0,0,1)],  
	#[1.57*vec3(1,0,0), 1.57*vec3(0,1,0), 1.57*vec3(0,0,1)], 
	)
m6.check()
assert m6.issurface()
#m6.options.update({'debug_display': True, 'debug_points':True})
m6 = m6.transform(vec3(0,0,-4))

m7 = icosphere(vec3(0,3,0), 1)
m7.options['color'] = (1, 0.5, 0.1)
#m7.options = {'debug_display':True, 'debug_points':True}

corner = Mesh(
			[vec3(0), vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)], 
			[(0,1,2), (0,2,3), (0,3,1)],
			[0,1,2]
			).transform(vec3(0,-3,0)).flip()
corner.options['color'] = (1, 0.5, 0.1)
m8 = thicken(corner, 0.1, 0.5)

show([m1, m2, m3, m4, m5, m6, m7, corner, m8])
