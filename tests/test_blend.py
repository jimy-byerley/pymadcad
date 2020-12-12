from madcad import *
from madcad.blending import *

if False:
	# test junction
	m = blendpair(
			Wire([vec3(2,0,0), vec3(1,0,0), vec3(0,0,0), vec3(-1,0,0), vec3(-2,0,0)]),
			Wire([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)]),
			tangents='straight',
			)
	m.check()
	assert m.issurface()

	show([m], {'display_wire':True})

if True:
	m = blendloop(
			Wire([vec3(1,0,0), vec3(1,0.2,0), vec3(0.8, 0.6, 0.2), vec3(0.1, 0.9, 0.1), vec3(0,1.5, -0.2), vec3(-0.4, 0.5, -0.4), vec3(-0.6, -0.2, -0.3), vec3(-0.4,-0.3,-0.2), vec3(0, -1, 0), vec3(1,0,0)]),
			tangents='normal',
			weight=-1,
			)
	m.check()
	assert m.issurface()
	
	show([m], {'display_wire':True})
