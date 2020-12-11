from madcad import *
from madcad.blending import *

# test junction
m = blendpair(
		Wire([vec3(-2,0,0), vec3(-1,0,0), vec3(0,0,0), vec3(1,0,0), vec3(2,0,0)]),
		Wire([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)]),
		)
m.check()
assert m.issurface()
#m4.options.update({'debug_display': True, 'debug_points': True})

show([m], {'display_wire':True})
