from madcad import vec3, Box, brick, quickdisplay
from madcad.mesh import suites
from madcad.cut import *
from madcad import cut

cube = brick(Box(center=vec3(0), width=vec3(2)))
w = [(0,1),(1,2),(2,3),(3,0),(1,5)]
#cut.cut(cube, w, ('depth', 0.2))
chamfer(cube, w, ('width', 0.3))
#bevel(cube, w, ('width', 0.3))

cube.mergeclose()
cube.check()
cube.options = {'debug_display':True, 'debug_points':True}
quickdisplay([cube], options={'display_wire':True})
