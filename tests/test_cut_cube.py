from madcad import vec3, Box, brick, quickdisplay
from madcad.mesh import suites
from madcad.cut import *

cube = brick(Box(center=vec3(0), width=vec3(2)))
#bevel(cube, [0,1,2,3,0], ('depth', 0.2))
#cube.mergeclose()
#bevel(cube, suites(cube.group(3).outlines_unoriented() & cube.group(2).outlines_unoriented())[0], ('depth', 0.2))
#chamfer(cube, [(0,1),(1,2),(2,3),(3,0),(3,4)], ('depth', 0.2))
from madcad import cut
print('pts', cube.points)
w = [(0,1),(1,2),(2,3),(3,0),(1,5)]
print(w)
cut.cut(cube, w, cut.planeoffsets(cube, w, ('depth', 0.2)))

cube.mergeclose()
cube.check()
cube.options = {'debug_display':True, 'debug_points':True}
quickdisplay([cube], options={'display_wire':True})
