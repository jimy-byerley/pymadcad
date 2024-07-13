from madcad import vec3, Box, brick, show
from madcad.mesh import suites
from madcad.cut import *
from madcad import scaledir, normalize

cube = brick(Box(center=vec3(0), width=vec3(2))) #.transform(scaledir(normalize(vec3(1)), 0.5))
w = [(0,1),(1,2),(2,3),(0,3),(1,5),(0,4)]
#multicut(cube, w, ('width', 0.3))
#chamfer(cube, w, ('width', 0.3))
bevel(cube, w, ('width', 0.3))

cube.mergeclose()
cube.check()
#cube.options = {'debug_display':True, 'debug_points':True}
show([cube], options={'display_wire':True})
