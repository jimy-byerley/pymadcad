# test intersections
from madcad import vec3, saddle, tube, ArcThrough, Web, web, bevel, chamfer
from madcad.mesh import suites
from madcad import view, text
from madcad.cut import multicut
import sys
from PyQt5.QtWidgets import QApplication
from nprint import nprint
from copy import deepcopy

app = QApplication(sys.argv)
main = scn3D = view.Scene()

m = saddle(
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
m.check()

#cut.cut_corner(m, 87, 0.6*vec3(0,0.2,0.05))
w = m.group(1).outlines_unoriented() & m.group(2).outlines_unoriented()
#multicut(m, w, ('depth', 0.6))
#chamfer(m, w, ('depth', 0.6))
bevel(m, w, ('depth', 0.6))
#beveltgt(m, w, ('depth', 0.6))
m.check()

#m.check()	# TODO fix the face using the same point multiple times
#assert m.issurface()

#m.options.update({'debug_display': True, 'debug_points': True })
scn3D.options.update({'display_wire':True})
scn3D.add(m)
scn3D.look(m.box())
main.show()
sys.exit(app.exec())
