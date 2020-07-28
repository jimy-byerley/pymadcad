import sys
from PyQt5.QtWidgets import QApplication
from madcad.mathutils import vec3
from madcad.mesh import Mesh, Web, Wire
from madcad.primitives import *
from madcad.view import *
from madcad.displays import *

app = QApplication(sys.argv)
scn = Scene()

m = Mesh(
	[
		vec3(1.0, -1.0, -1.0),
		vec3(1.0, -1.0, 1.0),
		vec3(-1.0, -1.0, 1.0),
		vec3(-1.0, -1.0, -1.0),
		vec3(1.0, 1.0, -1.0),
		vec3(1.0, 1.0, 1.0),
		vec3(-1.0, 1.0, 1.0),
		vec3(-1.0, 1.0, -1.0)],
	[
		(0, 1, 2),
		(0, 2, 3),
		(4, 7, 6),
		(4, 6, 5),
		(0, 4, 5),
		(0, 5, 1),
		(1, 5, 6),
		(1, 6, 2),
		(2, 6, 7),
		(2, 7, 3),
		(4, 0, 3),
		(4, 3, 7)],
	list(range(12)),
	)
scn.add(m)

w = Web(
	[	
		vec3(0,0,0),
		vec3(1,0,0),vec3(0,1,0),vec3(0,0,1),
		vec3(0,1,1),vec3(1,0,1),vec3(1,1,0),
		vec3(1,1,1)],
	[	
		(0,1),(0,2),(0,3),
		(1,6),(2,6),
		(1,5),(3,5),
		(2,4),(3,4),
		(4,7),(5,7),(6,7),
		],
	list(range(12)),
	)
w.transform(vec3(0,0,2))
scn.add(w)

scn.add(Displayable(PointDisplay, vec3(1,1,1) ))
scn.add(Displayable(AxisDisplay, (vec3(2,0,0), vec3(0,0,1)) ))
scn.add(Displayable(LengthMeasure, vec3(-2, 0, 0), vec3(0,-2,0), vec3(-1,-1,2) ))
scn.add(Circle((vec3(-2,-2,0), vec3(0,0,1)), 1) )
scn.add(Displayable(RadiusMeasure, Circle((vec3(-2,-2,0), vec3(0,0,1)), 1), vec3(-2,-4,0) ))
scn.add(Displayable(ArrowDisplay, (vec3(3,0,0), vec3(0,0,1)) ))
scn.add(Displayable(BoxDisplay, Box(vec3(-3), vec3(-1,-2,-1.5)) ))

scn.look(Box(center=fvec3(0), width=fvec3(1)))
scn.show()
sys.exit(app.exec())

