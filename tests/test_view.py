import sys
from PyQt5.QtWidgets import QApplication
from madcad.mathutils import vec3
from madcad.mesh import Mesh, Web, Wire
from madcad.view import *

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

app = QApplication(sys.argv)
scn = Scene()
scn.add(m)
scn.add(w)
scn.look(Box(center=fvec3(0), width=fvec3(1)))

scn.show()
sys.exit(app.exec())
