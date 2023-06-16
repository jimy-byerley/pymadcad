import sys

from PyQt6.QtWidgets import QApplication

from madcad import view
from madcad.mathutils import vec3
from madcad.mesh import Mesh
from madcad.selection import *

m = Mesh(
	[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0),    vec3(2,-1,0), vec3(2,1,0),    vec3(-2,1,0), vec3(-2,-1,0)],
	[(0,1,2),(0,2,3), (4,0,3),(0,4,5),  (6,2,1),(2,6,7)],
	[0,0, 1,1, 2,2],
	)
print('all (10)\t', select(m, (0,1)).edges)
print('crossover (1)\t', select(m, (0,1), crossover).edges)
print('stopangle (3)\t', select(m, (0,1), stopangle(0.1)).edges)
print('straight (8)\t', select(m, (0,1), straight).edges)
print('short (4)\t', select(m, (0,1), short).edges)
print('short | straight (1)\t', select(m, (0,1), short | straight).edges)
print('edgenear\t', edgenear(m, (vec3(0,2,0), vec3(1,1,0))))

#m.options.update({'debug_display':True, 'debug_points':True})
#app = QApplication(sys.argv)
#scn3D = view.Scene()
#scn3D.add(m)
#scn3D.look(m.box())
#scn3D.show()
#sys.exit(app.exec())
