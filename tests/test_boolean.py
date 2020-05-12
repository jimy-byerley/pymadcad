from time import time
from copy import deepcopy
from nprint import nprint
from madcad import vec3,mat4, rotate, Mesh
from madcad.boolean import difference, booleanwith, intersectwith
from madcad import boolean

from madcad import view, text
import sys
from PyQt5.QtWidgets import QApplication

app = QApplication(sys.argv)

main = scn3D = view.Scene()


m1 = Mesh(
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
	[	0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5	],
	[None] * 6,
	)
m2 = deepcopy(m1)
#
#m2.transform(vec3(0.6, 0.3, 0.4))
#m2.transform(rotate(mat4(1), 0.3, vec3(1,1,1)))
#
#m2.transform(vec3(0.6, 0.3, 0.4))
#m2.transform(vec3(0.5, 0.3, 0.4))
#m2.transform(rotate(mat4(1), 0.7, vec3(1,0,0)))
#
m2.transform(vec3(0.5, 0.3, 0.4))
m2.transform(rotate(mat4(1), 0.7, vec3(1,1,0)))
#
#m2.transform(vec3(1.99, 0.52, 0.51))
#
#m2.transform(vec3(2, 0.5, 0.5))

#boolean.debug_propagation = True
#boolean.scn3D = scn3D
#m3 = deepcopy(m1)
#intersectwith(m3, m2)
#booleanwith(m3, m2, True)

m3 = boolean.boolean(m1, m2, (True, False))
m3.mergeclose()
m3.strippoints()
m3.check()
assert m3.isenvelope()

# debug purpose
m3.options.update({'debug_display':True, 'debug_points':True, 'debug_faces':False})
m2.options.update({'debug_display':True, 'debug_points':False, 'debug_faces':'indices'})
m1.options.update({'debug_display':True, 'debug_points':False, 'debug_faces':'indices'})
#m3.groups = [None]*len(m3.faces)
#m3.tracks = list(range(len(m3.faces)))
# display
scn3D.add(m3)
#scn3D.add(m2)
#scn3D.add(m1)

#scn3D.add(debug_vox)

main.show()
sys.exit(app.exec())
