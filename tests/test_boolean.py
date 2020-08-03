from time import time
from copy import deepcopy
from nprint import nprint
from madcad import vec3,mat4, rotate, Mesh, quat, transform, normalize
from madcad.boolean import difference, booleanwith, intersectwith
from madcad import boolean

from madcad.view import quickdisplay

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

m2 = m1.transform(vec3(0.5, 0.3, 0.4)).transform(quat(0.7*vec3(1,1,0)))

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
#m3.options.update({'debug_display':True, 'debug_points':True, 'debug_faces':False})
#m2.options.update({'debug_display':True, 'debug_points':False, 'debug_faces':'indices'})
#m1.options.update({'debug_display':True, 'debug_points':False, 'debug_faces':'indices'})
#m3.groups = [None]*len(m3.faces)
#m3.tracks = list(range(len(m3.faces)))
# display
quickdisplay([m3])

