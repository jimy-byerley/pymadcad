from madcad import *
from madcad.rendering import *
from madcad.displays import *

scn = []
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
#scn.append(m)

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
#scn.append(w)

scn.append(Displayable(PointDisplay, vec3(1,1,1) ))
scn.append(Displayable(AxisDisplay, (vec3(2,0,0), vec3(0,0,1)) ))
scn.append(Displayable(BoxDisplay, Box(vec3(-3), vec3(-1,-2,-1.5)) ))
scn.append(Displayable(GridDisplay, vec3(0)))

show(dict(enumerate(scn)))


