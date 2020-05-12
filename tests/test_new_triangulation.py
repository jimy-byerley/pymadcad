from nprint import nprint
from copy import deepcopy
from math import cos,sin,pi
from madcad.mathutils import vec3
from madcad.mesh import Mesh,Wire,Web, web
from madcad import primitives
from madcad.triangulation import *

nprint('triangle\n', 
	skeleton(Wire([vec2(0,0), vec2(1,0), vec2(0,1)]))
	)
nprint('rectangle\n',
	skeleton(Wire([vec2(0,0), vec2(1,0), vec2(1,2), vec2(0,2)]))
	)
# complex shape
# custom one
#shape = Wire([
		#vec2(0,-1), vec2(1,-1), vec2(2,-2), vec2(4,-1), vec2(3,1), vec2(3,3), vec2(2,4), 
		#vec2(0,4), vec2(-1,1), vec2(-3,-1), vec2(-2,-2),
		#])
# circle
#res = 50
#shape = Wire([ vec2(cos(i/res*2*pi), 2*sin(i/res*2*pi))	for i in range(res) ])
# haricot
res = 13
shape = Wire(
		[ vec3(2*cos(i/res*2*pi), 2*sin(i/res*2*pi), 0)	for i in range(res) ]
	+	[ vec3(  cos(i/res*2*pi),   sin(i/res*2*pi), 0)	for i in reversed(range(res)) ]
	)
shape.indices.append(shape.indices[0])
#print(shape.indices)
#nprint('complex shape\n', skeleton(shape))

#s = skeleton(deepcopy(shape))
#s.check()
#s.mergeclose(0.1)
#nprint(s.edges)

#f = triangulation_skeleton(deepcopy(shape))
#f.check()
#f.mergeclose(0.1)
#nprint(f.faces)

w = Web(
		[ vec3(2*cos(i/res*2*pi), 2*sin(i/res*2*pi), 0)	for i in range(res) ]
	+	[ vec3(  cos(i/res*2*pi),   sin(i/res*2*pi), 0)	for i in reversed(range(res)) ], 
		[ (i, (i+1)%res)  for i in range(res) ]  
		# hole
	+ 	[ (i +res, (i+1)%res +res)  for i in range(res) ]
		# cluster for the hole
	+ 	[ (i +res, (i+1)%res +res)  for i in range(res) ] 
		# connection bt the hole and the outline
	#+	[ (0,25), (25,0) ]		# y oriented
	#+	[ (12,13), (13,12) ]	# random
	)
print(w.edges)
#w = web(shape)
w.check()
f = triangulation_sweepline(w)
f.check()
nprint(f.faces)

#shape.points = [vec3(p,0) for p in shape.points]
#s.points = [vec3(p,0) for p in s.points]
#f.points = [vec3(p,0) for p in f.points]
#s.options['color'] = (0.2,0.4,1.0)
f.options.update({'debug_display':True, 'debug_points':True})
w.options.update({'debug_display':True, 'debug_points':True})

from madcad import view
import sys
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
main = scn3D = view.Scene()
#scn3D.add(s)
scn3D.add(w)
scn3D.add(f)
main.show()
sys.exit(app.exec())


