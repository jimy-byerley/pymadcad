from madcad import *
from madcad.triangulation import *

rectangle = Wire([vec3(0,0,0), vec3(2,0,0), vec3(2,3,0), vec3(0,3,0)])
# complex shape
# custom one
scomplex = Wire([
		vec3(0,-1,0), vec3(1,-1,0), vec3(2,-2,0), vec3(4,-1,0), vec3(3,1,0), vec3(3,3,0), vec3(2,4,0), 
		vec3(0,4,0), vec3(-1,1,0), vec3(-3,-1,0), vec3(-2,-2,0),
		])
# circle
res = 50
scircle = Wire([ vec3(cos(i/res*2*pi), 2*sin(i/res*2*pi), 0)	for i in range(res) ])
# haricot
res = 13
sharicot = Wire(
		[ vec3(2*cos(i/res*2*pi), 2*sin(i/res*2*pi), 0)	for i in range(res) ]
	+	[ vec3(  cos(i/res*2*pi),   sin(i/res*2*pi), 0)	for i in reversed(range(res)) ]
	)

# sequence of tests
for shapename in ['rectangle', 'scomplex', 'scircle', 'sharicot']:
	print('test with', shapename)
	mesh = triangulation_outline(globals()[shapename], vec3(0,0,1))
	mesh.check()
	mesh.issurface()
	print(mesh.faces)

# display a specific case
shape = scomplex
mesh = triangulation_outline(shape, vec3(0,0,1))

show([shape, mesh], options={'display_wire':True})
