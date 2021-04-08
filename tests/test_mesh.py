from random import random
from madcad import *
from madcad.mesh import *
from madcad.nprint import nprint


## test Mesh

ico = icosahedron(vec3(0), 1)
# test check with static definition
ico.check()
# test display
nprint(ico)
nprint(repr(ico))

# test concatenation
bri = brick(center=vec3(2,0,0), width=vec3(0.5))
m = ico + bri
m.check()

# test separation
l = m.islands()
assert len(l) == 2
l[0].check()
l[1].check()

# test frontiers
m = bri.frontiers(0,2,3)
assert {e:m.groups[m.tracks[i]]	for i,e in enumerate(m.edges)} == {(0,1):(0,2), (1,5):(2,3), (1,2):(0,3)}

# test transform
m = Mesh([vec3(0,0,0), vec3(1,0,0), vec3(0,1,0)], [(0,1,2)]).transform(vec3(0,0,-5))
m.check()
# test distance
assert abs(mesh_distance(m, ico)[0] - 4) < 0.2

# test orientation
ico = icosphere(vec3(0), 1)
ico.faces = [f if random()>0.5 else (f[0],f[2],f[1])	for f in ico.faces]

o = ico.orient()
o.check()
#assert len(o.outlines_unoriented()) == 0

o = ico.orient(vec3(1,0,0))
o.check()
#assert len(o.outlines_unoriented()) == 0

## test Web

# test islands
m = Web(
	[vec3(0), vec3(1,0,0), vec3(1,0,0), vec3(2,1,0), vec3(1,3,0), vec3(1,5,4), vec3(2,5,0)], 
	[(0,1), (1,2), (2,3), (4, 5), (5,6), (6,4)])
m.check()
assert len(m.islands()) == 2


