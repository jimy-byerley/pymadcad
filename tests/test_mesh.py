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
nprint('bri', bri)
m = ico + bri
m.check()

# test simple properties
def closeto(x, ref, prec):	return abs(x-ref)/ref < prec
r = 1
s = icosphere(vec3(0), r)
assert closeto(s.surface(), 4*pi*r**2, 0.05)
assert closeto(s.volume(), 4/3*pi*r**3, 0.05)

# test separation
l = m.islands()
assert len(l) == 2
l[0].check()
l[1].check()

# test frontiers
m = bri.frontiers(0,2,3)
nprint(m)
nprint(m.tracks, m.edges)
assert {tuple(e) : m.groups[m.tracks[i]]	for i,e in enumerate(m.edges)} == {(0,1):(0,2), (1,5):(2,3), (1,2):(0,3)}
m = bri.group({0,1})

m = bri.group({0,2}).frontiers({2,None})
assert set(m.edges) == {uvec2( 0, 4 ), uvec2( 4, 5 ), uvec2( 1, 5 )}

# test transform
m = Mesh([vec3(0,0,0), vec3(1,0,0), vec3(0,1,0)], [(0,1,2)]).transform(vec3(0,0,-5))
m.check()
# test distance
assert abs(mesh_distance(m, ico)[0] - 4) < 0.2

# test orientation
ico = icosphere(vec3(0), 1)
ico.faces = typedlist((f if random()>0.5 else (f[0],f[2],f[1])	for f in ico.faces), dtype=uvec3)

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

bri = brick(center=vec3(2,0,0), width=vec3(0.5))
m = bri.frontiers(0,2,3).frontiers({1,2,None})
assert set(m.indices) == {5, 1, 2}


# test group qualification
bri = brick(center=vec3(2,0,0), width=vec3(0.5))
bri.qualify('truc', select=0)
assert bri.groups[0] == {'truc':None}
assert bri.groups[1] == None
bri.qualify('machin')
assert bri.groups[0] == {'truc':None, 'machin':None}
assert bri.groups[1] == {'machin':None}
bri.qualify('bidule', select='truc')
assert bri.groups[0] == {'truc':None, 'machin':None, 'bidule':None}
assert bri.groups[1] == {'machin':None}
assert len(bri.group(['truc', 'machin']).faces) == 2
nprint('groups', bri.groups)
