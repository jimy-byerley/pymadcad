from madcad import *
from madcad.mesh import *

# test islands
ico = icosahedron(vec3(0), 1)
# test check with static definition
ico.check()

# test concatenation
bri = brick(center=vec3(2,0,0), width=vec3(0.5))
m = ico + bri
m.check()
# test separation
l = m.islands()
assert len(l) == 2
l[0].check()
l[1].check()

# test transform
m = Mesh([vec3(0,0,0), vec3(1,0,0), vec3(0,1,0)], [(0,1,2)]).transform(vec3(0,0,-5))
m.check()
# test distance
assert abs(mesh_distance(m, ico)[0] - 4) < 0.2

#show(l)
