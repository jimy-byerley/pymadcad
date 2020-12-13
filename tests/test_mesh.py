from madcad import *

# test islands
ico = icosahedron(vec3(0), 1)
bri = brick(center=vec3(2,0,0), width=vec3(0.5))
m = ico + bri
l = m.islands()
assert len(l) == 2
l[0].check()
l[1].check()

#show(l)
