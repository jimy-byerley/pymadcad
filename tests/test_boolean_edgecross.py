from madcad import *

m1 = brick(width=vec3(2))
m2 = m1.transform(vec3(1, -0.5, 1))

m3 = boolean.difference(m1, m2)
show([m3], options={'display_wire':True})
