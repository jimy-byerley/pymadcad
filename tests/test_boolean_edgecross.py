from madcad import *

# 2 bricks intersecting
b1 = brick(width=vec3(2))
b2 = b1.transform(vec3(1, -0.5, 1))

br = boolean.difference(b1, b2)
br.check()
assert br.isenvelope()

# cylinders intersecting
o = vec3(0)
z = vec3(0,0,1)

def cylinder(axis, radius, height):
	return extrusion(
		-height*axis[1],
		flatsurface(wire(Circle(axis, radius, resolution=('rad', pi/16)))),
		)

cyl = difference(
	cylinder((o,z), 2, 2),
	cylinder((vec3(2,0,1),z), 1, 4),
	)
cyl.check()
assert cyl.isenvelope()

# display when everything happened fine
show([
	br, 
	cyl.transform(vec3(4,0,0)),
	], options={'display_wire':True})
