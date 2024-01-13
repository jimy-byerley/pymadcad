from madcad import *

O = vec3(0)
X = vec3(1,0,0)
Y = vec3(0,1,0)
Z = vec3(0,0,1)

# we define the solids, they intrinsically have nothing particular
base = Solid()
s1 = Solid()
s2 = Solid()
s3 = Solid()
s4 = Solid()
s5 = Solid()
wrist = Solid(name='wrist')     # give it a fancy name

# the joints defines the kinematic.
# this is a 6 DoF (degrees of freedom) robot arm
csts = [
		Pivot(base,s1, (O,Z)),                   # pivot using axis (O,Z) both in solid base and solid 1
		Pivot(s1,s2, (vec3(0,0,1), X), (O,X)),   # pivot using different axis coordinates in each solid
		Pivot(s2,s3, (vec3(0,0,2), X), (O,X)),
		Pivot(s3,s4, (vec3(0,0,1), Z), (vec3(0,0,-1), Z)),
		Pivot(s4,s5, (O,X)),
		Pivot(s5,wrist, (vec3(0,0,0.5), Z), (O,Z)),
		]

# the kinematic is created with some fixed solids (they interact but they don't move)
kin = Kinematic(csts, fixed=[base])

show([kin])
