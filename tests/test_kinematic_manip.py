from madcad import vec3, normalize, show, Box
from madcad.kinematic import *
from madcad.joints import *

O = vec3(0)
X = vec3(1,0,0)
Y = vec3(0,1,0)
Z = vec3(0,0,1)

s0 = Solid(name='base')
s1 = Solid(pose=(vec3(0,0,0), vec3(0,0,1)))
s2 = Solid(pose=(vec3(2,0,1.5), 2*vec3(1,1,0)))
s3 = Solid()
s4 = Solid()
s5 = Solid(name='wrist')
s6 = Solid()

# basic structure
csts = [
	#Plane(s0,s1, (O,Z)),  
	#Pivot(s0,s1, (O,Z)),
	#Track(s0,s1, (vec3(0,0,10),X,Y), (O,X,Y)),
	Gliding(s0,s1, (O,Z)),
	Pivot(s1,s2, (vec3(0,0,1), X), (vec3(0,0,-1), X)),
	#Pivot(s2,s3, (vec3(0,0,2), normalize(vec3(1,1,0))), (O,X)),
	
	Pivot(s2,s3, (vec3(0,0,2), X), (O,X)),
	Pivot(s3,s4, (vec3(0,0,2), X), (O,X)),
	Pivot(s4,s5, (vec3(0,0,2), X), (O,X)),
	]

# 6 DoF robot arm
csts2 = [
	Pivot(s0,s1, (O,Z)),
	Pivot(s1,s2, (vec3(0,0,1), X), (O,X)),
	Pivot(s2,s3, (vec3(0,0,2), X), (O,X)),
	Pivot(s3,s4, (vec3(0,0,1), Z), (vec3(0,0,-1), Z)),
	Pivot(s4,s5, (O,X)),
	Pivot(s5,s6, (vec3(0,0,0.5), Z), (O,Z)),
	]

# custom structure
csts = [
	Ball(s0,s1, vec3(-1,0,0), O),
	Punctiform(s0,s2, (vec3(1,0,0.5),Z), O),
	Ball(s1,s3, vec3(0,0,1), O),
	Ball(s3,s4, vec3(0,0,2), O),
	Ball(s2,s4, vec3(0,0,3), O),
	]

# gears
csts = [
	Pivot(s0,s1, (vec3(0,1,0),Y), (O,Z)),
	Pivot(s0,s2, (vec3(1,0,0),X), (O,Z)),
	Pivot(s0,s4, (vec3(0,1,1),Y), (O,Z)),
	Gear(s1,s2, -1.5, (-0.5*Z,Z), (O,Z)),
	Pivot(s2,s3, (vec3(1,1,0),Z), (O,Z)),
	Gear(s1,s4, 3, (O,Z)),
	Helicoid(s4,s5, 0.2, (0.5*Z,Z)),
	Track(s0,s5, (vec3(0,2,1),Z,X), (Z,X,Y)),
	]

solvekin(csts, [s0], precision=1e-2)
makescheme(csts)
show([Kinematic(csts, [s0]), O, X, Y, Z])
