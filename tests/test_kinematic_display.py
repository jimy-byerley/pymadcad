from madcad import vec3, show
from madcad.kinematic import *
from madcad.joints import *

s1 = Solid()
s2 = Solid(pose=(vec3(0,0,0.5), vec3(1,0,0)))
s3 = Solid()
s4 = Solid()
A = vec3(2,0,0)
B = vec3(0,2,0)
C = vec3(0,-1,1)
D = vec3(1,-2,0)
x = vec3(1,0,0)
y = vec3(0,1,0)
csts = [Pivot(s1,s2, (A,x)), Ball(s1,s2, B), Plane(s2,s3, (C,y), position=(C,C)), Punctiform(s3,s4,(D,y))]

makescheme(csts)
show([s1, s2, s3, s4, A, B, C, D])
