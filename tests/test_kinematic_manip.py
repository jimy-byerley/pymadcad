from madcad.mathutils import *
from madcad.primitives import Axis
from madcad.kinematic import *
from madcad.joints import *
from madcad.rendering import show
import numpy as np

np.set_printoptions(linewidth=1000, precision=3)

# # planar pivots
# show([Chain([
# 	Revolute((0,1), Axis(0*X,Z)),
# 	Revolute((1,2), Axis(1*X,Z)),
# 	Revolute((2,3), Axis(1.5*X,Z)),
# 	Revolute((3,4), Axis(2*X,Z)),
# 	])])

# # print('pivot loop')
# show([Kinematic([
# 	Revolute((0,1), Axis(O,Z)),
# 	Revolute((1,2), Axis(X,Z)),
# 	Revolute((2,3), Axis(X+Y,Z)),
# 	Revolute((3,0), Axis(Y,Z)),
# 	])])

# print('2 free pivot loops')
# show([Kinematic([
# 	Revolute((0,1), Axis(O,Z)),
# 	Revolute((1,2), Axis(X,Z)),
# 	Revolute((2,3), Axis(X+Y,Z)),
# 	Revolute((3,0), Axis(Y,Z)),
# 	Revolute((2,4), Axis(X-Z,Z)),
# 	Revolute((4,5), Axis(2*X-Z,Z)),
# 	Revolute((5,6), Axis(2*X+Y-Z,Z)),
# 	Revolute((6,2), Axis(X+Y-Z,Z)),
# 	])])

# # print('2 bound pivot loops')
# show([Kinematic([
# 	Revolute((0,1), Axis(O,Z)),
# 	Revolute((1,2), Axis(X,Z)),
# 	Revolute((2,3), Axis(X+Y,Z)),
# 	Revolute((3,0), Axis(Y,Z)),
# 	Revolute((1,4), Axis(2*X,Z)),
# 	Revolute((4,3), Axis(2*X+Y,Z)),
# 	])])

# njoints = 10
# joints = []
# for i in range(0, njoints, 2):
# 	joints.append(Revolute((i,i+1), Axis(vec3(i+1,i,0), X)))
# 	joints.append(Revolute((i+1,i+2), Axis(vec3(i+1,i+1,0), Y)))
# show([Chain(joints)])

show([Chain([
	Planar((0,1), Axis(X,-X)),
	Revolute((1,2), Axis(O,Z)),
	Planar((2,3), Axis(Y,Y)),
	])])
