from madcad.mathutils import *
from madcad.primitives import Axis
from madcad.kinematic import *
from madcad.joints import *
from madcad.rendering import show
from madcad import standard
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

# # print('2 free pivot loops')
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

# show([Chain([
# 	Planar((0,1), Axis(X,-X)),
# 	Revolute((1,2), Axis(O,Z)),
# 	Planar((2,3), Axis(Y,Y)),
# 	])])

# show([Chain([
# 	Prismatic((0,1), Axis(X,-X)),
# 	Ball((1,2), O),
# 	Reverse(Prismatic((3,2), Axis(Y,Y))),
# 	])])

# show([Chain([
# 	Ball((0,1), -X-Y),
# 	Ball((1,2), X-Y),
# 	Ball((2,3), X+Y),
# 	Ball((3,4), -X+Y),
# 	])])

# show([Kinematic([
# 	Ball((0,1), -X-Y),
# 	Ball((1,2), X-Y),
# 	Ball((2,3), X+Y),
# 	Ball((3,0), -X+Y),
# 	])])

# show([standard.serial6(1, 1), standard.serial7(1, 1)])

# show([standard.delta3(1, 0.5, 1, 2)])

from madcad.gear import gearprofile
from madcad.generation import repeataround
step = 1/3*(2*pi)/10
show([Chain([
    Gear((0,1), ratio=-0.5, centerline=1, axis=Axis(O,Z), local=Axis(O,Z)),
    ], content=[
    repeataround(gearprofile(step, 1/3*(2*pi)/step)),
    repeataround(gearprofile(step, 2/3*(2*pi)/step)) .transform(rotate(0.5*step/(2/3), Z)),
    ])])

# from madcad.gear import gearprofile, rackprofile
# from madcad.generation import repeataround, repeat
# step = 2*pi*1/10
# show([Chain([
#     Rack((0,1), 1, Axis(O,X), Axis(O,Z)),
#     ], content=[
#     repeat(rackprofile(step), 10, step*X) .transform(-1*Y + 0.*step*X),
#     repeataround(gearprofile(step, 1*2*pi/step)),
#     ])])
