from madcad.mathutils import *
from madcad.primitives import Axis
from madcad.kinematic import *
from madcad.joints import *
from madcad.rendering import show
import numpy as np

# 2 pivots
show([Chain([
	Revolute((0,1), Axis(0*X,Z)),
	Revolute((1,2), Axis(1*X,Z)),
	Revolute((2,3), Axis(1.5*X,Z)),
	Revolute((3,4), Axis(2*X,Z)),
	])])

# print('pivot loop')
# show(Kinematic([
# 	Revolute((0,1), Axis(O,Z)),
# 	Revolute((1,2), Axis(X,Z)),
# 	Revolute((2,3), Axis(X+Y,Z)),
# 	Revolute((3,0), Axis(Y,Z)),
# 	]).solve(close=np.random.random(4)))
