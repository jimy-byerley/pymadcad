from madcad.mathutils import *
from madcad.primitives import Axis
from madcad.kinematic import *
from madcad.joints import *
import numpy as np

print('1 pivot')
print(solve([
	Pivot((0,1), Axis(O,Y)),
	]))

print('2 pivots')
print(solve([
	Pivot((0,1), Axis(O,Z)),
	Pivot((1,2), Axis(X,Z)),
	]))

print('pivot loop')
print(solve([
	Pivot((0,1), Axis(O,Z)),
	Pivot((1,2), Axis(X,Z)),
	Pivot((2,3), Axis(X+Y,Z)),
	Pivot((3,0), Axis(Y,Z)),
	], init=np.random.random(4)))

print('2 free pivot loops')
print(solve([
	Pivot((0,1), Axis(O,Z)),
	Pivot((1,2), Axis(X,Z)),
	Pivot((2,3), Axis(X+Y,Z)),
	Pivot((3,0), Axis(Y,Z)),
	Pivot((2,4), Axis(X,Z)),
	Pivot((4,5), Axis(2*X,Z)),
	Pivot((5,6), Axis(2*X+Y,Z)),
	Pivot((6,2), Axis(X+Y,Z)),
	], init=np.random.random(8)))

print('2 bound pivot loops')
print(solve([
	Pivot((0,1), Axis(O,Z)),
	Pivot((1,2), Axis(X,Z)),
	Pivot((2,3), Axis(X+Y,Z)),
	Pivot((3,0), Axis(Y,Z)),
	Pivot((1,4), Axis(2*X,Z)),
	Pivot((4,3), Axis(2*X+Y,Z)),
	], init=np.random.random(6)))
