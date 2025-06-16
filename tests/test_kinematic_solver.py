import numpy as np

from madcad.mathutils import *
from madcad.primitives import Axis
from madcad.kinematic import *
from madcad.joints import *

from . import visualcheck


@visualcheck
def test_one_free_pivot():
	return Kinematic([
		Revolute((0,1), Axis(O,Y)),
		]).solve()

@visualcheck
def test_two_free_pivots():
	return Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		]).solve()

@visualcheck
def test_three_constrainted_pivots():
	# 3 constrainted pivots
	return Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(2*X,Z)),
		Weld((0,3), translate(-0.5*X)), 
		]).solve(close=np.random.random(3))

@visualcheck
def test_three_pivots_inverse():
	# 3 pivots to inverse
	return Kinematic(
		inputs=[
			Revolute((0,1), Axis(O,Z)),
			Revolute((1,2), Axis(X,Z)),
			Revolute((2,3), Axis(2*X,Z)),
			],
		outputs=[3], 
		ground=0,
		).inverse([translate(-0.5*X)], close=np.random.random(15))

@visualcheck
def test_pivot_loop():
	return Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		]).solve(close=np.random.random(4))

@visualcheck
def test_two_pivot_loops():
	return Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		Revolute((2,4), Axis(X,Z)),
		Revolute((4,5), Axis(2*X,Z)),
		Revolute((5,6), Axis(2*X+Y,Z)),
		Revolute((6,2), Axis(X+Y,Z)),
		]).solve(close=np.random.random(8))

@visualcheck
def test_two_bound_pivot_loops():
	return Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		Revolute((1,4), Axis(2*X,Z)),
		Revolute((4,3), Axis(2*X+Y,Z)),
		]).solve(close=2*pi*np.random.random(6))

@visualcheck
def test_stretched_chain():
	njoints = 80
	joints = []
	for i in range(0, njoints, 2):
		joints.append(Revolute((i,i+1), Axis(vec3(i+1,i,0), X)))
		joints.append(Revolute((i+1,i+2), Axis(vec3(i+1,i+1,0), Y)))
	joints[-1].solids = (len(joints)-1, 0)
	return Kinematic(joints).solve(close=2*pi*np.random.random(len(joints)), precision=1e-3, maxiter=1000)

@visualcheck
def test_looping_chain():
	njoints = 80
	joints = []
	for i in range(0, njoints, 2):
		joints.append(Revolute((i,i+1), Axis(X,X), Axis(O,X)))
		joints.append(Revolute((i+1,i+2), Axis(Y,Y), Axis(O,Y)))
	joints[-1].solids = (len(joints)-1, 0)
	return Kinematic(joints).solve(close=2*pi*np.random.random(len(joints)), maxiter=1000)

@visualcheck
def test_grid():
	size = 5
	joints = []
	def name(x,y):
		return '{}-{}'.format(x,y)
	for x in range(size):
		for y in range(size):
			if x > 0:
				joints.append(Revolute((name(x-1,y), name(x,y)), Axis(vec3(1,0,0),Z), Axis(O,Z)))
			if y > 0:
				joints.append(Revolute((name(x,y-1), name(x,y)), Axis(vec3(0,1,0),Z), Axis(O,Z)))
	return Kinematic(joints).solve(close=2*pi*np.random.random(len(joints)), precision=1e-3, maxiter=1000)
