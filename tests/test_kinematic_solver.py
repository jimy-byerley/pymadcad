import numpy as np
from pytest import skip, approx

from madcad.mathutils import *
from madcad.primitives import Axis
from madcad.kinematic import *
from madcad.joints import *

from . import visualcheck

np.random.seed(7)

def test_drotate():
	from madcad.joints import drotate
	epsilon = 1e-6
	for dir in [X, Y, Z, normalize(X+Y+Z), normalize(vec3(1,2,3))]:
		for i in range(20):
			print(dir, i)
			assert np.array(drotate(i,dir)) == approx(np.array(rotate(i+epsilon,dir)-rotate(i,dir))/epsilon, abs=10*epsilon)

def test_joints():
	def check(joint):
		for i in range(5):
			print(joint, i)
			
			pose = np.arange(len(flatten_state(joint.default))) + float(i)
			close = pose + 0.2
			pose = structure_state(pose, joint.default)		
			close = structure_state(close, joint.default)
			# close reciprocity
			# assert flatten_state(joint.inverse(joint.direct(pose), close=close)) == approx(flatten_state(pose))
			# derivative
			assert np.array(joint.grad(pose)) == approx(np.array(Joint.grad(joint, pose, 1e-6)), abs=1e-5)
			# far reciprocity
			pose = joint.direct(pose)
			assert np.array(joint.direct(joint.inverse(pose))) == approx(np.array(pose))
	
	check(Free((0,1)))
	check(Weld((0,1)))
	check(Revolute((0,1), Axis(O,normalize(X+Y+Z))))

def test_one_free_pivot():
	assert Kinematic([
		Revolute((0,1), Axis(O,Y)),
		]).solve() == approx(np.zeros(1))

def test_two_free_pivots():
	assert Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		]).solve() == approx(np.zeros(2))

def test_three_constrainted_pivots():
	# 3 constrainted pivots
	print(Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(2*X,Z)),
		Weld((0,3), translate(-0.5*X)), 
		]).solve(close=np.arange(3)))

def test_three_pivots_inverse():
	# 3 pivots to inverse
	print(Kinematic(
		inputs=[
			Revolute((0,1), Axis(O,Z)),
			Revolute((1,2), Axis(X,Z)),
			Revolute((2,3), Axis(2*X,Z)),
			],
		outputs=[3], 
		ground=0,
		).inverse([translate(-0.5*X)], close=np.arange(15)))

def test_free_loop():
	kin = Kinematic([
		Free((0,1)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		])
	print(kin.solve(close=structure_state(np.arange(10), kin.default)))

def test_pivot_loop():
	print(Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		]).solve(close=np.arange(4)))

def test_two_pivot_loops():
	print(Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		Revolute((2,4), Axis(X,Z)),
		Revolute((4,5), Axis(2*X,Z)),
		Revolute((5,6), Axis(2*X+Y,Z)),
		Revolute((6,2), Axis(X+Y,Z)),
		]).solve(close=np.arange(8)))

def test_two_bound_pivot_loops():
	print(Kinematic([
		Revolute((0,1), Axis(O,Z)),
		Revolute((1,2), Axis(X,Z)),
		Revolute((2,3), Axis(X+Y,Z)),
		Revolute((3,0), Axis(Y,Z)),
		Revolute((1,4), Axis(2*X,Z)),
		Revolute((4,3), Axis(2*X+Y,Z)),
		]).solve(close=np.arange(6)))

def test_stretched_chain():
	njoints = 80
	joints = []
	for i in range(0, njoints, 2):
		joints.append(Revolute((i,i+1), Axis(vec3(i+1,i,0), X)))
		joints.append(Revolute((i+1,i+2), Axis(vec3(i+1,i+1,0), Y)))
	joints[-1].solids = (len(joints)-1, 0)
	print(Kinematic(joints).solve(close=2*pi*np.random.random(len(joints)), precision=1e-3, maxiter=1000))

def test_looping_chain():
	njoints = 80
	joints = []
	for i in range(0, njoints, 2):
		joints.append(Revolute((i,i+1), Axis(X,X), Axis(O,X)))
		joints.append(Revolute((i+1,i+2), Axis(Y,Y), Axis(O,Y)))
	joints[-1].solids = (len(joints)-1, 0)
	print(Kinematic(joints).solve(close=2*pi*np.random.random(len(joints)), maxiter=1000))

def test_grid():
	skip('numerically unstable')
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
	print(Kinematic(joints).solve(close=2*pi*np.random.random(len(joints)), precision=1e-3, maxiter=1000))
