# This file is part of pymadcad,  distributed under license LGPL v3

''' This module defines the types and functions for kinematic manimulation and computation.

	
	A Kinematic is a conceptual approach of mechanisms. It sort parts in groups called solids (in solids all parts have the same movement), and links the solids to each other using constraints named joints.
	That way no matter what are the parts, or what are their shape, or how parts interact - solids movements can be deduced only from joints.

	This allows designing the mechanisms before designing its parts. This also allows visualizing the mechanism whether it is complete or not.
	
	As parts in the same solid all have the same movement, solids are considered to be undeformable. This allows the to use the Screw theory to represent the force and movement variables (see https://en.wikipedia.org/wiki/Screw_theory). 
	In this module, screws are called ``Screw``.
	
	This module mainly features:
		- `Joint` - the base class for all joints, instances of joints define a kinematic
		- `Kinematic` - the general kinematic solver
		- `Chain` - a joint and kinematic solver dedicated to kinematic chains
		- `Kinemanip` - a display to move mechanisms in the 3d view
			
	joints are defined in `madcad.joints`
'''

'''
	
	Kinematc([..., Free((0, n))])
	.parts([joint_position]) -> [solid_position]
	.solve({joint: position}, close: [joint_position]) -> [joint_position]
	.grad([joint_position]) -> [mat4]
	# kinematic solving with a set of constraints from the present constraints
	.solve(dict(zip(inverse, pose)), close)
	.solve(dict(zip(direct, pose)), close)
	# for convenience, we can define 
	Kinematic([..., Free((0,n))], direct=[...], inverse=[...])
	.direct([joint_position], close: [joint_position]) -> [solid_position]
	.inverse([solid_position], close: [joint_position]) -> [joint_position]

'''


from copy import copy, deepcopy
from dataclasses import dataclass
from numpy.core import ndarray
import itertools
import numpy as np
import scipy
import moderngl as mgl
from PyQt5.QtCore import Qt, QEvent	

from .common import resourcedir
from .mathutils import *
from .mesh import Mesh, Web, Wire, striplist, distance2_pm, typedlist_to_numpy
from . import settings
from . import rendering
from . import scheme
from . import nprint
from .displays import BoxDisplay

__all__ = ['Screw', 'comomentum', 
			'Solid', 'Kinematic', 'KinematicManip', 
			'Joint', 'Weld', 'Free', 'Reverse', 'Chain', 'ChainManip',
			'KinematicError',
			]

mat4x3 = dmat4x3


class KinematicError(Exception): 
	''' raised when a kinematic problem cannot be solved because the constraints cannot be satisfied or the solver cannot satisfy it '''
	pass
			


class Joint:
	'''
		A Joint constraints the relative position of two solids. 
		
		In this library, relative positioning is provided by transformation matrices which need a start-end convention, so every joint is directed and the order of `self.solids` matters.
		
		.. image:: /schemes/kinematic-joint.svg
		
		There is two ways of defining the relative positioning of solids
		
		- by a joint position :math:`(q_i)`
		- by a start-end matrix :math:`T_{ab}`
		
		.. image:: /schemes/kinematic-direct-inverse.svg
		
		we can switch from one representation to the other using the `direct` and `inverse` methods.
		
		Attributes:
		
			solids:  a tuple (start, end) or hashable objects representing the solids the joint is linking
			
			default:  a default joint position
			bounds:  a tuple of `(min, max)` joint positions
	'''
	def __init__(self, *args, default=0, **kwargs):
		if isinstance(args[0], Solid) and isinstance(args[1], Solid):
			self.solids = args[:2]
			args = args[2:]
		self.default = default
		self.init(*args, **kwargs)
	
	# parameter bounds if any
	bounds = (-inf, inf)
	default = 0
	
	def direct(self, parameters) -> 'mat4':
		''' direct kinematic computation
		
			Parameters:
				parameters:	
				
					the parameters defining the joint state
					It can be any type accepted by `numpy.array`
				
			Returns:
				the transfer matrix from solids `self.stop` to `self.start`
		'''
		raise NotImplemented
	
	def inverse(self, matrix, close=None) -> 'params':
		''' inverse kinematic computation
		
			the default implementation is using a least squares method on the matrix coefficients
			
			Parameters:
				matrix:	the transfer matrix from solids `self.stop` to `self.start` we want the parameters for
				close:  
				
					a know solution we want the result the closest to.
					if not specified, it defaults to `self.default`
			
			Returns:
				the joint parameters so that `self.direct(self.inverse(x)) == x` (modulo numerical precision)
		'''
		res = scipy.optimize.least_squares(
			lambda x: np.asanyarray(self.direct(x) - matrix).ravel()**2, 
			close if close is not None else self.default, 
			bounds=self.bounds,
			)
		if res.success:
			return res.x
		raise KinematicError('unable to inverse')
	
	def grad(self, parameters, delta=1e-6) -> '[mat4]':
		''' compute the gradient of the direct kinematic 
		
			The default implementation is using a finite differentiation
		
			Parameters:
				parameters:	anything accepted by `self.direct()` including singularities
				delta:	finite differentiation interval
			
			Returns:
				a list of the matrix derivatives of `self.direct()`, one each parameter
		'''
		grad = []
		base = self.direct(parameters)
		for i, x in enumerate(parameters):
			grad.append((partial_difference_increment(self.direct, i, x, delta) - base) / delta)
		return grad
	
	def transmit(self, force: 'Screw', parameters=None, velocity=None) -> 'Screw':
		''' compute the force transmited by the kinematic chain in free of its moves
			
			The default implementation uses the direct kinematic gradient to compute the moving directions of the chain
			
			Parameters:
				force:	force sent by `self.start` in its coordinate system
				
				parameters:  
					
					the joint position in which the joint is at the force transmision instant.
					If not specified it defaults to `self.default` (many joints transmit the same regardless of their position)
				
				velocity:  
				
					current derivative of `parameters` at the transmision instant
					Perfect joints are transmiting the same regardless of their velocity
					
			Returns:	
				force received by `self.stop` in its coordinate system
		'''
		if not parameters:	parameters = self.default
		grad = self.grad(parameters)
		indev
	
	def scheme(self, size: float, junc: vec3=None) -> '[Scheme]':
		''' generate the scheme elements to render the joint '''
		raise NotImplemented
	
	def display(self, scene):
		return scene.display(self.scheme(inf, None, None))


def partial_difference_increment(f, i, x, d):
	p = copy(x)
	try:
		p[i] = x[i] + d
		return self.direct(p)
	except KinematicError:
		try:
			p[i] = x[i] - d
			return self.direct(p)
		except KinematicError:
			pass
	raise ValueError('cannot compute below or above parameter {} given value'.format(i))

squeezed_homogeneous = 12
def squeeze_homogeneous(m):
	return np.asarray(dmat4x3(m)).ravel()
def unsqueeze_homogeneous(m):
	return mat4(mat4x3(m))


class Weld(Joint):
	''' 
		joint with no degree of freedom,
		simply welding a solid to an other with a transformation matrix to place one relatively to the other 
		
		It is useful to fix solids between each other without actually making it the same solid in a kinematic.
	'''
	dtype = np.dtype([])
	bounds = (None, None)
	default = None
	
	def __init__(self, solids, transform: mat4=None):
		self.solids = solids
		self.transform = transform or affineInverse(s1.pose) * s2.pose
	
	def direct(self, parameters):
		assert len(parameters) == 0
		return self.transform
		
	def inverse(self, matrix, close=None):
		return ()
		
	def grad(self, parameters, delta=1e-6):
		return ()
		
	def __repr__(self):
		return '{}({}, {})'.format(self.__class__.__name__, self.solids, self.transform)
		
class Free(Joint):
	'''
		joint of complete freedom.
		it adds no effective constraint to the start and end solids. its parameter is its transformation matrix.
		
		it is useful to control the explicit pose of a solid solid in a kinematic.
	'''
	dtype = np.dtype((float, (3, 4)))
	bounds = (
		mat4x3(-1, -1, -1,  -1, -1, -1,  -1, -1, -1,   -inf, -inf, -inf), 
		mat4x3(+1, +1, +1,  +1, +1, +1,  +1, +1, +1,   +inf, +inf, +inf),
		)
	default = mat4x3()
	
	def __init__(self, solids):
		self.solids = solids
	
	def direct(self, parameters):
		return mat4(parameters)
		
	def inverse(self, matrix, close=None):
		return mat4x3(matrix)
		
	def grad(self, parameters):
		grad = np.zeros((squeezed_homogeneous, 4, 4))
		for x in range(3):
			for y in range(4):
				grad[x+y][x][y] = 1
		return grad
	
	def __repr__(self):
		return '{}({})'.format(self.__class__.__name__, self.solids)

class Reverse(Joint):
	''' 
		that joint behaves like its wrapped joint but with swapped start and stop solids 
	
		.. image:: /schemes/kinematic-reverse.svg
	'''
	def __init__(self, joint):
		self.joint = joint
		self.solids = joint.solids[::-1]
		self.position = self.joint.position
		
	@property
	def dtype(self):    return self.joint.dtype
		
	@property
	def default(self):   return self.joint.default
		
	@property
	def bounds(self):    return self.joint.bounds
		
	def direct(self, parameters):
		return affineInverse(self.joint.direct(parameters))
		
	def inverse(self, matrix, close=None):
		return self.joint.inverse(hinverse(matrix), close)
		
	def grad(self, parameters):
		if hasattr(self.joint.default, '__len__'):
			f = self.joint.direct(parameters)
			return [- affineInverse(f) * df * affineInverse(f)
				for f, df in self.joint.grad(parameters)]
		else:
			f, df = self.joint.direct(parameters), self.joint.grad(parameters)
			return - affineInverse(f) * df * affineInverse(f)
		
	def __repr__(self):
		return '{}({})'.format(self.__class__.__name__, self.joint)
		
	def scheme(self, scene):
		a, b = self.joint.scheme(scene)
		return (b, a)

class Chain(Joint):
	''' 
		Kinematic chain, This chain of joints acts like one only joint
		The new formed joint has as many degrees of freedom as its enclosing joints.
		
		.. image:: /schemes/kinematic-chain.svg
		
		This class is often used instead of `Kinematic` when possible, because having more efficient `inverse()` and `direct()` methods dedicated to kinematics with one only cycle. It also has simpler in/out parameters since a chain has only two ends where a random kinematic may have many
		
		A `Chain` doesn't tolerate modifications of the type of its joints once instanciated. a joint placement can be modified as long as it doesn't change its hash.
	'''
	def __init__(self, joints, content:list=None):
		# TODO: add an argument for displays, like solids in Kinematic constructor
		
		if not all(joints[i-1].solids[-1] == joints[i].solids[0]  
				for i in range(1, len(joints))):
			raise ValueError('joints do not form a direct chain, joints are not ordered or badly oriented')
		self.joints = joints
		self.content = content
		self.solids = (joints[0].solids[0], joints[-1].solids[-1])
		self.dtype = np.dtype([(str(i), np.dtype(joint.dtype))   for i in enumerate(self.joints)])
		self.default = np.array(tuple([joint.default  for joint in self.joints]), self.dtype)
		self.bounds = (
					np.array(tuple([joint.bounds[0]   for joint in self.joints]), self.dtype),
					np.array(tuple([joint.bounds[1]   for joint in self.joints]), self.dtype),
					)
	
	def direct(self, parameters):
		b = mat4()
		for x, joint in zip(parameters, self.joints):
			b *= joint.direct(x)
		return b
	
	def inverse(self, matrix, close=None, precision=1e-6, maxiter=None):
		if close is None:
			close = self.default
		
		def cost(x):
			cost = squeeze_homogeneous(self.direct(structure_state(x, close)) - matrix)
			return np.asanyarray(cost).ravel()
			
		def jac(x):
			jac = self.grad(structure_state(x, close))
			return np.asanyarray(flatten(jac)).reshape((len(x), squeezed_homogeneous)).transpose((1,0))
		
		# solve
		res = scipy.optimize.least_squares(
					cost, 
					flatten_state(close, dtype), 
					method = 'trf', 
					bounds = (
						flatten_state(mins, dtype), 
						flatten_state(maxes, dtype),
						), 
					jac = jac, 
					xtol = precision, 
					max_nfev = maxiter,
					)
		if not res.success:
			raise KinematicError(res.message, res)
			
		return structure_state(res.x, close)
	
	def grad(self, parameters):
		# built the left side of the gradient product of the direct joint
		directs = []
		grad = []
		b = mat4(1)
		for x, joint in zip(parameters, self.joints):
			f = joint.direct(x)
			g = joint.grad(x)
			if isinstance(g, mat4) or isinstance(g, np.ndarray) and g.size == 16:
				g = g,
			directs.append((f, len(grad)))
			for df in g:
				grad.append(b*df)
			b = b*f
		# build the right side of the product
		b = mat4(1)
		i = len(grad)
		for f,n in reversed(directs):
			for i in reversed(range(n, i)):
				grad[i] = grad[i]*b
			b = f*b
		return grad
	
	def parts(self, parameters) -> list:
		''' return the pose of each solid in the chain '''
		solids = [mat4()] * (len(self.joints)+1)
		for i in range(len(self.joints)):
			solids[i+1] = solids[i] * self.joints[i].direct(parameters[i])
		return solids
		
	def to_kinematic(self) -> 'Kinematic':
		return Kinematic(direct=self.joints, inverse=[Free(self.solids)], ground=self.solids[0])
		
	def to_dh(self) -> '(dh, transforms)':
		''' 
			denavit-hartenberg representation of this kinematic chain. 
			
			it also returns the solids base definitions relative to the denavit-hartenberg convention, it the joints already follows the conventions, these should be eye matrices 
		'''
		indev
		
	def from_dh(dh, transforms=None) -> 'Self':
		''' build a kinematic chain from a denavit-hartenberge representation, and eventual base definitions relative to the denavit-hartenberg convention '''
		indev
		
	def __repr__(self):
		return '{}({})'.format(self.__class__.__name__, repr(self.joints))
		
	def display(self, scene):
		return ChainManip(scene, self)
	

class Kinematic:
	'''
		This class allows resolving direct and inverse kinematic problems with any complexity. 
		It is not meant to be a data format for kinematic, since the whole kinematic definition holds in joints. This class builds appropriate internal data structures on instanciation so that calls to `solve()` are fast and reproducible.
		Realtime (meaning fixed time resolution) is not a target, but reliability and convenience to compute any sort of mechanical interactions between solids.
		
		A kinematic is defined by its joints:
			- each joint works using position variables, the list of all joint positions is called the `state` of the kinematic
			- each joint is a link between 2 solids (start, stop)
			- each joint can provide a transformation matrix from its start solid to stop solid deduced from the joint position, as well as a gradient of this matrix
			
		A kinematic problem is defined by
			- the joints we fix (or solids we fix, but fixing a solid can be done using a joint)
			- the joints who stay free, whose positions need to be deduced from the fixed joints
		
		A list of joints can be seen as a graph of links between solids. The complexity of the kinematic probleme depends on
		
			- the number cycles
			- the degree of freedom
		
		Example:
			
			>>> # keep few joints apart, so we can use them as dict keys, for calling `solve`
			>>> motor1 = Pivot((0,2), Axis(...))
			>>> motor2 = Pivot((0,7), Axis(...))
			>>> free = Free((7,4))
			>>> # the kinematic solver object
			>>> kinematic = Kinematic([
			... 	Pivot((0,1), Axis(...)),
			... 	motor1,
			... 	motor2,
			... 	Planar((7,3), ...),
			... 	Gliding((1,3), ...),
			... 	Ball((3,2), ...),
			... 	Track((4,3), ...),
			... 	Planar((1,5), ...),
			... 	Planar((1,6), ...),
			... 	Weld((7,5), mat4(...)),
			... 	Weld((7,6), mat4(...)),
			... 	free,
			... 	], ground=7)
			
			defines a kinematic with the following graph
		
			.. image::
				/schemes/kinematic-kinematic.svg
		
		.. tip::
			If your kinematic is a chain of joints, then prefer using `Chain` to reduce the overhead of the genericity.
		
		.. note::
			A `Kinematic` doesn't tolerate modifications of the type of its joints once instanciated. joints positions could eventually be modified at the moment it doesn't affect the hash of the joints (See `Joint`)
		
		Attributes:
			joints: 
				
				a list of `Joint`, defining the kinematic
				these joints could for a connex graph or not, with any number of cycles
				
				each joint is a link between 2 solids, which are represented by a hashable object (it is common to designate these solids by integers, strings, or objects hashable by their id)
				
			solids:
				display object for each solid, this can be anything implementing the display protocol, and will be used only when this kinematic is displayed
				
			ground: the reference solid, all other solids positions will be relative to it
			
			direct_parameters:   a list of joints to fix when calling `direct()`
			inverse_parameters:  a list of joints to fix when calling `inverse()`
	'''
	def __init__(self, joints:list=[], content:dict=None, ground=0, direct=None, inverse=None):
		if (direct is None) ^ (inverse is None):
			raise TypeError("direct and inverse must be both provided or undefined")
		elif direct and inverse:	
			joints = direct + joints + inverse
			self.direct_parameters = direct
			self.inverse_parameters = inverse
		
		self.content = content
		self.joints = joints
		self.ground = ground
		
		self.dtype = np.dtype([(str(i), joint.dtype)  for i, joint in enumerate(self.joints)])
		self.default = np.array([tuple([joint.default  for joint in self.joints])], self.dtype)
		self.bounds = (
			np.array([tuple([joint.bounds[0]  for joint in self.joints])], self.dtype),
			np.array([tuple([joint.bounds[1]  for joint in self.joints])], self.dtype),
			)
		
		# collect the joint graph as a connectivity and the solids
		conn = {}  # connectivity {node: [node]}  with nodes being joints and solids
		for joint in joints:
			conn[joint] = joint.solids
			for solid in joint.solids:
				if solid not in conn:
					conn[solid] = []
				conn[solid].append(joint)
		
		# number of scalar variables
		vars = {joint: joint.dtype.itemsize / floatsize
			for joint in conn  
			if isinstance(joint, Joint)}
		
		# reversed joints using the tree
		tree = depthfirst(conn, starts=[ground])
		self.rev = {}
		
		# joint computation order, for direct kinematic
		self.order = []
		for parent, child in tree:
			if child in vars:
				joint = child
				if joint.solids[0] != parent:
					if joint not in self.rev:
						self.rev[joint] = Reverse(joint)
				self.order.append(joint)
		
		# decompose into cycles, for inverse kinematic
		self.cycles = []
		for cycle in shortcycles(conn, vars, branch=False, tree=tree):
			# cycles may not begin with a solid, change this
			if isinstance(cycle[0], Joint):
				cycle.pop(0)
				cycle.append(cycle[0])
			# # keep minimum number of reversed joints
			# inversions = sum(cycle[i-1] != cycle[i].solids[0]  
							# for i in range(1, len(cycle), 2))
			# if inversions > len(cycle)//2:
				# cycle.reverse()
			chain = []
			for i in range(1, len(cycle), 2):
				joint = cycle[i]
				if cycle[i-1] != joint.solids[0]:
					if joint not in self.rev:
						self.rev[joint] = Reverse(joint)
					joint = self.rev[joint]
				chain.append(joint)
			self.cycles.append(chain)
			
		nprint('cycles', self.cycles)
		
	def to_chain(self) -> 'Chain':
		indev
	
	def to_urdf(self):  
		indev
	
	def simplify(self) -> 'Self':
		indev
		
	def cost_residuals(self, state: ndarray, fixed=()) -> ndarray:
		'''
			build residuals to minimize to satisfy the joint constraints
			cost function returns residuals, the solver will optimize the sum of their squares
		'''
		# collect transformations
		state = iter(state[0])
		transforms = {}
		for joint in self.joints:
			if joint in fixed:
				transforms[joint] = fixed[joint]
			else:
				transforms[joint] = joint.direct(next(state, empty))
			if joint in self.rev:
				transforms[self.rev.get(joint)] = affineInverse(transforms[joint])
		
		# chain transformations
		residuals = np.empty((len(self.cycles), squeezed_homogeneous), float)
		for i, cycle in enumerate(self.cycles):
			# b is the transform of the complete cycle: it should be identity
			b = mat4()
			for joint in cycle:
				b = b * transforms[joint]
			# the residual of this matrix is the difference to identity
			# pick only non-redundant components to reduce the problem size
			residuals[i] = squeeze_homogeneous(b - mat4())
			#residuals[i] **= 2
		return residuals.ravel()
		
	def cost_jacobian(self, state: ndarray, fixed=()) -> ndarray:
		''' jacobian of the residuals function '''
		# collect gradient and transformations
		state = iter(state[0])
		transforms = {}
		index = 0
		nprint(self.joints)
		for joint in self.joints:
			if joint in fixed:
				grad = ()
				f = fixed[joint]
				size = 0
			else:
				p = next(state)
				print(joint, p)
				size = np.asarray(p).size
				grad = joint.grad(p)
				# ensure it is a jacobian and not a single derivative
				if isinstance(grad, mat4) or isinstance(grad, np.ndarray) and grad.size == 16:
					grad = grad,
				# ensure the homogeneous factor is 0 for derivatives
				for df in grad:
					df[3][3] = 0
				f = joint.direct(p)
			transforms[joint] = (index, f, grad)
			if joint in self.rev:
				rf = affineInverse(f)
				transforms[self.rev[joint]] = (index, rf, [-rf*df*rf  for df in grad])
				
			index += size
		
		# stack gradients, transformed by the chain successive transformations
		jac = np.zeros((len(self.cycles), index, squeezed_homogeneous), float)
		# jac = scipy.sparse.csr_matrix((len(cycles)*squeezed_homogeneous, self.nvars))
		for icycle, cycle in enumerate(self.cycles):
			grad = []
			positions = []
			# pre transformations
			b = mat4(1)
			for joint in cycle:
				i, f, g = transforms[joint]
				for j, df in enumerate(g):
					grad.append(b*df)
					positions.append(i+j)
				b = b*f
			# post transformations
			b = mat4(1)
			k = len(grad)-1
			for joint in reversed(cycle):
				i, f, g = transforms[joint]
				for df in reversed(g):
					grad[k] = grad[k]*b
					k -= 1
				b = f*b
			
			for ig, g in zip(positions, grad):
				# print('-', icycle, ig)
				jac[icycle, ig] = squeeze_homogeneous(g)
				# jac[icycle*squeezed_homogeneous:(icycle+1)*squeezed_homogeneous, ig] = squeeze_homogeneous(g)
		
		# assert jac.transpose((0,2,1)).shape == (len(cycles), 9, nvars)
		return jac.transpose((0,2,1)).reshape((len(self.cycles)*squeezed_homogeneous, index))
		# return jac
	
	def solve(self, fixed:dict={}, close:list=None, precision=1e-6, maxiter=None) -> ndarray:
		'''
			compute the joint positions for the given fixed solids positions
			
			Args:
				fixed:  list of `mat4` poses of the fixed solids in the same order as given to `__init__`
				close:  
					the joint positions we want the result the closest to. 
					If not provided, `self.default` will be used
				precision:
					the precision of the expected loop closing. kinematic loop transformations ending with a difference above this precision will raise a `KinematicError`
				maxiter: maximum number of iterations allowed to the solver
			
			Return:  the joint positions allowing the kinematic to have the fixed solids in the given poses
			Raise: KinematicError if no joint position can satisfy the fixed positions
			
			Example:
				
				>>> # solve with no constraints
				>>> kinematic.solve()
				[...]
				>>> # solve by fixing solid 4
				>>> kinematic.solve({free: mat4(...)})
				[...]
				>>> # solve by fixing some joints
				>>> kinematic.solve({motor1: radians(90)})
				[...]
				>>> kinematic.solve({motor1: radians(90), motor2: radians(15)})
				[...]
		'''
		
		if close is None:
			close = self.default
		if len(self.cycles) == 0:
			return close
		if isinstance(close, ndarray):
			if close.dtype != self.dtype:
				close = close.view(self.dtype)
		else:
			close = np.asarray([tuple(close)], self.dtype)
		
		if fixed:
			index = [str(i)   for i, joint in enumerate(self.joints)  if joint not in fixed]
			dtype = np.dtype([(str(i), np.dtype(t))  for i,t in enumerate(self.joints)  if joint not in fixed])
			init = np.asarray(close, self.dtype)[index].astype(dtype)
			mins, maxes = self.bounds[0][index].astype(dtype), self.bounds[1][index].astype(dtype)
		else:
			dtype = self.dtype
			init = np.asarray(close, self.dtype)
			mins, maxes = self.bounds
		
		# solve
		# from time import perf_counter as time
		# start = time()
		
		res = scipy.optimize.least_squares(
					lambda x: self.cost_residuals(x.view(dtype), fixed), 
					init.view(float), 
					method = 'trf', 
					bounds = (
						mins.view(float), 
						maxes.view(float),
						), 
					jac = lambda x: self.cost_jacobian(x.view(dtype), fixed), 
					xtol = precision, 
					# ftol = 0,
					# gtol = 0,
					max_nfev = maxiter,
					)
		
# 		print('solved in', time() - start)
# 		print(res)
# 		np.set_printoptions(linewidth=np.inf)
# 		estimated = res.jac
# 		# computed = self.jac(structure_state(res.x))
# 		# print(estimated, (np.abs(estimated) > 1e-3).sum())
# 		# print()
# 		# print(computed, (np.abs(computed) > 1e-3).sum())
# 		
# 		from matplotlib import pyplot as plt
# 		# estimated = estimated.toarray()
# 		# computed = computed.toarray()
# 		plt.imshow(np.stack([
# 			estimated*0, 
# 			# np.abs(estimated)/np.abs(estimated).max(axis=1)[:,None],
# 			# np.abs(computed)/np.abs(estimated).max(axis=1)[:,None],
# 			np.log(np.abs(estimated))/10+0.5,
# 			np.log(np.abs(computed))/10+0.5,
# 			], axis=2))
		# plt.show()
		
		if not res.success:
			raise KinematicError('failed to converge: '+res.message, res)
		if np.any(np.abs(res.fun) > precision):
			raise KinematicError('position out of reach: no solution found for closing the kinematic cycles')
		
		# structure results
		if fixed:
			result = close.copy()
			result[index] = res.x.view(dtype)
			for i, joint in self.joints:
				if joint in fixed:
					result[str(i)] = fixed[joint]
		else:
			result = res.x.view(dtype)
		return result
	
	def parts(self, state, precision=1e-6) -> dict:
		''' return the pose of all solids in the kinematic for the given joints positions 
			The arguments are the same as for `self.direct()`
		'''
		if isinstance(state, dict):
			state = [state.get(joint) or joint.default  for joint in self.joints]
		transforms = {}
		# collect transformations
		for joint, p in zip(self.joints, state):
			transforms[joint] = joint.direct(p)
			if joint in self.rev:
				transforms[self.rev[joint]] = affineInverse(transforms[joint])
		# chain transformations
		poses = {}
		for joint in self.order:
			base = transforms.get(joint.solids[0]) or mat4()
			tip = base * transforms[joint]
			if joint.solids[-1] in poses:
				if np.any(np.abs(poses[joint.solids[-1]] - tip) > precision):
					raise KinematicError('position out of reach: kinematic cycles not closed')
			poses[joint.solids[-1]] = tip
		return poses
		
	def freedom(self, state) -> list:
		''' 
			list of free movement joint directions. the length of the list is the degree of freedom . 
			
			Note:
				When state is a singular position in the kinematic, the degree of freedom is locally smaller or bigger than in other positions
		'''
		free = scipy.linalg.null_space(self.cost_jacobian(state))
		return [structure_state(x, state)  for x in free]
	
	def grad(self, state) -> dict:
		''' 
			return a gradient of the all the solids poses at the given joints position 
			
			Note:
				this function will ignore any degree of freedom of the kinematic that is not defined in `direct_parameters`
		'''
		free = scipy.linalg.null_space(self.cost_jacobian(state)).transpose()
		nprint('free', free)
		# TODO: orient the base of the null space so that they are the closest to the degrees of freedom and in same order
		inverses = sum(flatten_state(p.default, self.dtype).size  for p in self.inverse_parameters)
		directs = sum(flatten_state(p.default, self.dtype).size  for p in self.direct_parameters)
		jac = free[-inverses:] @ free[:directs].T
		return [structure_state([ 
					structure_state(df, inverse.default)
					for inverse in self.inverse_parameters], direct.default)
				for direct, df in zip(self.direct_parameters, jac)]
		
	def direct(self, parameters: list, close=None) -> list:
		''' 
			shorthand to `self.solve(self.direct_parameters)` and computation of desired transformation matrices
			it only works when direct and inverse constraining joints have been set
		'''
		fixed = dict(zip(self.direct_parameters, parameters))
		result = self.solve(fixed, close)
		return [joint.direct(result[i])  for i in range(len(self.inverse_parameters))]
		
	def inverse(self, parameters: list, close=None) -> list:
		''' 
			shorthand to `self.solve(self.inverse_parameters)` and extraction of desired joints
			it only works when direct and inverse constraining joints have been set
		'''
		fixed = dict(zip(self.inverse_parameters, parameters))
		result = self.solve(fixed, close)
		return [result[-i]  for i in range(len(self.direct_parameters))]
	
	def display(self, scene):
		''' display allowing manipulation of kinematic '''
		return KinematicManip(scene, self)

floatsize = np.dtype(float).itemsize
empty = ()

def flatten(structured):
	if hasattr(structured, '__iter__'):
		for x in structured:
			yield from flatten(x)
	else:
		yield structured
			
def flatten_state(structured, dtype):
	return np.array(list(flatten(structured)), dtype)

def structure_state(flat, structure):
	it = iter(flat)
	structured = []
	for ref in structure:
		if hasattr(ref, '__len__'):
			structured.append(structure_state(it, ref))
		else:
			structured.append(next(it))
	return structured

def cycles(conn: '{node: [node]}') -> '[[node]]':
	''' extract a set of any-length cycles decomposing the graph '''
	todo

def shortcycles(conn: '{node: [node]}', costs: '{node: float}', branch=True, tree=None) -> '[[node]]':
	'''
		extract a set of minimal cycles decompsing the graph
		
		.. image:: /schemes/kinematic-cycles.svg
	'''
	# orient the graph in a depth-first way, and search for fusion points
	distances = {}
	merges = []
	tree = {}
	for parent, child in tree or depthfirst(conn):
		if child in distances:
			merges.append((parent, child))
		else:
			tree[child] = parent
			if parent not in tree:
				distances[parent] = 0
			distances[child] = distances.get(parent, 0) + costs.get(child, 0)
	# sort merge points with
	#  - the first distance being the merge point distance to the root node
	#  - the second distance being the secondary branch bigest distance to the root node
	key = lambda edge: (distances[edge[1]], -distances[edge[0]])
	merges = sorted(merges, key=key)
	nprint('tree', tree)
	nprint('distances', distances)
	nprint('merges', merges)
	
	cycles = []
	c = len(merges)
	while c > 0:
		del merges[c:]
		c -= 1
		# collect candidates to next cycle
		# merge node that have the same distance may be concurrent cycles, so they must be sorted out
		while c > 0 and distances[merges[c][1]] == distances[merges[c-1][1]]:
			c -= 1
		# recompute distance to root
		# only the second distance can have changed since the depthfirst tree ensure that the first is already optimal
		for i in range(c, len(merges)):
			parent, child = merges[i]
			node, cost = parent, 0
			while node in tree:
				cost += costs.get(node, 0)
				node = tree[node]
			distances[parent] = cost
		# process all the candidates
		# the second distances cannot swap during this process, so any change will keep the same order
		for parent, child in sorted(merges[c:], key=key):
			# unroll from parent and child each on their own until union
			parenthood, childhood = [parent], [child]
			while parent != child:
				if distances[parent] >= distances[child]:
					parent = tree[parent]
					parenthood.append(parent)
				elif distances[parent] <= distances[child]:
					child = tree[child]
					childhood.append(child)
			# assemble cycle
			cycles.append(childhood[::-1] + parenthood)
			# report simplifications in the graph
			for i in range(1, len(parenthood)):
				parent, child = parenthood[i-1], parenthood[i]
				dist = distances[child] + costs.get(parent, 0)
				if dist >= distances[parent]:		break
				tree[parent] = child
				distances[parent] = dist
	return cycles
			
				
	
def depthfirst(conn: '{node: [node]}', starts=()) -> '[(parent, child)]':
	''' 
		generate a depth-first traversal of the givne graph 
	
		.. image:: /schemes/kinematic-depthfirst.svg
	'''
	edges = set()
	reached = set()
	ordered = []
	for start in itertools.chain(starts, conn):
		if start in reached:
			continue
		front = [(None, start)]
		while front:
			parent, node = front.pop()
			if (parent, node) in edges:
				continue
			reached.add(node)
			if parent is not None:
				edges.add((parent, node))
				edges.add((node, parent))
				ordered.append((parent, node))
			for child in conn[node]:
				if (node, child) not in reached:
					front.append((node, child))
	nprint('depthfirst', ordered)
	return ordered
	
def arcs(conn: '{node: [node]}') -> '[[node]]':
	''' find ars in the given graph '''
	suites = []
	empty = ()
	edges = set()
	reached = set()
	for start in conn:
		if start in reached or len(conn.get(start, empty)) > 2:
			continue
		suite = [start]
		reached.add(start)
		def propagate():
			while True:
				node = suite[-1]
				reached.add(node)
				for child in conn.get(node, empty):
					if (child, node) not in edges:
						suite.append(child)
						edges.add((node, child))
						edges.add((child, node))
						break
				else:
					break
		propagate()
		suite.reverse()
		propagate()
		suites.append(suite)
	return suites


class Screw(object):
	''' A 3D torsor aka Screw aka Wrench aka Twist - is a mathematical object defined as follow:
		  * a resulting vector R
		  * a momentum vector field M

		The momentum is a function of space, satisfying the relationship:
			M(A) = M(B) + cross(R, A-B)
		
		Therefore it is possible to represent a localized torsor such as:
		  * R = resulting
		  * M = momentum vector at position P
		  * P = position at which M takes the current value
		
		Torsor are useful for generalized solid mechanics to handle multiple variables of the same nature:
		  * Force torsor:	
			  Screw(force, torque, pos)
		  * Velocity (aka kinematic) torsor:
			  Screw(rotation, velocity, pos)
		  * Kinetic (inertia) torsor:
			  Screw(linear movement quantity, rotational movement quantity, pos)
			
		  All these torsors makes it possible to represent all these values independently from expression location
		  
		  
		Attributes:
			resulting (vec3): 
			momentum (vec3):
			position (vec3):
	'''
	__slots__ = ('resulting', 'momentum', 'position')
	def __init__(self, resulting=None, momentum=None, position=None):
		self.resulting, self.momentum, self.position = resulting or vec3(0), momentum or vec3(0), position or vec3(0)
	def locate(self, pt) -> 'Screw':
		''' Gets the same torsor, but expressed for an other location '''
		return Screw(self.resulting, self.momentum + cross(self.resulting, pt-self.position), pt)
	
	def transform(self, mat) -> 'Screw':
		''' Changes the torsor from coordinate system '''
		if isinstance(mat, mat4):
			rot, trans = mat3(mat), vec3(mat[3])
		elif isinstance(mat, mat3):
			rot, trans = mat, 0
		elif isinstance(mat, quat):
			rot, trans = mat, 0
		elif isinstance(mat, vec3):
			rot, trans = 1, mat
		else:
			raise TypeError('Screw.transform() expect mat4, mat3 or vec3')
		return Screw(rot*self.resulting, rot*self.momentum, rot*self.position + trans)
	
	def __add__(self, other):
		if other.position != self.position:		other = other.locate(self.position)
		return Screw(self.resulting+other.resulting, self.momentum+other.momentum, self.position)
	
	def __sub__(self, other):
		if other.position != self.position:		other = other.locate(self.position)
		return Screw(self.resulting-other.resulting, self.momentum-other.momentum, self.position)
	
	def __neg__(self):
		return Screw(-self.resulting, -self.momentum, self.position)
	
	def __mul__(self, x):
		return Screw(x*self.resulting, x*self.momentum, self.position)
	
	def __div__(self, x):
		return Screw(self.resulting/x, self.momentum/x, self.position)
		
	def __repr__(self):
		return '{}(\n\t{}, \n\t{}, \n\t{})'.format(self.__class__.__name__, repr(self.resulting), repr(self.momentum), repr(self.position))

def comomentum(t1, t2):
	''' Comomentum of screws:   `dot(M1, R2)  +  dot(M2, R1)`
		
		The result is independent of torsors location
	'''
	t2 = t2.locate(t1.position)
	return dot(t1.momentum, t2.resulting) + dot(t2.momentum, t1.resulting)


class Solid:
	''' Solid for objects display
	
		A Solid is also a way to group objects and move it anywhere without modifying them, as the objects contained in a solid are considered to be in solid local coordinates.
		A Solid is just like a dictionary with a pose.
	
		Attributes:
			orientation (quat):  rotation from local to world space
			position (vec3):     displacement from local to world
			content (dict/list):      objects to display using the solid's pose
			name (str):          optional name to display on the scheme
			
		Example:
			
			>>> mypart = icosphere(vec3(0), 1)
			>>> s = Solid(part=mypart, anything=vec3(0))   # create a solid with whatever inside
			
			>>> s.transform(vec3(1,2,3))   # make a new translated solid, keeping the same content without copy
			
			>>> # put any content in as a dict
			>>> s['part']
			<Mesh ...>
			>>> s['whatever'] = vec3(5,2,1)
	'''
	
	def __init__(self, pose=1, **content):
		self.pose = mat4(pose)
		self.content = content
		
	def __copy__(self):
		return Solid(self.pose, **self.content)
	
	def transform(self, value) -> 'Solid':
		''' Displace the solid by the transformation '''
		return Solid(transform(value) * self.pose, **self.content)
	
	def place(self, *args, **kwargs) -> 'Solid': 
		''' Strictly equivalent to `.transform(placement(...))`, see `placement` for parameters specifications. '''
		return Solid(placement(*args, **kwargs), **self.content)
		
	def loc(self, *args):
		transform = mat4()
		for key in args:
			transform = transform @ obj.pose
		return transform
	
	def deloc(self, *args):
		obj = self
		transform = mat4()
		for key in args:
			transform = transform @ obj.pose
			obj = obj.content[key]
		return obj.transform(transform)
	
	def reloc(self, *args):
		indev
	
	# convenient content access
	def __getitem__(self, key):
		''' Shorthand to `self.content` '''
		return self.content[key]
		
	def __setitem__(self, key, value):
		''' Shorthand to `self.content` '''
		self.content[key] = value
	
	def add(self, value):
		''' Add an item in self.content, a key is automatically created for it and is returned '''
		key = next(i 	for i in range(len(self.content)+1)
						if i not in self.content	)
		self.content[key] = value
		return key
	
	def set(self, **objs):
		''' Convenient method to set many elements in one call.
			Equivalent to `self.content.update(objs)`
		'''
		self.content.update(objs)
		return self
	
	
	class display(rendering.Group):
		''' Movable `Group` for the rendering pipeline '''
		def __init__(self, scene, solid):
			super().__init__(scene, solid.content, local=solid.pose)
		
		def update(self, scene, solid):
			if not isinstance(solid, Solid):	return
			super().update(scene, solid.content)
			self.local = fmat4(solid.pose)
			return True
		
		def stack(self, scene):
			for key,display in self.displays.items():
				if key == 'annotations' and not scene.options['display_annotations'] and not self.selected:
					continue
				for sub,target,priority,func in display.stack(scene):
					yield ((key, *sub), target, priority, func)

from .rendering import Group

@dataclass
class scale_solid:
	''' scheme space scaling around a point in a given solid '''
	solid: object
	center: fvec3
	size: float
	pose: fmat4 = fmat4()
	
	def __call__(self, view):
		m = view.uniforms['view'] * view.uniforms['world'] * self.pose
		e = view.uniforms['proj'] * fvec4(1,1,(m*self.center).z,1)
		e /= e[3]
		return m * translate(self.center) * scale(fvec3(min(self.size, 2 / (e[1]*view.target.height))))

@dataclass
class world_solid:
	solid: object
	pose: fmat4 = fmat4()
	
	def __call__(self, view):
		return view.uniforms['view'] * view.uniforms['world'] * self.pose

class ChainManip(Group):
	''' object to display and interact with a robot in the 3d view

		Attributes:
		
			robot:      the robot to get joint positions from
			kinematic:  kinematic model of the robot. at contrary to `RobotDisplay`, it MUST be a chain kinematic
			toolcenter (vec3):   the initial tool rotation point
			joints:   True if each solid has a dedicated joint, then the joint menipulation mode is available
			options (QManipOptions):   the helper widget setting the modes and so on
	'''
	min_colinearity = 1e-2
	max_increment = 0.3
	prec = 1e-6
	
	def __init__(self, scene, chain, pose=None, toolcenter=None):
		super().__init__(scene)
		self.chain = chain
		self.toolcenter = chain.joints[-1].position[1]
		self.pose = pose or chain.default
		self.parts = self.chain.parts(self.pose)
		
		from .rendering import displayable
		if chain.content:
			for key, solid in enumerate(chain.content):
				if displayable(solid):
					self.displays[key] = scene.display(solid)
		
		self.displays['scheme'] = scene.display(
					kinematic_toolcenter(self.toolcenter) 
					+ kinematic_scheme(chain.joints))
		
	# TODO: add content update method
	
	def stack(self, scene):
		''' rendering stack requested by the madcad rendering system '''
		yield ((), 'screen', -1, self.place_solids)
		yield from super().stack(scene)

	def place_solids(self, view):
		world = self.world * self.local
		
		index = {}
		for i, joint in enumerate(self.chain.joints):
			solid = joint.solids[-1]
			index[solid] = i+1
			if display := self.displays.get(solid):
				display.word = word * fmat4(self.parts[i])
		
		for space in self.displays['scheme'].spacegens:
			if isinstance(space, (world_solid, scale_solid)) and space.solid in index:
				space.pose = world * fmat4(self.parts[index[space.solid]])
			elif isinstance(space, scheme.halo_screen):
				if view.scene.options['kinematic_manipulation'] == 'rotate':
					space.position = fvec3(self.parts[-1] * self.toolcenter)
				else:
					space.position = fvec3(nan)

	def control(self, view, key, sub, evt):
		''' user event manager, optional part of the madcad rendering system '''
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
			evt.accept()
			
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.RightButton:
			evt.accept()
			self.panel.setParent(view)
			self.panel.move(evt.pos())
			self.panel.show()
		
		if evt.type() == QEvent.MouseMove and evt.buttons() & Qt.LeftButton:
			evt.accept()
			# put the tool into the view, to handle events
			# TODO: allow changing mode during move
			if sub == ('scheme', 0):
				view.tool.append(rendering.Tool(self.move_tool, view, sub, evt))
			else:
				view.tool.append(rendering.Tool(getattr(self, 'move_'+view.scene.options['kinematic_manipulation']), view, sub, evt))
				
	def move_tool(self, dispatcher, view, sub, evt):
		place = mat4(self.world) * self.parts[-1]
		init = place * vec3(self.toolcenter)
		offset = init - view.ptfrom(evt.pos(), init)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break
			
			self.toolcenter = affineInverse(place) * (view.ptfrom(evt.pos(), init) + offset)
	
	def move_joint(self, dispatcher, view, sub, evt):
		
		# find the clicked solid, and joint controled
		if sub[0] == 'scheme':
			solid = (sub[1]-1)//2
		else:
			solid = sub[0]
		
		# TODO: support both using the joint or the chain methods
		# get 3d location of mouse
		click = view.ptat(view.somenear(evt.pos()))
		joint = max(0, solid-1)
		anchor = affineInverse(mat4(self.world * self.local) * self.parts[joint+1]) * vec4(click,1)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local) * self.parts[joint]
				direct = affineInverse(self.parts[joint]) * self.parts[joint+1]
				target = qtpos(evt.pos(), view)
				current = model * (direct * anchor)
				move = target - current.xy / current.w

				# solid move directions that can be acheived with this joint
				jac = self.chain.joints[joint].grad(self.pose[joint])
				# combination of the closest directions to the mouse position
				# several degrees of freedom
				if hasattr(self.pose[joint], '__len__'):
					# gradient of the screen position
					jac = np.stack([
						(model * (grad * anchor)).xy / current.w
						for grad in jac])
					# colinearity between the desired move and the gradient directions. it avoids the mechanisme to burst when close to a singularity and also when the mouse move is big
					colinearity = min(1, sum(
								dot(grad, move)**2 / (length2(grad) + length2(move) + self.prec)
								for grad in jac) / self.min_colinearity)
					increment = np.linalg.lstsq(jac.transpose(), move * colinearity)
				# faster equivalent computation in case of one degree of freedom
				else:
					grad = (model * (jac * anchor)).xy / current.w
					colinearity = min(1, 1e2 * dot(grad, move)**2 / (length2(grad) + length2(move) + self.prec))
					increment = dot(grad, move * colinearity) / length2(grad)
				
				self.pose[joint] += clamp(increment, -self.max_increment, self.max_increment)
				self.parts = self.chain.parts(self.pose)

	def move_translate(self, dispatcher, view, sub, evt):
		# translate the tool
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[-1]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[-1]
				jac = self.chain.grad(self.pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_anchor = model * solid * anchor
				target_anchor = qtpos(evt.pos(), view)
				
				move = np.concatenate([
					target_anchor - current_anchor.xy / current_anchor.w,
					np.ravel(mat3(init_solid - solid)),
					])
				jac = np.stack([
					np.concatenate([
						(model * grad * anchor).xy / current_anchor.w, 
						np.ravel(mat3(grad)), 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = np.linalg.solve(jac @ jac.transpose() + np.eye(4)*self.prec, jac @ (move * colinearity))
				# increment = np.linalg.pinv(jac @ jac.transpose()) @ (jac @ (move * colinearity))
				
				self.pose += increment.clip(-self.max_increment, self.max_increment)
				self.parts = self.chain.parts(self.pose)

	def move_rotate(self, dispatcher, view, sub, evt):
		# translate the tool
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[-1]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		tool = vec4(self.toolcenter,1)
		init_tool = solid * tool
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[-1]
				jac = self.chain.grad(self.pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_tool = model * solid * tool
				target_anchor = qtpos(evt.pos(), view)
				target_tool = model * init_tool
				
				move = np.concatenate([
					(init_tool - solid * tool).xyz,
					(target_anchor - target_tool.xy/target_tool.w) 
						- (model * solid * normalize(anchor - tool)).xy / current_tool.w,
					])
				jac = np.stack([
					np.concatenate([
						(grad * tool).xyz, 
						(model * grad * normalize(anchor - tool)).xy / current_tool.w, 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = np.linalg.solve(jac @ jac.transpose() + np.eye(4)*self.prec, jac @ (move * colinearity))
				# increment = np.linalg.pinv(jac @ jac.transpose()) @ (jac @ (move * colinearity))
				
				self.pose += increment.clip(-self.max_increment, self.max_increment)
				self.parts = self.chain.parts(self.pose)


def qtpos(qtpos, view):
	''' convert qt position in the widget to opengl screen coords in range (-1, 1) '''
	return vec2(
		+ (qtpos.x()/view.width() *2 -1),
		- (qtpos.y()/view.height() *2 -1),
		)

def normsq(x):
	return (x*x).sum()
		
class KinematicManip(Group):
	''' Display that holds a kinematic structure and allows the user to move it
	'''
	
	def __init__(self, scene, kinematic, pose=None):
		super().__init__(scene)
		self.kinematic = kinematic
		self.toolcenter = ...
		self.pose = self.kinematic.solve(close=pose or self.kinematic.default)
		self.parts = self.kinematic.parts(self.pose)
		
		from .rendering import displayable
		if self.kinematic.content:
			for key, solid in self.kinematic.content.items():
				if displayable(solid):
					self.displays[key] = scene.display(solid)
		
		self.displays['scheme'] = scene.display(kinematic_scheme(kinematic.joints))
		
	def stack(self, scene):
		''' rendering stack requested by the madcad rendering system '''
		yield ((), 'screen', -1, self.place_solids)
		yield from super().stack(scene)

	def place_solids(self, view):
		world = self.world * self.local
		
		for key in self.displays:
			if key in self.parts:
				self.displays[key].world = world * self.parts[key]
		
		for space in self.displays['scheme'].spacegens:
			if isinstance(space, (world_solid, scale_solid)) and space.solid in self.parts:
				space.pose = world * fmat4(self.parts[space.solid])

	def control(self, view, key, sub, evt):
		''' user event manager, optional part of the madcad rendering system '''
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
			evt.accept()
			
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.RightButton:
			evt.accept()
			self.panel.setParent(view)
			self.panel.move(evt.pos())
			self.panel.show()
		
		if evt.type() == QEvent.MouseMove and evt.buttons() & Qt.LeftButton:
			evt.accept()
			# put the tool into the view, to handle events
			# TODO: allow changing mode during move
			if sub == ('scheme', 0):
				view.tool.append(rendering.Tool(self.move_tool, view, sub, evt))
			else:
				view.tool.append(rendering.Tool(getattr(self, 'move_'+view.scene.options['kinematic_manipulation']), view, sub, evt))
	
	def move_translate(self, dispatcher, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = sub[1]//2
		else:                   moved = sub[0]
		# define the kinematic problem in term of that solid
		kinematic = Kinematic(
			[], 
			ground = self.kinematic.ground, 
			direct = self.kinematic.joints,
			inverse = [Free((self.kinematic.ground, moved))],
			)
		pose = (*self.pose, kinematic.joints[-1].inverse(self.parts[moved]))
		# constants during translation
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[moved]
				jac = kinematic.grad(pose)
				nprint('jac', jac)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_anchor = model * solid * anchor
				target_anchor = qtpos(evt.pos(), view)
				
				move = np.concatenate([
					target_anchor - current_anchor.xy / current_anchor.w,
					np.ravel(mat3(init_solid - solid)),
					])
				jac = np.stack([
					np.concatenate([
						(model * grad * anchor).xy / current_anchor.w, 
						np.ravel(mat3(grad)), 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = np.linalg.solve(jac @ jac.transpose() + np.eye(4)*self.prec, jac @ (move * colinearity))
				
				pose = self.kinematic.solve(close=pose + increment.clip(-self.max_increment, self.max_increment))
				self.pose = pose[:len(self.pose)]
				self.parts = self.chain.parts(self.pose)
				
	def move_joint(self, dispatcher, view, sub, evt):
		# define the kinematic problem in term of that solid
		# select the free direction the closest to a pure one-joint movement
		directions = self.kinematic.freedom()
		increment = ...
		self.pose = self.kinematic.solve(close=self.pose + increment)

	
def kinematic_toolcenter(toolcenter):
	''' create a scheme for drawing the toolcenter in kinematic manipulation '''
	from .generation import revolution
	from .mesh import web
	from .scheme import Scheme, halo_screen
	
	color = settings.display['annotation_color']
	angle = 2
	radius = 60
	size = 3
	tend = -sin(angle)*X + cos(angle)*Y
	rend = cos(angle)*X + sin(angle)*Y
	sch = Scheme(
		space=halo_screen(fvec3(toolcenter)), 
		layer=1e-4,
		shader='line',
		track=0,
		)
	sch.add([size*cos(t)*X + size*sin(t)*Y   for t in linrange(0, 2*pi, step=0.2)], color=fvec4(color,1))
	sch.add([radius*cos(t)*X + radius*sin(t)*Y  for t in linrange(0, angle, step=0.1)], color=fvec4(color,1))
	sch.add([radius*cos(t)*X + radius*sin(t)*Y  for t in linrange(angle, 2*pi, step=0.1)], color=fvec4(color,0.2))
	sch.add(
		revolution(2*pi, (radius*rend, tend), web([radius*rend, radius*rend-14*tend-3.6*rend]), resolution=('div', 8)), 
		shader='fill',
		color=fvec4(color,0.8),
		)
	return sch

def kinematic_scheme(joints) -> 'Scheme':
	''' create a kinematic scheme for the given joints '''
	from .scheme import Scheme
	
	centers = {}
	for joint in joints:
		for solid, position in zip(joint.solids, joint.position):
			centers[solid] = centers.get(solid, 0) + vec4(position, 1)
	for solid, center in centers.items():
		centers[solid] = center.xyz / center.w
	
	scheme = Scheme()
	for joint in joints:
		size = max(
			distance(position, centers[solid])
			for solid, position in zip(joint.solids, joint.position))
		scheme += joint.scheme(size, centers[joint.solids[0]], centers[joint.solids[-1]])
		
	return scheme
		

def placement(*pairs, precision=1e-3):
	''' Return a transformation matrix that solved the placement constraints given by the surface pairs
	
		Parameters:
		
			pairs:	a list of pairs to convert to kinematic joints
					
					- items can be couples of surfaces to convert to joints using `guessjoint`
					- tuples (joint_type, a, b)  to build joints `joint_type(solida, solidb, a, b)`
			
			precision: surface guessing and kinematic solving precision (distance)
		
		Each pair define a joint between the two assumed solids (a solid for the left members of the pairs, and a solid for the right members of the pairs). Placement will return the pose of the first relatively to the second, satisfying the constraints.
		
		Example:
		
			>>> # get the transformation for the pose
			>>> pose = placement(
			...		(screw['part'].group(0), other['part'].group(44)),  # two cylinder surfaces: Gliding joint
			...		(screw['part'].group(4), other['part'].group(25)),    # two planar surfaces: Planar joint
			...		)  # solve everything to get solid's pose
			>>> # apply the transformation to the solid
			>>> screw.pose = pose
			
			>>> # or
			>>> screw.place(
			...		(screw['part'].group(0), other['part'].group(44)),
			...		(screw['part'].group(4), other['part'].group(25)),
			...		)
			
			>>> screw.place(
			...		(Pivot, screw['axis'], other['screw_place']),
			...		)
	'''
	from .reverse import guessjoint
	
	joints = []
	for pair in pairs:
		if len(pair) == 2:		joints.append(guessjoint((0, 1), *pair, precision*0.25))
		elif len(pair) == 3:	joints.append(pair[0]((0, 1), *pair[1:]))
		else:
			raise TypeError('incorrect pair definition', pair)
	
	if len(joints) > 1:
		kin = Kinematic(joints)
		parts = kin.parts(kin.solve())
		return affineInverse(parts[0]) * parts[1]
	else:
		return joints[0].direct(joints[0].default)
	
def convexhull(pts):
	import scipy.spatial
	if len(pts) == 3:
		return Mesh(pts, [(0,1,2),(0,2,1)])
	elif len(pts) > 3:
		hull = scipy.spatial.ConvexHull(typedlist_to_numpy(pts, 'f8'))
		m = Mesh(pts, hull.simplices.tolist())
		return m
	else:
		return Mesh(pts)

			
def extract_used(obj):
	if isinstance(obj, Mesh):	links = obj.faces
	elif isinstance(obj, Web):	links = obj.edges
	elif isinstance(obj, Wire):	links = [obj.indices]
	else:
		raise TypeError('obj must be a mesh of any kind')
	
	return striplist(obj.points[:], links)[0]

	
def explode_offsets(solids) -> '[(solid_index, parent_index, offset, barycenter)]':
	''' Build a graph of connected objects, ready to create an exploded view or any assembly animation.
		See `explode()` for an example. The exploded view is computed using the meshes contained in the given solids, so make sure there everything you want in their content.
	
		Complexity is `O(m * n)` where m = total number of points in all meshes, n = number of solids
		
		NOTE:
			
			Despite the hope that this function will be helpful, it's (for computational cost reasons) not a perfect algorithm for complex assemblies (the example above is at the limit of a simple one). The current algorithm will work fine for any simple enough assembly but may return unexpected results for more complex ones.
		
	'''
	import scipy.spatial.qhull
	# build convex hulls
	points = [[] for s in solids]
	# recursively search for meshes in solids
	def process(i, solid):
		if hasattr(solid.content, 'values'):	it = solid.content.values()
		else:									it = solid.content
		for obj in solid.content.values():
			if isinstance(obj, Solid):
				process(i, obj)
			elif isinstance(obj, (Mesh,Web,Wire)):
				try:
					points[i].extend(extract_used(convexhull(extract_used(obj)).transform(solid.pose)))
				except scipy.spatial.qhull.QhullError:
					continue
			
	for i,solid in enumerate(solids):
		process(i,solid)
	
	# create convex hulls and prepare for parenting
	hulls = [convexhull(pts).orient()  for pts in points]
	boxes = [hull.box()  for hull in hulls]
	normals = [hull.vertexnormals()  for hull in hulls]
	barycenters = [hull.barycenter()  for hull in hulls]
	
	scores = [inf] * len(solids)
	parents = [None] * len(solids)
	offsets = [vec3(0)] * len(solids)
	
	# build a graph of connected things (distance from center to convex hulls)
	for i in range(len(solids)):
		center = barycenters[i]	
		for j in range(len(solids)):
			if i == j:
				continue
			
			# case of non-connection, the link won't appear in the graph
			if boxes[i].intersection(boxes[j]).isempty():
				continue
			
			# the parent is always the biggest of the two, this also breaks any possible parenting cycle
			if length2(boxes[i].width) > length2(boxes[j].width):
				continue
			
			# select the shortest link
			d, prim = distance2_pm(center, hulls[j])
			
			if d < scores[i]:
				# take j as new parent for i
				scores[i] = d
				parents[i] = j
				
				# get the associated displacement vector
				pts = hulls[j].points
				if isinstance(prim, int):
					normal = normals[j][prim]
					offsets[i] = center - pts[prim]
				elif len(prim) == 2:
					normal = normals[j][prim[0]] + normals[j][prim[1]]
					offsets[i] = noproject(center - pts[prim[0]],  
										pts[prim[0]]-pts[prim[1]])
				elif len(prim) == 3:
					normal = cross(pts[prim[1]]-pts[prim[0]], pts[prim[2]]-pts[prim[0]])
					offsets[i] = project(center - pts[prim[0]],  normal)
				else:
					raise AssertionError('prim should be an index for  point, face, triangle')
				if dot(offsets[i], normal) < 0:
					offsets[i] = -offsets[i]
	
	# resolve dependencies to output the offsets in the resolution order
	order = []
	reached = [False] * len(solids)
	i = 0
	while i < len(solids):
		if not reached[i]:
			j = i
			chain = []
			while not (j is None or reached[j]):
				reached[j] = True
				chain.append(j)
				j = parents[j]
			order.extend(reversed(chain))
		i += 1
	
	# move more parents that have children on their way out				
	blob = [deepcopy(box) 	for box in boxes]
	for i in reversed(range(len(solids))):
		j = parents[i]
		if j and length2(offsets[i]):
			offsets[i] *= (1 
							+ 0.5* length(blob[i].width) / length(offsets[i]) 
							- dot(blob[i].center - barycenters[i], offsets[i]) / length2(offsets[i])
							)
			blob[j].union_update(blob[i].transform(offsets[i]))
								
	return [(i, parents[i], offsets[i], barycenters[i])  for i in order]
			
	
def explode(solids, factor=1, offsets=None) -> '(solids:list, graph:Mesh)':
	''' Move the given solids away from each other in the way of an exploded view.
		It makes easier to seen the details of an assembly . See `explode_offsets` for the algorithm.
		
		Parameters:
			
			solids:		a list of solids (copies of each will be made before displacing)
			factor:		displacement factor, 0 for no displacement, 1 for normal displacement
			offsets:	if given, must be the result of `explode_offsets(solids)`
		
		Example:
		
			>>> # pick some raw model and separate parts
			>>> imported = read(folder+'/some_assembly.stl')
			>>> imported.mergeclose()
			>>> parts = []
			>>> for part in imported.islands():
			...     part.strippoints()
			...     parts.append(Solid(part=segmentation(part)))
			... 
			>>> # explode the assembly to look into it
			>>> exploded = explode(parts)
		
	'''
	solids = [copy(solid)  for solid in solids]
	if not offsets:
		offsets = explode_offsets(solids)
	
	graph = Web(groups=[None])
	shifts = [	(solids[solid].position - solids[parent].position)
				if parent else vec3(0)
				for solid, parent, offset, center in offsets]
	for solid, parent, offset, center in offsets:
		if parent:
			solids[solid].position = solids[parent].position + shifts[solid] + offset * factor
			
			graph.edges.append((len(graph.points), len(graph.points)+1))
			graph.tracks.append(0)
			graph.points.append(solids[parent].position + shifts[solid] + center)
			graph.points.append(solids[solid].position + center)
			
	return [solids, graph]

