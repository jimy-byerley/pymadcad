# This file is part of pymadcad,  distributed under license LGPL v3
__all__ = [
	'Kinematic', 'Joint', 'Weld', 'Free', 'Reverse', 'Chain',
	'flatten_state', 'structure_state',
	'cycles', 'shortcycles', 'depthfirst', 'arcs',
	'KinematicError',
	]

from copy import copy, deepcopy
import itertools
import numpy as np
import numpy.linalg as la
import scipy

from ..mathutils import *
from .. import nprint

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
	
	def normalize(self, state) -> 'params':
		'''  make the given joint coordinates consistent. For most joint is is a no-op or just coordinates clipping
		
			for complex joints coordinates containing quaternions or directions or so, it may need to perform normalizations or orthogonalizations or so on.
			This operation may be implemented in-place or not.
		'''
		return state
	
	def direct(self, state) -> 'mat4':
		''' direct kinematic computation
		
			Parameters:
				state:	
				
					the parameters defining the joint state
					It can be any type accepted by `numpy.array`
				
			Returns:
				the transfer matrix from solids `self.stop` to `self.start`
		'''
		raise NotImplemented
	
	def inverse(self, matrix, close=None, precision=1e-6, maxiter=None, strict=0.) -> list:
		''' inverse kinematic computation
		
			the default implementation is using a newton method to nullify the error between the given and acheived matrices
			
			Parameters:
				matrix:	the transfer matrix from solids `self.stop` to `self.start` we want the parameters for
				close:  
				
					a know solution we want the result the closest to.
					if not specified, it defaults to `self.default`
			
				precision: 
					desired error tolerance on the given matrix, this function returns once reached
				strict:  
					expected error tolerance on the given matrix, if not reached after `maxiter` iterations, this function will raise `KinematicError`.
					Leave it to 0 to raise when `precision` has not been reached
				maxiter:  
					maximum number of iterations allowed, or None if not limit
			
			Returns:
				the joint parameters so that `self.direct(self.inverse(x)) == x` (modulo numerical precision)
		'''
		if close is None:
			close = self.default
		max_increment = 0.3
		
		state = close
		k = 0
		while True:
			k += 1
			if maxiter and k > maxiter and not strict:
				break
			
			move = matrix - self.direct(state)
			error = np.abs(move).max()
			if error <= precision:
				break
			elif maxiter and k > maxiter:
				if error <= strict:
					break
				else:
					raise KinematicError('convergence failed after maximum iterations')
			
			# newton method with a pseudo inverse
			jac = self.grad(state)
			increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ move)
			# apply correction to pose
			state = self.normalize(structure_state(
							flatten_state(state)
							+ increment * min(1, max_increment / np.abs(increment).max()), 
							state))
		return state
	
	def grad(self, state, delta=1e-6) -> '[mat4]':
		''' compute the gradient of the direct kinematic 
		
			The default implementation is using a finite differentiation
		
			Parameters:
				state:	anything accepted by `self.direct()` including singularities
				delta:	finite differentiation interval
			
			Returns:
				a list of the matrix derivatives of `self.direct()`, one each parameter
		'''
		grad = []
		base = self.direct(state)
		for i in range(len(state)):
			grad.append(partial_difference(self.direct, state, base, i, delta))
		return grad
	
	def transmit(self, force: 'Screw', state=None, velocity=None) -> 'Screw':
		''' compute the force transmited by the kinematic chain in free of its moves
			
			The default implementation uses the direct kinematic gradient to compute the moving directions of the chain
			
			Parameters:
				force:	force sent by `self.start` in its coordinate system
				
				state:  
					
					the joint position in which the joint is at the force transmision instant.
					If not specified it defaults to `self.default` (many joints transmit the same regardless of their position)
				
				velocity:  
				
					current derivative of `state` at the transmision instant
					Perfect joints are transmiting the same regardless of their velocity
					
			Returns:	
				force received by `self.stop` in its coordinate system
		'''
		if not state:	state = self.default
		grad = self.grad(state)
		indev
	
	def scheme(self, size: float, junc: vec3=None) -> '[Scheme]':
		''' generate the scheme elements to render the joint '''
		raise NotImplemented
	
	def display(self, scene):
		''' display showing the schematics of this joint, without interaction '''
		display = scene.display(self.scheme(dict(zip(self.solids, range(2))), inf, None, None))
		display.spacegens[0].pose = fmat4()
		display.spacegens[1].pose = fmat4(self.direct(self.default))
		return display


def partial_difference(f, x, fx, i, d):
	p = copy(x)
	try:
		p[i] = x[i] + d
		return (f(p) - fx) / d
	except KinematicError:
		try:
			p[i] = x[i] - d
			return (fx - f(p)) / d
		except KinematicError:
			pass
	raise ValueError('cannot compute below or above parameter {} given value'.format(i))

squeezed_homogeneous = 9
_squeeze = np.array([0,1,2,  5,6, 10,  12,13,14])
def squeeze_homogeneous(m):
	return np.asarray(m, order='F').ravel('F')[_squeeze]
def unsqueeze_homogeneous(m):
	return mat4(m[0], m[1], m[2], 0,  
			-m[1], m[3], m[4], 0,  
			-m[2],-m[4], m[5], 0,  
				m[6], m[7], m[8], 1)


class Weld(Joint):
	''' 
		joint with no degree of freedom,
		simply welding a solid to an other with a transformation matrix to place one relatively to the other 
		
		It is useful to fix solids between each other without actually making it the same solid in a kinematic.
	'''
	bounds = ((), ())
	default = ()
	
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
		
		it is useful to control the explicit relative pose of solids in a kinematic.
	'''
	def __init__(self, solids):
		self.solids = solids
		
	bounds = (
		np.array([-2]*4 + [-inf]*3, float), 
		np.array([+2]*4 + [+inf]*3, float), 
		)
	default = np.array([1,0,0,0,   0,0,0], float)
			
	def normalize(self, parameters):
		return np.concatenate([normalize(quat(parameters[:4])), parameters[4:]])
	
	def direct(self, parameters):
		m = mat4(quat(parameters[:4]))
		m[3] = vec4(parameters[4:],1)
		return m
		
	def inverse(self, matrix, close=None):
		return np.concatenate([quat(matrix), matrix[3].xyz])
		
	def grad(self, parameters):
		a,b,c,d,*_ = parameters
		return (
			# derivatives of quaternion based rotation matrix (column major of course)
			mat4( 
				 2*a,  2*d, -2*c, 0,
				-2*d,  2*a,  2*b, 0,
				 2*c, -2*b,  2*a, 0,
				 0,   0,     0,   0,
				),
			mat4(
				 2*b,  2*c,  2*d, 0,
				 2*c, -2*b,  2*a, 0,
				 2*d, -2*a, -2*b, 0,
				 0,    0,    0,   0,
				),
			mat4(
				-2*c,  2*b, -2*a, 0,
				 2*b,  2*c,  2*d, 0,
				 2*a,  2*d, -2*c, 0,
				 0,    0,    0,   0,
				),
			mat4(
				-2*d,  2*a,  2*b, 0,
				-2*a, -2*d,  2*c, 0,
				 2*b,  2*c,  2*d, 0,
				 0,    0,    0,   0,
				),
			# derivatives of translation (column major of course)
			mat4(
				 0,    0,    0,   0,
				 0,    0,    0,   0,
				 0,    0,    0,   0,
				 1,    0,    0,   0,
				),
			mat4(
				 0,    0,    0,   0,
				 0,    0,    0,   0,
				 0,    0,    0,   0,
				 0,    1,    0,   0,
				),
			mat4(
				 0,    0,    0,   0,
				 0,    0,    0,   0,
				 0,    0,    0,   0,
				 0,    0,    1,   0,
				),
			)
	
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
		if hasattr(self.joint, 'position'):
			self.position = self.joint.position
		
	@property
	def default(self):
		return self.joint.default
		
	@property
	def bounds(self):
		return self.joint.bounds
		
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
		
	def scheme(self, index, maxsize, attach_start, attach_end):
		return self.joint.scheme(index, maxsize, attach_end, attach_start)





class Chain(Joint):
	''' 
		Kinematic chain, This chain of joints acts like one only joint
		The new formed joint has as many degrees of freedom as its enclosing joints.
		
		.. image:: /schemes/kinematic-chain.svg
		
		This class is often used instead of `Kinematic` when possible, because having more efficient `inverse()` and `direct()` methods dedicated to kinematics with one only cycle. It also has simpler in/out parameters since a chain has only two ends where a random kinematic may have many
		
		A `Chain` doesn't tolerate modifications of the type of its joints once instanciated. a joint placement can be modified as long as it doesn't change its hash.
		
		Attributes:
			joints (list):  
				joints in the chaining order
				it must satisfy `joints[i].solids[-1] == joints[i+1].solids[0]`
			content (list):
				displayables matching solids, its length must be `len(joints)+1`
			
			solids (tuple):  the (start,end) joints of the chain, as defined in `Joint`
			default:  the default joints positions
			bounds:   the (min,max) joints positions
	'''
	def __init__(self, joints, content:list=None, default=None):
		if not all(joints[i-1].solids[-1] == joints[i].solids[0]  
				for i in range(1, len(joints))):
			raise ValueError('joints do not form a direct chain, joints are not ordered or badly oriented')
		self.joints = joints
		self.content = content
		self.solids = (joints[0].solids[0], joints[-1].solids[-1])
		self.default = default or [joint.default  for joint in self.joints]
		self.bounds = (
			[joint.bounds[0]  for joint in self.joints],
			[joint.bounds[1]  for joint in self.joints],
			)
		
	def normalize(self, state) -> list:
		''' inplace normalize the joint coordinates in the given state '''
		for i, joint in enumerate(self.joints):
			state[i] = joint.normalize(state[i])
		return state
	
	def direct(self, state) -> mat4:
		b = mat4()
		for x, joint in zip(state, self.joints):
			b *= joint.direct(x)
		return b
	
	def grad(self, state) -> list:
		''' the jacobian of the flattened parameters list '''
		# built the left side of the gradient product of the direct joint
		directs = []
		grad = []
		b = mat4(1)
		for x, joint in zip(state, self.joints):
			f = joint.direct(x)
			g = regularize_grad(joint.grad(x))
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
	
	def parts(self, state) -> list:
		''' return the pose of each solid in the chain '''
		solids = [mat4()] * (len(self.joints)+1)
		for i in range(len(self.joints)):
			solids[i+1] = solids[i] * self.joints[i].direct(state[i])
		return solids
		
	def to_kinematic(self) -> 'Kinematic':
		return Kinematic(inputs=self.joints, outputs=[self.solids[-1]], ground=self.solids[0])
		
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
		''' display allowing manipulation of the chain '''
		from .displays import ChainManip
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
			>>> motor1 = Revolute((0,2), Axis(...))
			>>> motor2 = Revolute((0,7), Axis(...))
			>>> free = Free((7,4))
			>>> # the kinematic solver object
			>>> kinematic = Kinematic([
			... 	Revolute((0,1), Axis(...)),
			... 	motor1,
			... 	motor2,
			... 	Planar((7,3), ...),
			... 	Cylindrical((1,3), ...),
			... 	Ball((3,2), ...),
			... 	Prismatic((4,3), ...),
			... 	Planar((1,5), ...),
			... 	Planar((1,6), ...),
			... 	Weld((7,5), mat4(...)),
			... 	Weld((7,6), mat4(...)),
			... 	free,
			... 	], ground=7)
			
			defines a kinematic with the following graph
		
			.. image::
				/schemes/kinematic-kinematic.svg
				
			one can also define a kinematic with notions of direct and inverse transformations.
			The notion of direct and inverse is based on an input/output relation that we define as such: 
			
			- inputs is selection of joint coordinates
			- outputs is a selection of solids poses
			
			>>> kinematic = Kinematic([
			... 	Revolute((0,1), Axis(...)),
			... 	Planar((7,3), ...),
			... 	Cylindrical((1,3), ...),
			... 	Ball((3,2), ...),
			... 	Prismatic((4,3), ...),
			... 	Planar((1,5), ...),
			... 	Planar((1,6), ...),
			... 	Weld((7,5), mat4(...)),
			... 	Weld((7,6), mat4(...)),
			...	],
			...	ground = 7,
			...	inputs = [motor1, motor2],
			...	outputs = [4,5,6],
			...	)
		
		.. tip::
			If your kinematic is a chain of joints, then prefer using `Chain` to reduce the overhead of the genericity.
		
		.. note::
			A `Kinematic` doesn't tolerate modifications of the type of its joints once instanciated. joints positions could eventually be modified at the moment it doesn't affect the hash of the joints (See `Joint`)
		
		Attributes:
			joints: 
				
				a list of `Joint`, defining the kinematic
				these joints could for a connex graph or not, with any number of cycles
				
				each joint is a link between 2 solids, which are represented by a hashable object (it is common to designate these solids by integers, strings, or objects hashable by their id)
				
			content:
				display object for each solid, this can be anything implementing the display protocol, and will be used only when this kinematic is displayed
				
			ground: the reference solid, all other solids positions will be relative to it
			
			inputs:   a list of joints to fix when calling `direct()`
			outputs:  a list of solids to fix when calling `inverse()`
			
			default:  the default joint pose of the kinematic
			bounds:   a tuple of (min, max) joint poses
	'''
	def __init__(self, joints:list=[], content:dict=None, ground=None, inputs=None, outputs=None):
		if ground is None:
			if joints:	ground = joints[0].solids[0]
			if inputs:  ground = inputs[0].solids[0]
		if (inputs is None) ^ (outputs is None):
			raise TypeError("inputs and outputs must be both provided or undefined")
		elif inputs and outputs:	
			outputs = [Free((ground, out))  for out in outputs]
			joints = inputs + joints + outputs
			self.inputs = inputs
			self.outputs = outputs
		
		self.content = content
		self.joints = joints
		self.ground = ground
		self.default = [joint.default  for joint in self.joints]
		self.bounds = (
			[joint.bounds[0]  for joint in self.joints],
			[joint.bounds[1]  for joint in self.joints],
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
		vars = {joint: flatten_state(joint.default).size
			for joint in conn  
			if isinstance(joint, Joint)}
		
		self.dim = sum(vars.values())
		if inputs and outputs:
			self.inputs_dim = sum(vars[joint]  for joint in self.inputs)
			self.outputs_dim = sum(vars[joint]  for joint in self.outputs)
		
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
						self.rev[joint] = joint = Reverse(joint)
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
			
	def cycles(self) -> list:
		'''
			return a list of minimal cycles decomposing the gkinematic graph
			
			Example:
			
				>>> len(kinematic.cycles())
				5
		'''
		return self.cycles[:]
	
	def to_chain(self) -> 'Chain':
		if self.cycles:
			raise ValueError("this kinematic has cycles and do not form a chain")
		return Chain(self.cycles[0])
	
	def cost_residuals(self, state, fixed=()):
		'''
			build residuals to minimize to satisfy the joint constraints
			cost function returns residuals, the solver will optimize the sum of their squares
		'''
		# collect transformations
		state = iter(state)
		transforms = {}
		for joint in self.joints:
			if joint in fixed:
				transforms[joint] = joint.direct(fixed[joint])
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
		
	def cost_jacobian(self, state, fixed=()):
		''' jacobian of the residuals function '''
		# collect gradient and transformations
		state = iter(state)
		transforms = {}
		index = 0
		for joint in self.joints:
			if joint in fixed:
				grad = ()
				f = joint.direct(fixed[joint])
				size = 0
			else:
				p = next(state, empty)
				size = np.array(p).size
				grad = regularize_grad(joint.grad(p))
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
	
	def solve(self, fixed:dict={}, close:list=None, precision=1e-6, maxiter=None, strict=0.) -> list:
		'''
			compute the joint positions for the given fixed solids positions
			
			Args:
				fixed:  list of `mat4` poses of the fixed solids in the same order as given to `__init__`
				close:  
					the joint positions we want the result the closest to. 
					If not provided, `self.default` will be used
				precision:  
					the desired precision tolerance for kinematic loops closing. this function returns once reached
				strict:  
					expected error tolerance on the kinematic loops closing, if not reached after `maxiter` iterations, this function will raise `KinematicError`.
					Leave it to 0 to raise when `precision` has not been reached
				maxiter:  
					maximum number of iterations allowed, or None if not limit
			
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
		
		max_increment = 0.3
		prec = 1e-6
		
		# state when some joints are fixed
		joints = []
		state = []
		for joint, p in zip(self.joints, close):
			if joint not in fixed:
				joints.append(joint)
				state.append(p)
		
		k = 0
		while True:
			k += 1
			if maxiter and k > maxiter and strict==inf:
				break
			
			move = -self.cost_residuals(state, fixed)
			error = np.abs(move).max()
			if error <= precision:
				break
			elif maxiter and k > maxiter:
				if error <= strict:
					break
				else:
					raise KinematicError('convergence failed after maximum iterations')
			
			# newton method with a pseudo inverse
			jac = self.cost_jacobian(state, fixed).transpose()
			increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*prec, jac @ move)
			state = structure_state(
				flatten_state(state)
				+ increment * min(1, max_increment / np.abs(increment).max()),
				state)
			
			for i, joint in enumerate(joints):
				state[i] = joint.normalize(state[i])
		print('acheived in iteration', k, 'with error', error)
		
		# structure results
		result = iter(state)
		final = []
		for joint, default in zip(self.joints, close):
			if joint in fixed:
				final.append(default)
			else:
				final.append(next(result))
		return final
	
	def parts(self, state, precision=1e-6) -> dict:
		''' return the pose of all solids in the kinematic for the given joints positions 
			The arguments are the same as for `self.direct()`
			
			Parameters:
				precision:  error tolerance for kinematic loop closing, if a loop is above this threshold this function will raise `KinematicError`
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
		poses = {self.order[0].solids[0]: mat4()}
		for joint in self.order:
			base = poses.get(joint.solids[0]) or mat4()
			tip = base * transforms[joint]
			if joint.solids[-1] in poses:
				# check that the given state is consistent, meaning kinematic loops are closed
				if np.abs(poses[joint.solids[-1]] - tip).max() > precision:
					raise KinematicError('position out of reach: kinematic cycles not closed')
			else:
				poses[joint.solids[-1]] = tip
		return poses
		
	def normalize(self, state) -> list:
		''' inplace normalize the joint coordinates in the given state '''
		for i, joint in enumerate(self.joints):
			state[i] = joint.normalize(state[i])
		return state
		
	def freedom(self, state, precision=1e-6) -> list:
		''' 
			list of free movement joint directions. the length of the list is the degree of freedom . 
			
			Note:
				When state is a singular position in the kinematic, the degree of freedom is locally smaller or bigger than in other positions
		'''
		return null_space(self.cost_jacobian(state), precision).transpose()
	
	def grad(self, state) -> dict:
		''' 
			return a gradient of the all the solids poses at the given joints position 
			
			Note:
				this function will ignore any degree of freedom of the kinematic that is not defined in `inputs`
		'''
		free = self.freedom(state).T
		inputs, outputs = free[:self.inputs_dim], free[-self.outputs_dim:]
		# pseudo inverse the input directions to get a jacobian matrix with input space being the defined input space
		try:
			jac = outputs @ la.inv(inputs.T @ inputs) @ inputs.T
		except la.LinAlgError as err:
			raise KinematicError("the defined degrees of freedom cannot move") from err
		# get matrix derivatives for each output joint
		result = []
		dmats = [j.grad(s)  for j,s in zip(self.joints[-len(self.outputs):], state[-len(self.outputs):])]
		for i,djoint in enumerate(jac.T):
			for dmat in dmats:
				result.append(sum( dm * dj   for dm,dj in zip(dmat,djoint) ))
		return result
		
	def direct(self, parameters: list, close=None) -> list:
		''' 
			shorthand to `self.solve(self.inputs)` and computation of desired transformation matrices
			it only works when direct and inverse constraining joints have been set
		'''
		fixed = dict(zip(self.inputs, parameters))
		result = self.solve(fixed, close)[-len(self.outputs):]
		return [joint.direct(x)  for joint, x in zip(self.outputs, result)]
		
	def inverse(self, parameters: list, close=None) -> list:
		''' 
			shorthand to `self.solve(self.outputs)` and extraction of desired joints
			it only works when direct and inverse constraining joints have been set
		'''
		fixed = {joint: joint.inverse(x)  for joint, x in zip(self.outputs, parameters)}
		result = self.solve(fixed, close)[:len(self.inputs)]
		return result[:len(self.inputs)]
	
	def display(self, scene):
		''' display allowing manipulation of kinematic '''
		from .displays import KinematicManip
		return KinematicManip(scene, self)


empty = ()

def flatten(structured):
	if hasattr(structured, '__iter__'):
		for x in structured:
			yield from flatten(x)
	else:
		yield structured

def flatten_state(structured, dtype=float):
	return np.array(typedlist(flatten(structured), dtype))

def structure_state(flat, structure):
	if isinstance(structure, (list, tuple)):
		it = iter(flat)
		structured = []
		for ref in structure:
			if hasattr(ref, '__len__'):
				structured.append(structure_state(it, ref))
			else:
				structured.append(next(it))
		return structured
	elif isinstance(structure, (vec1, vec2, vec3, vec4, quat)):
		return type(structure)(*[x  for i,x in zip(range(len(structure)), flat)])
	elif isinstance(structure, np.ndarray):
		return np.asarray([x  for i,x in zip(range(structure.size), flat)], float)
	else:
		raise TypeError("cannot structure {}".format(type(structure)))

def null_space(m, rcond=None):
	''' equivalent to `scipy.linalg.null_space` but 10x faster because using the numpy SVD '''
	if m.size == 0:
		return m
	u, s, vh = la.svd(m)
	M, N = u.shape[0], vh.shape[1]
	if rcond is None:
		rcond = np.finfo(s.dtype).eps * max(M, N)
	tol = np.amax(s) * rcond
	dim = np.sum(s > tol, dtype=int)
	return vh[dim:,:].T.conj()

def cycles(conn: '{node: [node]}') -> '[[node]]':
	''' extract a set of any-length cycles decomposing the graph '''
	todo

def shortcycles(conn: '{node: [node]}', costs: '{node: float}', branch=True, tree=None) -> '[[node]]':
	'''
		extract a set of minimal cycles decompsing the graph
		
		.. image:: /schemes/kinematic-cycles.svg
	'''
	# TODO: debug this because now it is not producing the shortest cycles
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
	# nprint('tree', tree)
	# nprint('distances', distances)
	# nprint('merges', merges)
	
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

def regularize_grad(grad):
	''' ensure it is a jacobian and not a single derivative '''
	if isinstance(grad, mat4) or isinstance(grad, np.ndarray) and grad.size == 16:
		return grad,
	return grad
