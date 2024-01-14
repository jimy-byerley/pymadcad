# This file is part of pymadcad,  distributed under license LGPL v3

''' This module defines the types and functions for kinematic manimulation and computation.

	
	A Kinematic is a conceptual approach of mechanisms. It sort parts in several groups with the same movement (so in a solid, the solids are all bound together), and it links the defined solids by joints corresponding to the constraints each solid put to the other solids in the joint. 
	That way no matter what are the parts, and what are their shape, even whan surfaces links the solids - the solid always have the same movements when there is the same joints between them.

	So to analyse a mechanisme we look at its kinematic. And that can be done prior or after the part design as it is independant.
	
	A kinematic in itself is a set of solids, observing movement relations. Those are modeled across the following classes: ``Solid`` and ``Kinematic``.
	
	Solids are considered to be undeformable, this allows the to use the Screw theory to represent the force and movement variables (see https://en.wikipedia.org/wiki/Screw_theory). 
	In this module, screws are called ``Screw``.
	
	.. tip::
		In case of undeformable solids, torsors makes possible to represent both the translative and rotative part of each movement aspect, independently from the point in the solid.
'''

from copy import copy, deepcopy
import numpy.core as np
import scipy
import moderngl as mgl
from PyQt5.QtCore import Qt, QEvent	

from .common import ressourcedir
from .mathutils import *
from .mesh import Mesh, Web, Wire, striplist, distance2_pm, typedlist_to_numpy
from . import settings
from . import rendering
from . import nprint
from .displays import BoxDisplay

__all__ = ['Screw', 'comomentum', 
			'Solid', 'Kinematic', 'Kinemanip', 
			'Joint', 'Reverse', 'Chain', 
			'solve', 'KinematicError',
			]

class KinematicError(Exception): pass
			

class Screw(object):
	''' a 3D torsor aka Screw aka Wrench aka Twist - is a mathematical object defined as follow:
		  * a resulting vector R
		  * a momentum vector field M
		  the momentum is a function of space, satisfying the relationship:
			M(A) = M(B) + cross(R, A-B)
		
		therefore it is possible to represent a localized torsor such as:
		  * R = resulting
		  * M = momentum vector at position P
		  * P = position at which M takes the current value
		
		torsor are usefull for generalized solid mechanics to handle multiple variables of the same nature:
		  * force torsor:	
			  Screw(force, torque, pos)
		  * velocity (aka kinematic) torsor:
			  Screw(rotation, velocity, pos)
		  * kinetic (inertia) torsor:
			  Screw(linear movement quantity, rotational movement quantity, pos)
			
		  all these torsors makes it possible to represent all these values independently from expression location
		  
		  
		Attributes:
			resulting (vec3): 
			momentum (vec3):
			position (vec3):
	'''
	__slots__ = ('resulting', 'momentum', 'position')
	def __init__(self, resulting=None, momentum=None, position=None):
		self.resulting, self.momentum, self.position = resulting or vec3(0), momentum or vec3(0), position or vec3(0)
	def locate(self, pt) -> 'Screw':
		''' gets the same torsor, but expressed for an other location '''
		return Screw(self.resulting, self.momentum + cross(self.resulting, pt-self.position), pt)
	
	def transform(self, mat) -> 'Screw':
		''' changes the torsor from coordinate system '''
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
	''' comomentum of screws:   `dot(M1, R2)  +  dot(M2, R1)`
		
		the result is independent of torsors location
	'''
	t2 = t2.locate(t1.position)
	return dot(t1.momentum, t2.resulting) + dot(t2.momentum, t1.resulting)


class Solid:
	''' Solid for kinematic definition, used as variable by the kinematic solver
	
		A Solid is also a way to group objects and move it anywere without modifying them, as the objects contained in a solid are considered to be in solid local coordinates.
		A Solid is just like a dictionnary with a pose.
	
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
	def __init__(self, pose=None, **content):
		if isinstance(pose, tuple):
			self.position = pose[0]
			self.orientation = quat(pose[1])
		elif isinstance(pose, mat4):
			self.position = pose[3].xyz
			self.orientation = quat(mat3(pose))
		else:
			self.position = vec3(0)
			self.orientation = quat()
		self.content = content
	
	# solver variable definition
	slvvars = 'position', 'orientation',
	
	@property
	def pose(self) -> 'mat4':
		''' transformation from local to global space, 
			therefore containing the translation and rotation from the global origin 
		'''
		return transform(self.position, self.orientation)
		
	@pose.setter
	def pose(self, mat):
		self.position = vec3(mat[3])
		self.orientation = quat_cast(mat3(mat))
		
	def __copy__(self):
		s = Solid()
		s.position = copy(self.position)
		s.orientation = copy(self.orientation)
		s.content = self.content
		return s
	
	def transform(self, trans) -> 'Solid':
		''' displace the solid by the transformation '''
		s = copy(self)
		if isinstance(trans, mat4):
			rot, trans = quat_cast(mat3(trans)), vec3(trans[3])
		elif isinstance(trans, mat3):
			rot, trans = quat_cast(trans), 0
		elif isinstance(trans, quat):
			rot, trans = trans, 0
		elif isinstance(trans, vec3):
			rot, trans = 1, trans
		else:
			raise TypeError('Screw.transform() expect mat4, mat3 or vec3')
		s.orientation = rot*self.orientation
		s.position = trans + rot*self.position
		return s
	
	def place(self, *args, **kwargs) -> 'Solid': 
		''' strictly equivalent to `self.pose = placement(...)`, see `placement` for parameters specifications. '''
		s = copy(self)
		s.pose = placement(*args, **kwargs)
		return s
		
	def deloc(self, *args):
		indev
	def reloc(self, *args):
		indev
	def loc(self, *args):
		indev
	

	# convenient content access
	def __getitem__(self, key):
		''' shorthand to `self.content` '''
		return self.content[key]
		
	def __setitem__(self, key, value):
		''' shorthand to `self.content` '''
		self.content[key] = value
	
	def add(self, value):
		''' add an item in self.content, a key is automatically created for it and is returned '''
		key = next(i 	for i in range(len(self.content)+1)	
						if i not in self.content	)
		self.content[key] = value
		return key
	
	def set(self, **objs):
		''' contenient method to set many elements in one call.
			equivalent to `self.content.update(objs)`
		'''
		self.content.update(objs)
		return self
	
	
	class display(rendering.Group):
		''' movable `Group` for the rendering pipeline '''
		def __init__(self, scene, solid):
			super().__init__(scene, solid.content)
			self.solid = solid
			self.apply_pose()
		
		def update(self, scene, solid):
			if not isinstance(solid, Solid):	return
			super().update(scene, solid.content)
			self.solid = solid
			return True
		
		def stack(self, scene):
			for key,display in self.displays.items():
				if key == 'annotations' and not scene.options['display_annotations'] and not self.selected:
					continue
				for sub,target,priority,func in display.stack(scene):
					yield ((key, *sub), target, priority, func)
				
		def control(self, view, key, sub, evt):
				# solid manipulation
			if not view.scene.options['lock_solids'] and evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
				evt.accept()
				start = view.ptat(view.somenear(evt.pos()))
				offset = self.solid.position - affineInverse(mat4(self.world)) * start
				view.tool.append(rendering.Tool(self.move, view, start, offset))
			# this click might have been a selection, ask for scene restack just in case
			elif evt.type() == QEvent.MouseButtonRelease and evt.button() == Qt.LeftButton:
				view.scene.touch()
				
		def move(self, dispatcher, view, pt, offset):
			moved = False
			while True:
				evt = yield
				
				if evt.type() == QEvent.MouseMove:
					evt.accept()
					moved = True
					world = mat4(self.world)
					pt = affineInverse(world) * view.ptfrom(evt.pos(), world * pt)
					self.solid.position = pt + offset
					self.apply_pose()
					view.update()
				
				if evt.type() == QEvent.MouseButtonRelease and evt.button() == Qt.LeftButton:
					if moved:	evt.accept()
					break
						
		def apply_pose(self):
			self.pose = fmat4(self.solid.pose)
			
			
'''
k = Kinematic([
	Welded((s1, s2), translate(2*X)),
	Pivot((s2, s3), Axis(O,Z)),
	Pivot((s1, s3), (Axis(O,Z), Axis(A,X))),
	])
k = Kinematic([
	Welded((1, 2), translate(2*X)),
	Pivot((2, 3), Axis(O,Z)),
	Pivot((1, 3), (Axis(O,Z), Axis(A,X))),
	])
k.direct([None, 1.5, 0])
k.inverse({1: mat4(), 2: mat4(), 3: mat4()})

'''
			
class Joint:
	def __init__(self, *args, default=0, **kwargs):
		if isinstance(args[0], Solid) and isinstance(args[1], Solid):
			self.solids = args[:2]
			args = args[2:]
		self.default = default
		self.init(*args, **kwargs)
	
	# parameter bounds if any
	bounds = (-inf, inf)
	
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
	
	def transmit(self, force: Screw, parameters=None, velocity=None) -> 'Screw':
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
	
	def schemes(self, size: float, junc: vec3=None) -> '[Scheme]':
		''' generate the scheme elements to render the joint '''
		raise NotImplemented
	
	class display(rendering.Display):
		def __init__(self, scene, joint):
			self.joint = joint
			self.schemes = [scene.display(joint.scheme(s, 1, joint.position[i]))]
		
		def __getitem__(self, sub):
			return self.schemes[sub]
		def __iter__(self):
			return iter(self.schemes)
		
		def stack(self, scene):
			yield ((), 'screen', -1, self.updateposes)
			for i,scheme in enumerate(self.schemes):
				for sub,target,priority,func in scheme.stack(scene):
					yield ((i, *sub), target, priority, func)
		
		def updateposes(self, view):
			''' update the pose of sub displays using their solid's pose '''
			for sch, solid, pos in zip(self.schemes, self.joint.solids, self.joint.position):
				pos = fvec3(pos)
				m = self.world * fmat4(solid.pose)
				d = (view.uniforms['view'] * m * fvec4(fvec3(pos),1)).z * 30/view.height()
				sch.world = m * translate(scale(translate(fmat4(1), pos), fvec3(d)), -pos)

class Kinematic(Joint):
	pass

class Kinemanip(rendering.Group):
	''' Display that holds a kinematic structure and allows the user to move it
	'''
	
	def __init__(self, scene, kinematic):
		super().__init__(scene)
		try:	kinematic.solve(maxiter=1000)
		except constraints.KinematicError:	pass
		self.sizeref = 1
		self._init(scene, kinematic)
	
	def update(self, scene, kinematic):
		# fail on any kinematic change
		if not isinstance(kinematic, Kinematic) or len(self.solids) != len(kinematic.solids):	return
		# keep current pose
		for ss,ns in zip(self.solids, kinematic.solids):
			ns.position = ss.position
			ns.orientation = ss.orientation
		# if any change in joints, rebuild the scheme
		for sj, nj in zip(self.joints, kinematic.joints):
			if type(sj) != type(nj) or sj.solids != nj.solids:
				self._init(scene, kinematic)
				return True
		return super().update(scene, kinematic.solids)
	
	def _init(self, scene, kinematic):
		self.joints = kinematic.joints
		self.fixed = kinematic.fixed
		self.locked = set(kinematic.fixed)
		self.solids = kinematic.solids
		self.register = {id(s): i	for i,s in enumerate(self.solids)}
		makescheme(self.joints)
		super().update(scene, self.solids)
		
	def control(self, view, key, sub, evt):
		# no action on the root solid
		if sub[0] in self.fixed:	return
		
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
			# start solid drag
			evt.accept()
			solid = self.solids[sub[0]]
			self.sizeref = max(norminf(self.box.width), 1)
			start = vec3(affineInverse(mat4(self.world)) * vec4(view.ptat(view.somenear(evt.pos())),1))
			offset = inverse(quat(solid.orientation)) * (start - solid.position)
			view.tool.append(rendering.Tool(self.move, view, solid, start, offset))
	
	def move(self, dispatcher, view, solid, start, offset):
		moved = False
		while True:
			evt = yield
			
			if evt.type() == QEvent.MouseMove:
				evt.accept()
				moved = True
				# unlock moving solid
				if self.islocked(solid):
					self.lock(view.scene, solid, False)
				# displace the moved object
				start = solid.position + quat(solid.orientation)*offset
				pt = vec3(affineInverse(mat4(self.world)) * vec4(view.ptfrom(evt.pos(), start),1))
				solid.position = pt - quat(solid.orientation)*offset
				# solve
				self.solve(False)
				view.update()
			
			elif evt.type() == QEvent.MouseButtonRelease and evt.button() == Qt.LeftButton:
				if moved:	evt.accept()
				break
		
		if moved:
			# finish on a better precision
			self.solve(True)
			view.update()
	
	def solve(self, final=False):
		try:	
			if final:	solvekin(self.joints, self.locked, precision=self.sizeref*1e-4, maxiter=1000)
			else:		solvekin(self.joints, self.locked, precision=self.sizeref*1e-3, maxiter=50)
		except constraints.KinematicError as err:	
			for disp in self.displays.values():
				if 'solid-fixed' in disp.displays:
					disp.displays['solid-fixed'].color = fvec3(settings.display['solver_error_color'])
		else:
			for disp in self.displays.values():
				if 'solid-fixed' in disp.displays:
					disp.displays['solid-fixed'].color = fvec3(settings.display['schematics_color'])
		self.apply_poses()
	
	def apply_poses(self):
		# assign new positions to displays
		for disp in self.displays.values():
			disp.apply_pose()
	
	def lock(self, scene, solid, lock):
		''' lock the pose of the given solid '''
		if lock == self.islocked(solid):	
			return
		key = id(solid)
		grp = self.displays[self.register[key]]
		if lock:
			# add solid's variables to fixed
			self.locked.add(key)
			box = Box(center=fvec3(0), width=fvec3(-inf))
			for display in grp.displays.values():
				box.union_update(display.box)
			grp.displays['solid-fixed'] = BoxDisplay(scene, box, color=fvec3(settings.display['schematics_color']))
			self.apply_poses()
		else:
			# remove solid's variables from fixed
			self.locked.remove(key)
			if 'solid-fixed' in grp.displays:
				del grp.displays['solid-fixed']
		scene.touch()
		
	def islocked(self, solid):
		return id(solid) in self.locked


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

def solve(joints, fixed:dict=None, init:list=None, precision=1e-8, maxiter=None) -> list:  pass
def solve(joints, fixed:dict=None, init:dict=None, precision=1e-8, maxiter=None) -> dict:  pass

class Kinematic:
	def __init__(self, joints, solids:dict=None):
		self.joints = joints
		self.vars
		self.cycles
		self.solids = solids
	def to_urdf(self):  
		indev
	def simplify(self) -> Kinematic:
		indev
	def direct(self, state) -> dict:
		indev
	def grad(self, state) -> dict:
		indev
	def inverse(self, fixed:dict=None, close:list=None, precision=1e-8, maxiter=None) -> list:
		indev
	def display(self, scene):
		indev

def solve(joints, fixed:dict=None, close:list=None, precision=1e-8, maxiter=None) -> list:
	dtype = np.float64
	usedict = isinstance(close, dict)
	if usedict:
		close = [close.get(joint) or joint.default  for joint in joints]
	elif close is None:
		close = [joint.default  for joint in joints]
	
	# collect the joint graph as a connectivity and the solids
	conn = {}  # connectivity {node: [node]}  with nodes being joints and solids
	for joint in joints:
		conn[joint] = joint.solids
		for solid in joint.solids:
			if solid not in conn:	
				conn[solid] = []
			conn[solid].append(joint)
			
	# # simplify the graph
	# simplified = {solid: []  for solid in solids}
	# vars = {}
	# nprint(conn)
	# nprint(arcs(conn))
	# for branch in arcs(conn):
	# 	# cycles may not begin with a solid, change this
	# 	if branch[-1] == branch[0] and isinstance(branch[0], Joint):
	# 		branch.pop(0)
	# 		branch.append(branch[0])
	# 	# simplify this arc into a chain if possible
	# 	if len(branch) > 3:
	# 		chain = Chain(branch[1::2])
	# 	else:
	# 		chain = branch[1]
	# 	# assemble new graph
	# 	vars[joint] = np.array(chain.default).size
	# 	simplified[chain] = chain.solids
	# 	for solid in chain.solids:
	# 		simplified[solid].append(chain)
	# nprint('simplified', simplified)
	
	# use the graph as is
	vars = {joint: flatten_state(joint.default, dtype).size
		for joint in conn  
		if isinstance(joint, Joint)}
	
	nvars = sum(vars.values())
	squeezed_homogeneous = 9
		
	# build bounds and state vector index
	mins = []
	maxes = []
	index = {}
	i = 0
	for joint, value in zip(joints, close):
		index[joint] = i
		i += flatten_state(value, dtype).size
		mins.append(joint.bounds[0])
		maxes.append(joint.bounds[1])
	
	# decompose into cycles
	cycles = []
	rev = {}
	for cycle in shortcycles(conn, vars, branch=False):
		# cycles may not begin with a solid, change this
		if isinstance(cycle[0], Joint):
			cycle.pop(0)
			cycle.append(cycle[0])
		chain = []
		for i in range(1, len(cycle), 2):
			joint = cycle[i]
			if cycle[i-1] != joint.solids[0]:
				if joint not in rev:
					rev[joint] = Reverse(joint)
					index[joint] = index[joint]
				joint = rev[joint]
			chain.append(joint)
		cycles.append(chain)
	
	nprint('cycles', cycles)
	if len(cycles) == 0:
		return close
	
	# build cost and gradient functions
	# cost function returns residuals, the solver will optimize the sum of their squares
	def cost(x):
		x = structure_state(x, close)
		cost = np.empty((len(cycles), squeezed_homogeneous), dtype)
		
		# collect transformations
		transforms = {}
		for joint, p in zip(joints, x):
			transforms[joint] = joint.direct(p)
			if joint in rev:
				transforms[rev.get(joint)] = affineInverse(transforms[joint])
		
		# chain transformations
		for i, cycle in enumerate(cycles):
			# b is the transform of the complete cycle: it should be identity
			b = mat4()
			for joint in cycle:
				b = b * transforms[joint]
			# the residual of this matrix is the difference to identity
			# pick only non-redundant components to reduce the problem size
			cost[i] = squeeze_homogeneous(b - mat4())
			#cost[i] **= 2
		return cost.ravel()
	
	# jacobian of the cost function
	def jac(x):
		x = structure_state(x, close)
		jac = np.zeros((len(cycles), nvars, squeezed_homogeneous), dtype)
		# jac = scipy.sparse.csr_matrix((len(cycles)*squeezed_homogeneous, nvars))
		
		# collect gradient and transformations
		transforms = {}
		for joint, p in zip(joints, x):
			size = np.array(p).size
			grad = joint.grad(p)
			# ensure it is a jacobian and not a single derivative
			if size <= 1:
				grad = grad,
			# ensure the homogeneous factor is 0 for derivatives
			for df in grad:
				df[3][3] = 0
			f = joint.direct(p)
			transforms[joint] = (index[joint], f, grad)
			if joint in rev:
				rf = affineInverse(f)
				transforms[rev[joint]] = (index[joint], rf, [-rf*df*rf  for df in grad])
		
		# stack gradients, transformed by the chain successive transformations
		for icycle, cycle in enumerate(cycles):
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
			k = len(cycle)-1
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
		return jac.transpose((0,2,1)).reshape((len(cycles)*squeezed_homogeneous, nvars))
		# return jac
	
	# solve
	from time import perf_counter as time
	start = time()
	
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
				# ftol = 0,
				# gtol = 0,
				max_nfev = maxiter,
				)
	
	print('solved in', time() - start)
	# 0.03 s with dense estimated jac
	# 0.013 s with dense provided jac
	# 0.02 s with sparse provided jac
	# nfev = 5 in all cases
	print(res)
	np.set_printoptions(linewidth=np.inf)
	estimated = res.jac
	computed = jac(res.x)
	# print(estimated, (np.abs(estimated) > 1e-3).sum())
	# print()
	# print(computed, (np.abs(computed) > 1e-3).sum())
	
	from matplotlib import pyplot as plt
	# estimated = estimated.toarray()
	# computed = computed.toarray()
	plt.imshow(np.stack([
		estimated*0, 
		# np.abs(estimated)/np.abs(estimated).max(axis=1)[:,None],
		# np.abs(computed)/np.abs(estimated).max(axis=1)[:,None],
		np.log(np.abs(estimated))/10+0.5,
		np.log(np.abs(computed))/10+0.5,
		], axis=2))
	plt.show()
	
	if not res.success:
		raise KinematicError('failed to converge: '+res.message, res)
# 	if res.cost > precision * nvars:
# 		raise KinematicError('cannot close the kinematic cycles')
	
	# structure results
	result = structure_state(res.x, close)
	if usedict:
		return dict(zip(joints, result))
	return result


def flatten(structured):
	if hasattr(structured, '__len__'):
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

def squeeze_homogeneous(m):
	return (m[0][0], m[1][1], m[2][2], m[2][1], m[2][0], m[1][0], m[3][0], m[3][1], m[3][2])


def cycles(conn: '{node: [node]}') -> '[[node]]':
	todo

def shortcycles(conn: '{node: [node]}', costs: '{node: float}', branch=True) -> '[[node]]':
	# orient the graph in a depth-first way, and search for fusion points
	distances = {}
	merges = []
	tree = {}
	for parent, child in depthfirst(conn):
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
			
				
	
def depthfirst(conn: '{node: [node]}') -> '[(parent, child)]':
	edges = set()
	reached = set()
	ordered = []
	for start in conn:
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

import numpy as np

class Reverse(Joint):
	def __init__(self, joint):
		self.joint = joint
		self.solids = joint.solids[::-1]
		
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

class Chain(Joint):
	def __init__(self, joints):
		if not all(joints[i-1].solids[-1] == joints[i].solids[0]  
				for i in range(len(joints))):
			raise ValueError('joints do not form a direct chain, joints are not badly ordered or oriented')
		self.joints = joints
		self.solids = (joints[0].solids[0], joints[-1].solids[-1])
		
	@property
	def default(self):
		return [joint.default  for joint in self.joints]
		
	@property
	def bounds(self):
		return [joint.bounds  for joint in self.joints]
	
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
			for df in joint.grad(x):
				grad.append(b*df)
			b = b*f
			directs.append((f, len(x)))
		# build the right side of the product
		b = mat4(1)
		for f,n in reversed(directs):
			for i in range(n):
				grad[i] = grad[i]*b
			b = f*b
		return grad
	
	def parts(self, parameters):
		solids = [mat4()] * (len(self.joints)+1)
		for i in range(len(self.joints)):
			solids[i+1] = solids[i] * self.joints[i].direct(parameters[i])
		return solids
		
	def to_dh(self) -> '(dh, transforms)':
		''' denavit-hartenberg representation of this kinematic chain '''
		indev
		
	def from_dh(dh, transforms=None) -> 'Self':
		indev
		
	def __repr__(self):
		return '{}({})'.format(self.__class__.__name__, repr(self.joints))
	
	

def makescheme(joints, color=None):
	''' create kinematic schemes and add them as visual elements to the solids the joints applies on '''
	# collect solids informations
	assigned = set()
	solids = {}
	diag = vec3(0)
	for cst in joints:
		for solid, pos in zip(cst.solids, cst.position):
			if id(solid) not in solids:
				solids[id(solid)] = info = [solid, [], vec3(0), 0, Box()]
			else:
				info = solids[id(solid)]
			info[1].append(cst)
			if pos:
				info[2] += pos
				info[3] += 1
				info[4].union_update(pos)
	# get the junction size
	#size = (max(diag) or 1) / (len(joints)+1)
	size = 0.5 * max(max(info[4].width) / info[3] for info in solids.values())  or 1
	
	for info in solids.values():
		container = info[0].content
		if id(container) not in assigned:
			container['scheme'] = Scheme([], [], [], [], color)
			assigned.add(id(container))
		scheme = container['scheme']
		center = info[2]/info[3]
		if not isfinite(center):	center = vec3(0)
		for cst in info[1]:
			scheme.extend(cst.scheme(info[0], size, center))
		

def placement(*pairs, precision=1e-3):
	''' return a transformation matrix that solved the placement constraints given by the surface pairs
	
		Parameters:
		
			pairs:	a list of pairs to convert to kinematic joints
					
					- items can be couples of surfaces to convert to joints using `guessjoint`
					- tuples (joint_type, a, b)  to build joints `joint_type(solida, solidb, a, b)`
			
			precision: surface guessing and kinematic solving precision (distance)
		
		each pair define a joint between the two assumed solids (a solid for the left members of the pairs, and a solid for the right members of the pairs). placement will return the pose of the first relatively to the second, satisfying the constraints.
		
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
	from random import random
	
	a, b = Solid(), Solid()
	joints = []
	for pair in pairs:
		if len(pair) == 2:		joints.append(guessjoint(a, b, *pair, precision*0.25))
		elif len(pair) == 3:	joints.append(pair[0](a, b, *pair[1:]))
		else:
			raise TypeError('incorrect pair definition', pair)
	solvekin(joints, fixed=[b], precision=precision, maxiter=1000)
	return a.pose

	
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
	''' build a graph of connected objects, ready to create an exploded view or any assembly animation.
		See `explode()` for an example. The exploded view is computed using the meshes contained in the given solids, so make sure there everything you want in their content.
	
		Complexity is `O(m * n)` where m = total number of points in all meshes, n = number of solids
		
		NOTE:
			
			Despite the hope that this function will be helpful, it's (for computational cost reasons) not a perfect algorithm for complex assemblies (the example above is at the limit of a simple one). The current algorithm will work fine for any simple enough assembly but may return unexpected results for more complexe ones.
		
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
	''' move the given solids away from each other in the way of an exploded view.
		makes easier to seen the details of an assembly . See `explode_offsets` for the algorithm
		
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

