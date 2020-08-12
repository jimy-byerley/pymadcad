# This file is part of pymadcad,  distributed under license LGPL v3

''' This module defines the types and functions for kinematic manimulation and computation.

	
	A Kinematic is a conceptual approach of mechanisms. It sort parts in several groups with the same movement (so in a solid, the solids are all bound together), and it links the defined solids by joints corresponding to the constraints each solid put to the other solids in the joint. 
	That way no matter what are the parts, and what are their shape, even whan surfaces links the solids - the solid always have the same movements when there is the same joints between them.

	So to analyse a mechanisme we look at its kinematic. And that can be done prior or after the part design as it is independant.
	
	A kinematic in itself is a set of solids, observing movement relations. Those are modeled across the following classes: ``Solid`` and ``Kinematic``.
	
	Solids are considered to be undeformable, this allows the to use the Screw theory to represent the force and movement variables (see https://en.wikipedia.org/wiki/Screw_theory). 
	In this module, screws are called ``Torsor``.
	
	.. tip::
		In case of undeformable solids, torsors makes possible to represent both the translative and rotative part of each movement aspect, independently from the point in the solid.
'''

from copy import copy, deepcopy
import numpy.core as np
import moderngl as mgl
from PyQt5.QtCore import QEvent	

from .common import ressourcedir
from .mathutils import (fvec3, fmat4, vec3, vec4, mat3, mat4, quat, mat4_cast, quat_cast,
						column, translate, inverse, isnan,
						dot, cross, length, normalize, project, noproject, dirbase, distance,
						atan2, acos, angle, axis, angleAxis,
						pi, inf, glm,
						Box, transform)
from .mesh import Mesh
from . import settings
from . import constraints
from . import text


__all__ = ['Torsor', 'comomentum', 'Pressure', 'Solid', 'Kinematic', 'Kinemanip', 'solvekin',
			'makescheme', 'Scheme', 'WireDisplay',
			]


class Torsor(object):
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
			  Torsor(force, torque, pos)
		  * velocity (aka kinematic) torsor:
			  Torsor(rotation, velocity, pos)
		  * kinetic (inertia) torsor:
			  Torsor(linear movement quantity, rotational movement quantity, pos)
			
		  all these torsors makes it possible to represent all these values independently from expression location
	'''
	__slots__ = ('resulting', 'momentum', 'position')
	def __init__(self, resulting=None, momentum=None, position=None):
		self.resulting, self.momentum, self.position = resulting or vec3(0), momentum or vec3(0), position or vec3(0)
	def locate(self, pt) -> 'Torsor':
		''' gets the same torsor, but expressed for an other location '''
		return Torsor(self.resulting, self.momentum + cross(self.resulting, pt-self.position), pt)
	
	def transform(self, mat) -> 'Torsor':
		''' changes the torsor from coordinate system '''
		if isinstance(mat, mat4):
			rot, trans = mat3(mat), vec3(mat[3])
		elif isinstance(mat, mat3):
			rot, trans = mat, 0
		elif isinstance(mat, vec3):
			rot, trans = 1, mat
		else:
			raise TypeError('Torsor.transform() expect mat4, mat3 or vec3')
		return Torsor(rot*self.resulting, rot*self.momentum, rot*self.position + trans)
	
	def __add__(self, other):
		if other.position != self.position:		other = other.locate(self.position)
		return Torsor(self.resulting+other.resulting, self.momentum+other.momentum, self.position)
	
	def __sub__(self, other):
		if other.position != self.position:		other = other.locate(self.position)
		return Torsor(self.resulting-other.resulting, self.momentum-other.momentum, self.position)
	
	def __neg__(self):
		return Torsor(-self.resulting, -self.momentum, self.position)
	
	def __mul__(self, x):
		return Torsor(x*self.resulting, x*self.momentum, self.position)
	
	def __div__(self, x):
		return Torsor(self.resulting/x, self.momentum/x, self.position)
		
	def __repr__(self):
		return '{}(\n\t{}, \n\t{}, \n\t{})'.format(self.__class__.__name__, repr(self.resulting), repr(self.momentum), repr(self.position))

def comomentum(t1, t2):
	''' comomentum of torsors:   dot(M1, R2)  +  dot(M2, R1)
		the result is independent of torsors location
	'''
	t2 = t2.locate(t1.position)
	return dot(t1.momentum, t2.resulting) + dot(t2.momentum, t1.resulting)


class Solid:
	''' Solid for kinematic definition, used as variable by the kinematic solver
	
	Attributes:
		:orientation (quat):  rotation from local to world space
		:position (vec3):     displacement from local to world
		:visuals (list):      of objects to display using the solid's pose
		:name (str):          optional name to display on the scheme
	'''
	def __init__(self, *args, pose=None, name=None):
		if pose:
			self.position = pose[0]
			self.orientation = quat(pose[1])
		else:
			self.position = vec3(0)
			self.orientation = quat()
		self.visuals = list(args)
		self.name = name
	# solver variable definition
	slvvars = 'position', 'orientation',
	
	#@property
	def pose(self) -> 'mat4':
		''' transformation from local to global space, 
			therefore containing the translation and rotation from the global origin 
		'''
		return transform(self.position, self.orientation)
		
	#@pose.setter
	#def pose(self, mat):
		#self.position = vec3(mat[3])
		#self.orientation = quat_cast(mat3(mat))
	
	def transform(self, mat):
		''' displace the solid by the transformation '''
		if isinstance(mat, mat4):
			rot, trans = quat_cast(mat3(mat)), vec3(mat[3])
		elif isinstance(mat, mat3):
			rot, trans = quat_cast(mat), 0
		elif isinstance(mat, vec3):
			rot, trans = 1, mat
		else:
			raise TypeError('Torsor.transform() expect mat4, mat3 or vec3')
		self.orientation = rot*self.orientation
		self.position += trans
	
	def display(self, scene):
		from .mathutils import boundingbox
		from .displays import BoxDisplay
		for visu in self.visuals:
			yield from scene.display(visu)
		
		#box = boundingbox(self.visuals)
		#m = min(box.width)
		#box.min -= 0.2*m
		#box.max += 0.2*m
		#yield BoxDisplay(scene, box)
		
		#if self.name:	
			#yield text.Text(box.max, self.name)


def solvekin(joints, fixed=(), precision=1e-4, maxiter=None, damping=0.9):
	''' solver for kinematic joint constraints.
	
		Unlike ``solve``, the present solver is dedicated to kinematic usage (and far more efficient and precise). It doesn't rely on variables as defined by solve, but instead use Solids as constraints.
	'''
	# register solids and corrections
	solids = []		# list of solids found
	register = {}	# solid index indexed by their id()
	counts = []		# correction count for each solid
	indices = []	# solid index for each successive corrections
	if not isinstance(fixed, set):
		fixed = set(id(solid) for solid in fixed)
	for joint in joints:
		for solid in joint.solids:
			if id(solid) in fixed:
				indices.append(-1)
			else:
				if id(solid) not in register:
					register[id(solid)] = i = len(register)
					solids.append(solid)
					counts.append(0)
				else:	
					i = register[id(solid)]
				counts[i] += 1
				indices.append(i)
	
	corrmax = inf
	itercount = 0
	while corrmax > precision:
		if maxiter and maxiter <= itercount:
			raise constraints.SolveError('maximum iteration count reached with no solution found, err='+str(corrmax))
		
		corrmax = 0
		# collect corrections
		corr = [[] for i in range(len(solids))]
		i = 0
		for joint in joints:
			for action in joint.corrections():
				if indices[i] >= 0:
					corr[indices[i]].append(action)
				i += 1
		for solid,corrections in zip(solids, corr):
			# corrections are displacement torsors, (similar to kinematic torsor) therefore the translation is the momentum and the rotation is the resulting
			l = len(corrections)
			v = vec3(0)
			w = vec3(0)
			center = vec3(0)
			
			'''
			# displacement based correction:  the correction torsor is assimilated to Velocity torsor
			# rotation center is determined by the center of correction applications points
			# displacement is the average of correction displacements
			for c in corrections:
				v += c.momentum
				center += c.position
			v /= l
			center /= l
			# rotation is the average of correction rotations and rotation requested by displacement differences to the average displacement
			for c in corrections:
				r = length(c.position-center) + 1e-15
				induced = cross(c.momentum-v, c.position-center) / r**2
				lind = length(induced)
				if lind > 1:	induced /= lind
				w += c.resulting - induced
				corrmax = max(corrmax, length(c.momentum), length(c.resulting)*r)
			w /= 2*l
			'''
			
			# force based correction: the correction torsor is assimilated to Force torsor
			for c in corrections:
				center += c.position
			center /= l
			
			for c in corrections:
				v += c.resulting
				w += c.locate(center).momentum
				r = length(c.position-center) + 1e-15	# radius, to evaluate rotation impact geometry shape
				corrmax = max(corrmax, length(c.resulting), length(c.momentum)*r)
			v /= l*2
			w /= l*2
			if length(w) > 1:
				w /= length(w)
			
			solid.position += v*damping
			solid.orientation = quat(w*damping) * solid.orientation
		
		itercount += 1


class Kinematic:
	''' Holds a kinematic definition, and methods to use it
		The solid objects used are considered as variables and are modified inplace by methods, and can be modified at any time by outer functions
		The joints are not modified in any case (and must not be modified while a Kinematic is using it)
		
		Attributes defined here:
			* joints	the joints constraints
			* solids	all the solids the joint applys on, and eventually more
			* fixed		the root solids that is considered to be fixed to the ground
	'''
	def __init__(self, joints, fixed=None, solids=None):
		self.joints = joints
		if isinstance(joints, set):	self.fixed = fixed
		else:						self.fixed = set(id(solid) for solid in fixed)
		if not solids:	self._detectsolids()
		self.options = {}
	def _detectsolids(self):
		solids = []
		known = set()
		for joint in self.joints:
			for solid in joint.solids:
				if id(solid) not in known:
					solids.append(solid)
		self.solids = solids
		
	def solve(self, *args, **kwargs):
		return solvekin(self.joints, self.fixed, *args, **kwargs)
	def forces(self, applied) -> '[junction forces], [solid resulting]':
		''' return the forces in each junction and the resulting on each solid, induces by the given applied forces '''
		indev
	def jacobian(self):
		''' return the numerical jacobian for the current kinematic position '''
		indev
	def exploded(self) -> '[pose]':
		''' poses for an exploded view of the kinematic '''
		indev
	def path(self, func:'f(t)', pts) -> '[Wire]':
		''' path followed by points when the kinematic is moved by the function
			function must assign a reproductible pose to the solids, it takes one variable (usally the animation time step)
		'''
		indev
	
	@property
	def pose(self) -> '[mat4]':
		return [solid.pose() 	for solid in self.solids]
	@pose.setter
	def pose(self, value):
		for solid,pose in zip(self.solids, pose):
			solid.position = vec3(pose[3])
			solid.orientation = quat_cast(mat3(pose))
		
	def graph(self) -> 'Graph':
		''' graph representing solid as nodes and joints as bidirectional links '''
		indev
	def display(self, scene):
		''' renderer for kinematic manipulation, linked to the current object '''
		manip = Kinemanip(scene, self)
		for solid in self.solids:
			displays = list(solid.display(scene))
			manip.solids[id(solid)][1].extend(displays)
			for disp in displays:
				disp.control = lambda scene, grp, ident, evt, solid=solid:	manip.start(solid, scene, ident, evt)	# lambda default argument uncapture the variable
				yield disp
		manip.applyposes(scene)
		yield manip


class Pressure:
	''' Represent a mechanical pressure repartition on a surface
		equivalent of a Torsor but with space repartition
	'''
	def __init__(self, surface: 'Mesh', values: '[float]'):
		self.surface = surface
		self.values = values
	@classmethod
	def fromtorsor(cls, surface, torsor):
		indev
	@classmethod
	def fromfield(cls, surface, func):
		indev
	def torsor(self, point) -> Torsor:
		indev
	
	def __add__(self, other):
		assert len(other.surface.points) == len(self.surface.points), 'surface point numbers must match'
		indev
	
	def display(self, scene):
		''' surface rendering with coloration in function of pressure '''
		indev

class Graph:
	''' 
		nodes: list or dict		- a node is any python object
		links: [int] or [key]	- a link is a triplet (start node, end node, info) 
	'''
	def __init__(self, nodes, links):
		self.nodes = nodes
		self.links = links
	def cycles(self) -> '[cycle]':
		''' yield all the possible cycles in the graph '''
		indev
	def connexes(self, nodes=None) -> '[Graph]':
		''' list of connex subgraphs
			if nodes is provided, only the subgraphs containing nodes present in this list will be returned 
		'''
		indev
	def reach(self, node: int, walk:'callable'=None) -> '{node index: custom}':
		''' return a set of node indices that can be reached starting from the given node
			if walk is given, its a function(link)  returning a True object when walking through a link is possible 
		'''
		indev
	
	def display(self) -> 'QGraphicItem':
		''' graph display widget for a 2D QGraphicScene '''
		indev


def store(dst, src):
	for i in range(len(dst)):
		dst[i] = src[i]

class Kinemanip:
	''' Display that holds a kinematic structure and allows the user to move it
	'''
	renderindex = 0
	
	def __init__(self, scene, kinematic):
		self.joints = kinematic.joints
		self.fixed = kinematic.fixed
		self.solids = {id(solid): (solid, [])   for solid in kinematic.solids}
		self.locked = set(kinematic.fixed)
	
	def render(self, scene):	pass
	def identify(self, scene, startident):	pass	
		
	def start(self, solid, scene, ident, evt):
		if id(solid) in self.fixed:		return
		evt.accept()
		
		self.startpt = scene.ptat(scene.objnear((evt.x(), evt.y())))
		self.ptoffset = inverse(quat(solid.orientation)) * (solid.position - self.startpt)
		self.actsolid = solid
		self.moved = False
		return self.move
	
	def move(self, scene, evt):
		evt.accept()
		if evt.type() == QEvent.MouseMove:
			# unlock moving solid
			self.lock(self.actsolid, False)
			self.moved = True
			# displace the moved object
			pt = scene.ptfrom((evt.x(), evt.y()), self.startpt)
			self.startpt = self.actsolid.position - quat(self.actsolid.orientation)*self.ptoffset
			#store(self.actsolid.position, pt+quat(self.actsolid.orientation)*self.ptoffset)
			self.actsolid.position = pt+quat(self.actsolid.orientation)*self.ptoffset
			# solve
			try:	solvekin(self.joints, self.locked, precision=1e-2, maxiter=50)
			except constraints.SolveError as err:	pass
		else:
			if not self.moved:
				self.lock(self.actsolid, id(self.actsolid) not in self.locked)
			else:
				# finish on a better precision
				try:	solvekin(self.joints, self.locked, precision=1e-4, maxiter=1000)
				except constraints.SolveError as err:	print(err)
			scene.tool = None
			
		self.applyposes(scene)
		
	def applyposes(self, scene):
		# assign new positions to displays
		for solid,displays in self.solids.values():
			trans = fmat4(solid.pose())
			for disp in displays:
				if hasattr(disp, 'transform'):
					disp.transform = trans
		scene.update()
		return True
	
	def lock(self, solid, lock):
		key = id(solid)
		if key in self.fixed:	return
		if lock:
			if key in self.locked:	return
			# add solid's variables to fixed
			self.locked.add(key)
			# grey display for locked solid
			for disp in self.solids[key][1]:
				disp.color = fvec3(0.5, 0.5, 0.5)
		else:
			if key not in self.locked:	return
			# remove solid's variables from fixed
			self.locked.discard(key)
			# reset solid color
			for disp in self.solids[key][1]:
				disp.color = fvec3(settings.display['schematics_color'])
		



def makescheme(joints, color=None):
	''' create kinematic schemes and add them as visual elements to the solids the joints applies on '''
	# collect solids informations
	solids = {}
	diag = vec3(0)
	for cst in joints:
		for solid, pos in zip(cst.solids, cst.position):
			if id(solid) not in solids:
				solids[id(solid)] = info = [solid, [], vec3(0), 0]
			else:
				info = solids[id(solid)]
			info[1].append(cst)
			if pos:
				info[2] += pos
				info[3] += 1
				diag = glm.max(diag, glm.abs(pos))
	# get the junction size
	size = (max(diag) or 1) / len(joints)
	
	for info in solids.values():
		scheme = Scheme([], [], [], [], color, solid.pose())
		center = info[2]/info[3]
		if glm.any(isnan(center)):	center = vec3(0)
		for cst in info[1]:
			scheme.extend(cst.scheme(info[0], size, center))
		info[0].visuals.append(scheme)
		if info[0].name:
			info[0].visuals.append(text.Text(center, info[0].name))


class Scheme:
	''' buffer holder to construct schemes, for now it's only usefull to append to buffer '''
	def __init__(self, points, transpfaces, opacfaces, lines, color=None, transform=None):
		self.color = color or settings.display['schematics_color']
		self.points = points
		self.transpfaces = transpfaces
		self.opaqfaces = opacfaces
		self.lines = lines
		self.transform = transform
	
	def extend(self, other):
		l = len(self.points)
		self.points.extend(other.points)
		self.transpfaces.extend(((a+l,b+l,c+l) for a,b,c in other.transpfaces))
		self.opaqfaces.extend(((a+l,b+l,c+l) for a,b,c in other.opaqfaces))
		self.lines.extend(((a+l, b+l)  for a,b in other.lines))
	
	def box(self):
		''' return the extreme coordinates of the mesh (vec3, vec3) '''
		if not self.points:		return None
		max = deepcopy(self.points[0])
		min = deepcopy(self.points[0])
		for pt in self.points:
			for i in range(3):
				if pt[i] < min[i]:	min[i] = pt[i]
				if pt[i] > max[i]:	max[i] = pt[i]
		return Box(min, max)
	
	def display(self, scene):
		return WireDisplay(scene, self.points, self.transpfaces, self.opaqfaces, self.lines, self.color, self.transform),


class WireDisplay:
	''' wireframe display for schemes, like kinematic schemes '''
	renderindex = 2
	
	def __init__(self, scene, points, transpfaces, opaqfaces, lines, color=None, transform=fmat4(1)):
		ctx = scene.ctx
		self.transform = fmat4(transform)
		self.color = fvec3(color or settings.display['wire_color'])
		
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/uniformcolor.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/uniformcolor.frag').read(),
						)
		self.uniformshader = scene.ressource('shader_uniformcolor', load)
		
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/glowenvelope.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/glowenvelope.frag').read(),
						)
		self.transpshader = scene.ressource('shader_glowenvelope', load)
		
		normals = Mesh(points, transpfaces).vertexnormals()
		#print('normals', normals)
		
		self.vb_vertices = ctx.buffer(np.hstack((
				np.array([tuple(v) for v in points], dtype='f4', copy=False),
				np.array([tuple(v) for v in normals], dtype='f4'),
				)))
		if transpfaces:
			self.vb_transpfaces = ctx.buffer(np.array(transpfaces, dtype='u4', copy=False))
			self.va_transpfaces = ctx.vertex_array(
					self.transpshader,
					[(self.vb_vertices, '3f4 3f4', 'v_position', 'v_normal')],
					self.vb_transpfaces,
					)
			self.va_ident_faces = ctx.vertex_array(
					scene.ident_shader,
					[(self.vb_vertices, '3f4 12x', 'v_position')],
					self.vb_transpfaces,
					)
		else:
			self.vb_transpfaces = None
		if opaqfaces:
			self.vb_opaqfaces = ctx.buffer(np.array(opaqfaces, dtype='u4', copy=False))
			self.va_opaqfaces = ctx.vertex_array(
					self.uniformshader,
					[(self.vb_vertices, '3f4 12x', 'v_position')],
					self.vb_opaqfaces,
					)
		else:
			self.vb_opaqfaces = None
		if lines:
			self.vb_lines = ctx.buffer(np.array(lines, dtype='u4', copy=False))
			self.va_lines = ctx.vertex_array(
					self.uniformshader,
					[(self.vb_vertices, '3f4 12x', 'v_position')],
					self.vb_lines,
					)
			self.va_ident_lines = ctx.vertex_array(
					scene.ident_shader,
					[(self.vb_vertices, '3f4 12x', 'v_position')],
					self.vb_lines,
					)
		else:
			self.vb_lines = None
	
	def render(self, scene):		
		viewmat = scene.view_matrix * self.transform
		
		#scene.ctx.blend_func = mgl.SRC_ALPHA, mgl.ONE_MINUS_SRC_ALPHA
		#scene.ctx.blend_equation = mgl.FUNC_ADD
		
		self.uniformshader['color'].write(self.color)
		self.uniformshader['view'].write(viewmat)
		self.uniformshader['proj'].write(scene.proj_matrix)
		
		scene.ctx.disable(mgl.DEPTH_TEST)
		scene.ctx.disable(mgl.CULL_FACE)
		if self.vb_opaqfaces:	self.va_opaqfaces.render(mgl.TRIANGLES)
		if self.vb_lines:		self.va_lines.render(mgl.LINES)
		
		scene.ctx.enable(mgl.CULL_FACE)
		if self.vb_transpfaces:	
			self.transpshader['color'].write(self.color)
			self.transpshader['view'].write(viewmat)
			self.transpshader['proj'].write(scene.proj_matrix)
			self.va_transpfaces.render(mgl.TRIANGLES)
		scene.ctx.enable(mgl.DEPTH_TEST)
		scene.ctx.disable(mgl.CULL_FACE)
	
	def identify(self, scene, startident):
		viewmat = scene.view_matrix * self.transform
		scene.ident_shader['ident'] = startident
		scene.ident_shader['view'].write(viewmat)
		scene.ident_shader['proj'].write(scene.proj_matrix)
		
		if self.vb_transpfaces:		self.va_ident_faces.render(mgl.TRIANGLES)
		if self.vb_lines:			self.va_ident_lines.render(mgl.LINES)
		return 1


