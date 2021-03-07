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
import moderngl as mgl
from PyQt5.QtCore import Qt, QEvent	

from .common import ressourcedir
from .mathutils import *
from .mesh import Mesh
from . import settings
from . import constraints
from . import text
from . import rendering
from . import nprint
from .displays import BoxDisplay

__all__ = ['Screw', 'comomentum', 'Pressure', 'Solid', 'Kinematic', 'Kinemanip', 'solvekin',
			 'Scheme', 'WireDisplay',
			]


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
	def __init__(self, pose=None, **objs):
		if pose:
			self.position = pose[0]
			self.orientation = quat(pose[1])
		else:
			self.position = vec3(0)
			self.orientation = quat()
		self.visuals = objs
	
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
	
	def transform(self, mat):
		''' displace the solid by the transformation '''
		if isinstance(mat, mat4):
			rot, trans = quat_cast(mat3(mat)), vec3(mat[3])
		elif isinstance(mat, mat3):
			rot, trans = quat_cast(mat), 0
		elif isinstance(mat, vec3):
			rot, trans = 1, mat
		else:
			raise TypeError('Screw.transform() expect mat4, mat3 or vec3')
		self.orientation = rot*self.orientation
		self.position += trans
		
	# convenient content access
	def __getitem__(self, key):
		return self.visuals[key]
	def __setitem__(self, key, value):
		self.visuals[key] = value
	def add(self, value):
		key = next(i 	for i in range(len(self.visuals)+1)	
						if i not in self.visuals	)
		self.visuals[key] = value
		return key
	def set(self, **objs):
		self.visuals.update(objs)
		return self
	
	class display(rendering.Group):
		def __init__(self, scene, solid):
			super().__init__(scene, solid.visuals)
			self.solid = solid
			self.apply_pose()
		
		def update(self, scene, solid):
			if not isinstance(solid, Solid):	return
			super().update(scene, solid.visuals)
			self.solid = solid
			self.apply_pose()
			return True
				
		def control(self, view, key, sub, evt):
			if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
				start = view.ptat(view.somenear(evt.pos()))
				offset = self.solid.position - vec3(affineInverse(mat4(self.world)) * vec4(start,1))
				view.tool.append(rendering.Tool(self.move, view, start, offset))
				
		def move(self, dispatcher, view, pt, offset):
			moved = False
			while True:
				evt = yield
				
				if evt.type() == QEvent.MouseMove:
					evt.accept()
					moved = True
					pt = vec3(affineInverse(mat4(self.world)) * vec4(view.ptfrom(evt.pos(), pt),1))
					self.solid.position = pt + offset
					self.apply_pose()
					view.update()
				
				if evt.button() == Qt.LeftButton and evt.type() == QEvent.MouseButtonRelease:
					if moved:	evt.accept()
					break
						
		def apply_pose(self):
			self.pose = fmat4(self.solid.pose)
			
		
			


def solvekin(joints, fixed=(), precision=1e-4, maxiter=None, damping=1):
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
	
	corr = [[] for i in range(len(solids))]
	corrmax = inf
	itercount = 0
	while corrmax > precision**2:
		if maxiter and maxiter <= itercount:
			raise constraints.SolveError('maximum iteration count reached with no solution found, err='+str(sqrt(corrmax)))
		
		for c in corr:
			c.clear()
		corrmax = 0
		# collect corrections
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
			rmax = max(length2(c.position-center)	for c in corrections)
			
			for c in corrections:
				v += c.resulting
				w += c.momentum 	
				# momentum evaluation must take care that the part size can vary a lot, therefore it's reduces by the maximum squared radius
				r = length2(c.position-center) # radius, to evaluate rotation impact geometry shape
				if rmax:
					# there is a coef on w because that correction component is adding energy
					w += 0.25 * cross(c.resulting, center-c.position) / rmax
				corrmax = max(corrmax, length2(c.resulting), length2(c.momentum)*r)
			
			v /= l*2
			w /= l*2
			
			if length2(w) > 0.25:
				w /= length(w)*2
			
			solid.position += v*damping
			solid.orientation = quat(w*damping) * solid.orientation
		
		itercount += 1

def isjoint(obj):
	''' return True if obj is considered to be a kinematic joint object '''
	return hasattr(obj, 'solids') and hasattr(obj, 'corrections')

class Joint:
	class display(rendering.Display):
		def __init__(self, scene, joint):
			self.schemes = [scene.display(joint.scheme(s, 1, joint.position[i]))
							for i,s in enumerate(joint.solids)]
			self.joint = joint
		
		def updateposes(self, view):
			''' update the pose of sub displays using their solid's pose '''
			for sch, solid, pos in zip(self.schemes, self.joint.solids, self.joint.position):
				pos = fvec3(pos)
				m = self.world * fmat4(solid.pose)
				d = (view.uniforms['view'] * m * fvec4(fvec3(pos),1)).z * 30/view.height()
				sch.world = m * translate(scale(translate(fmat4(1), pos), fvec3(d)), -pos)
		
		def stack(self, scene):
			yield ((), 'screen', -1, self.updateposes)
			for i,scheme in enumerate(self.schemes):
				for sub,target,priority,func in scheme.stack(scene):
					yield ((i, *sub), target, priority, func)
		
		def __getitem__(self, sub):
			return self.schemes[sub]
			
		def __iter__(self):
			return iter(self.schemes)


class Kinematic:
	''' Holds a kinematic definition, and methods to use it
		The solid objects used are considered as variables and are modified inplace by methods, and can be modified at any time by outer functions
		The joints are not modified in any case (and must not be modified while a Kinematic is using it)
		
		Attributes defined here:
			* joints	the joints constraints
			* solids	all the solids the joint applys on, and eventually more
			* fixed		the root solids that is considered to be fixed to the ground
	'''
	def __init__(self, joints, fixed=(), solids=None):
		self.joints = joints
		self.solids = solids or getsolids(joints)
		if isinstance(joints, set):	self.fixed = fixed
		else:						self.fixed = set(id(solid) for solid in fixed)
		self.options = {}
		
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
		return [solid.pose 	for solid in self.solids]
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
		return Kinemanip(scene, self)

def getsolids(joints):
	''' return a list of the solids used by the given joints '''
	solids = []
	known = set()
	for joint in joints:
		for solid in joint.solids:
			if id(solid) not in known:
				solids.append(solid)
				known.add(id(solid))
	return solids

class Pressure:
	''' Represent a mechanical pressure repartition on a surface
		equivalent of a Screw but with space repartition
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
	def torsor(self, point) -> Screw:
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


class Kinemanip(rendering.Group):
	''' Display that holds a kinematic structure and allows the user to move it
	'''
	
	def __init__(self, scene, kinematic):
		super().__init__(scene)
		try:	kinematic.solve(maxiter=1000)
		except constraints.SolveError:	pass
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
		except constraints.SolveError as err:	
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
				box.union(display.box)
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
		info[0]['scheme'] = scheme = Scheme([], [], [], [], color)
		center = info[2]/info[3]
		if not isfinite(center):	center = vec3(0)
		for cst in info[1]:
			scheme.extend(cst.scheme(info[0], size, center))


class Scheme:
	''' buffer holder to construct schemes, for now it's only usefull to append to buffer '''
	def __init__(self, points, transpfaces, opacfaces, lines, color=None):
		self.color = color
		self.points = points
		self.transpfaces = transpfaces
		self.opaqfaces = opacfaces
		self.lines = lines
	
	def extend(self, other):
		l = len(self.points)
		self.points.extend(other.points)
		self.transpfaces.extend(((a+l,b+l,c+l) for a,b,c in other.transpfaces))
		self.opaqfaces.extend(((a+l,b+l,c+l) for a,b,c in other.opaqfaces))
		self.lines.extend(((a+l, b+l)  for a,b in other.lines))
	
	def box(self):
		''' return the extreme coordinates of the mesh (vec3, vec3) '''
		return boundingbox(self.points)
	
	def display(self, scene):
		return WireDisplay(scene, self.points, self.transpfaces, self.opaqfaces, self.lines, self.color)


class WireDisplay(rendering.Display):
	''' wireframe display for schemes, like kinematic schemes '''
	
	def __init__(self, scene, points, transpfaces, opaqfaces, lines, color):
		ctx = scene.ctx
		self.color = fvec3(color or settings.display['schematics_color'])
		self.box = boundingbox(points).cast(fvec3)
		
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
		self.identshader = scene.ressource('shader_ident')
		
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
					self.identshader,
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
					self.identshader,
					[(self.vb_vertices, '3f4 12x', 'v_position')],
					self.vb_lines,
					)
		else:
			self.vb_lines = None
	
	def render(self, view):		
		viewmat = view.uniforms['view'] * self.world
		color = self.color if not self.selected else settings.display['select_color_line']
		ctx = view.scene.ctx
		
		self.uniformshader['color'].write(fvec4(color,1))
		self.uniformshader['view'].write(viewmat)
		self.uniformshader['proj'].write(view.uniforms['proj'])
		
		ctx.disable(mgl.DEPTH_TEST)
		ctx.disable(mgl.CULL_FACE)
		if self.vb_opaqfaces:	self.va_opaqfaces.render(mgl.TRIANGLES)
		if self.vb_lines:		self.va_lines.render(mgl.LINES)
		
		ctx.enable(mgl.CULL_FACE)
		if self.vb_transpfaces:
			self.transpshader['color'].write(color)
			self.transpshader['view'].write(viewmat)
			self.transpshader['proj'].write(view.uniforms['proj'])
			self.va_transpfaces.render(mgl.TRIANGLES)
		ctx.enable(mgl.DEPTH_TEST)
		ctx.disable(mgl.CULL_FACE)
	
	def identify(self, view):
		viewmat = view.uniforms['view'] * self.world
		self.identshader['ident'] = view.identstep(1)
		self.identshader['view'].write(viewmat)
		self.identshader['proj'].write(view.uniforms['proj'])
		
		ctx = view.scene.ctx
		ctx.blend_func = mgl.ZERO, mgl.ONE
		if self.vb_transpfaces:		self.va_ident_faces.render(mgl.TRIANGLES)
		if self.vb_lines:			self.va_ident_lines.render(mgl.LINES)

	def stack(self, scene):
		if not scene.options['display_annotations']:
			return ()
		return ( ((), 'screen', 1, self.render),
				 ((), 'ident', 2, self.identify))


