import numpy.core as np
import moderngl as mgl

from .mathutils import (fvec3, fmat4, vec3, vec4, mat3, mat4, quat, mat4_cast, quat_cast,
						column, translate, inverse, isnan,
						dot, cross, length, normalize, project, noproject, dirbase, distance,
						atan2, acos, angle, axis, angleAxis,
						Box, transform)
from .mesh import Mesh, Wire, web
from . import generation
from . import primitives
from . import settings
from . import constraints


'''
onesolid = Solid(mesh1, base=R0)	# solide avec visuel, maillage a transformer pour passer dans la base adhoc
othersolid = Solid(mesh2, pose=R1)	 # solide avec visuel, maillage prétransformé
onceagain = Solid()                 # solide vide, base bar defaut (transformation = I4)
csts = [
	Pivot(onesolid, othersolid, (O,x), (O,z)),
	Linear(onesolid, othersolid, (C,x), C),
	Plane(onesolid, othersolid, z, x),	# plan
	Screw(onesolid, othersolid, (B,x), (D,y), 0.2),	# helicoidale
	Gear(onesolid, onceagain, (O,z), (O,z), 2, offset=0),	# engrenage
	Rack(onesolid, onceagain, (O,z), x, 1, offset=0),	# cremaillere
	Track(onesolid, othersolid, x, y),		# glissiere
	InSolid(onesolid, A, B, C, D, E),	# points sur solide
	]
'''

'''
class Torsor:
	__slots__ = ('momentum', 'resulting', 'position')
	def __init__(self, momentum, resulting, position):
		self.momentum, self.resulting, self.position = momentum, resulting, position
	def locate(self, pt):
		return Torsor(self.momentum, self.resulting + cross(self.momentum, pt-self.position), pt)
	#def transform(self):
		# donne la matrice de l'application affine de la position vers la resultante
		#return transform(self.position) * transform(self.resulting, self.momentum) * transform(-self.position)

def comomentum(t1, t2):
	t2 = t2.locate(t1.position)
	return dot(t1.momentum, t2.resulting) + dot(t2.momentum, t1.resulting)
'''


class Solid:
	'''
		orientation - quat
		position - vec3
		visuals	- list of objects to display using the solid's pose
		name - optional name to display
	'''
	def __init__(self, *args, base=None, pose=None, name=None):
		if pose:
			self.position = pose[0]
			self.orientation = pose[1]
		else:
			self.position = vec3(0)
			self.orientation = vec3(0)
		self.visuals = list(args)
		self.name = name
	slvvars = 'position', 'orientation',
	
	def transform(self):
		return transform(self.position, self.orientation)
	
	def display(self, scene):
		for visu in self.visuals:
			yield from visu.display(scene)
		
	
class InSolid:	# TODO test
	def __init__(self, solid, points, reference=None):
		self.solid = solid
		self.points = points
		self.reference = reference or [p*solid.transform() for p in points]	# if no reference position is provided, use the current position in the solid
		# remove origin from references
		center = sum(reference)/len(reference)
		for p in self.reference:	p -= center
	
	def slvvars(self):
		return (self.solid.position, self.solid.orientation, *self.points)

	def pose(self):
		center = sum(self.points)/len(self.points)
		rot = vec3(0)
		for pt,ref in zip(self.points, self.reference):
			v, r = normalize(pt-ref), normalize(ref)
			c = cross(v,r)
			if not isnan(c):
				angle = acos(dot(v,r))
				rot += angleAxis(angle, c/sin(angle))
		rot = normalize(rot)
		#print('  pose', center, rot)
		return self.solid.position+center, self.solid.orientation+rot

cornersize = 0.1
GAPROT = 0.5

def solidtransform_axis(solid, obj):
	rot = quat(solid.orientation)
	return (rot*obj[0] + solid.position, rot*obj[1])

class Pivot:
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1,s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	slvvars = 'solids',
	
	def corrections(self):
		axis = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		r = cross(axis[0][1], axis[1][1])
		t = axis[1][0] - axis[0][0]
		return Torsor(r,t,axis[0][0]), Torsor(-r,-t,axis[1][0])
	
	def fit(self):
		orients = (quat(self.solids[0].orientation), quat(self.solids[1].orientation))
		a1 = (orients[0] * self.axis[0][0] + self.solids[0].position,  orients[0] * self.axis[0][1])
		a2 = (orients[1] * self.axis[1][0] + self.solids[1].position,  orients[1] * self.axis[1][1])
		return distance(a1[0],a2[0])**2 - dot(a1[1],a2[1])
	
	def freedom(self):
		orient = quat(self.solids[0].orientation)
		return Torsor(orient * self.axis[0][1], vec3(0), orient * self.axis[0][0] + self.solids[0].position)
		
	def scheme(self, solid, size, junc):
		''' return primitives to render the junction from the given solid side
			size is the desired size of the junction
			junc is the point the junction is linked with the scheme by
		'''
		if solid is self.solids[0]:
			radius = size/4
			axis = self.axis[0]
			center = axis[0] + project(self.position[0], axis[1])
			cyl = generation.extrusion(
						axis[1]*size * 0.8, 
						web(primitives.Circle(
								(center-axis[1]*size*0.4, axis[1]), 
								size/4, 
								resolution=('div', 16),
						)))
			l = len(cyl.points)
			v = junc - center
			v = normalize(v - project(v,axis[1]))
			p = center + v*size/4
			cyl.points.append(p)
			cyl.points.append(p + axis[1]*size*cornersize)
			cyl.points.append(p + v*size*cornersize)
			cyl.points.append(junc)
			return Scheme(
					cyl.points, 
					cyl.faces, 
					[(l,l+1,l+2)], 
					(	[(l, l+2), (l+2, l+3)] 
					+	[(i-1,i) for i in range(1,l//2)] 
					+	[(i-1,i) for i in range(l//2+1,l)] 
					+	[(l//2-1,0), (l-1,l//2)]),
					)
		elif solid is self.solids[1]:
			radius = size/4
			axis = self.axis[1]
			center = axis[0] + project(self.position[1], axis[1])
			side = axis[1] * size * 0.5
			if dot(junc-axis[0], axis[1]) < 0:
				side = -side
			if dot(junc-center-side, side) < 0:
				attach = side + normalize(noproject(junc-center, axis[1]))*radius
			else:
				attach = side
			c1 = primitives.Circle(
							(center-axis[1]*size*0.5, -axis[1]), 
							radius, 
							resolution=('div', 16),
							).mesh()
			c2 = primitives.Circle(
							(center+axis[1]*size*0.5, axis[1]), 
							radius, 
							resolution=('div', 16),
							).mesh()
			s1 = generation.flatsurface(c1)
			s2 = generation.flatsurface(c2)
			l = len(c1.points)
			indices = [(i-1,i)  for i in range(1,l-1)]
			indices.append((l-1,0))
			
			s = Scheme([center-side, center+side, center+attach, junc], [], [], [(0,1), (2,3)])
			s.extend(Scheme(c1.points, s1.faces, [], c1.edges()))
			s.extend(Scheme(c2.points, s2.faces, [], c2.edges()))
			return s

class Plane:
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1, s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	slvvars = 'solids',
		
	def corrections(self):
		#print('  --')
		orients = (quat(self.solids[0].orientation), quat(self.solids[1].orientation))
		a1 = (orients[0] * self.axis[0][0] + self.solids[0].position,  orients[0] * self.axis[0][1])
		a2 = (orients[1] * self.axis[1][0] + self.solids[1].position,  orients[1] * self.axis[1][1])
		#if dot(a1[1], a2[1]) < -0.2:
			#print('\tpush')
			#normal = a1[1]
			#rot = vec3(3.14,0,0)
		#else:							
		normal = normalize(a1[1] + a2[1])
		rot = cross(a1[1],a2[1])
		gap = project(a1[0] - a2[0], normal)
		#print('  a1', a1)
		#print('  a2', a2)
		#print('\tgap', gap)
		#print('\trot', rot)
		
		r = (orients[0]*self.axis[0][0], orients[1]*self.axis[1][0])
		s = (-gap - GAPROT*noproject(cross(rot, r[0]), normal),
			  rot + GAPROT*project(cross(r[0],-gap) / (dot(r[0],r[0]) + 1e-15), normal),
			  gap - GAPROT*noproject(cross(-rot, r[1]), normal),
			 -rot + GAPROT*project(cross(r[1],gap) / (dot(r[1],r[1]) + 1e-15), normal),
			)
		#print('  plane', s)
		return s
	
	def corrections(self):
		axis = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		r = cross(axis[0][1], axis[1][1])
		t = project(axis[1][0] - axis[0][0], normalize(axis[0][1] + axis[1][1]))
		return Torsor(r,t,axis[0][0]), Torsor(-r,-t,axis[1][0])
	
	def fit(self):
		orients = (quat(self.solids[0].orientation), quat(self.solids[1].orientation))
		a1 = (orients[0] * self.axis[0][0] + self.solids[0].position,  orients[0] * self.axis[0][1])
		a2 = (orients[1] * self.axis[1][0] + self.solids[1].position,  orients[1] * self.axis[1][1])
		normal = normalize(a1[1] + a2[1])
		return dot(a1[0]-a2[0], normal) **2 - dot(a1[1], a2[1])
		
	
	def scheme(self, solid, size, junc):
		if   solid is self.solids[0]:		(center, normal), position = self.axis[0], self.position[0]
		elif solid is self.solids[1]:		(center, normal), position = self.axis[1], self.position[1]
		else:	return
		center = position - project(position - center, normal)
		if dot(junc-center, normal) < 0:	normal = -normal
		if solid is self.solids[1]:			normal = -normal
		x,y,z = dirbase(normal)
		c = center + size*0.1 * z
		x *= size*0.7
		y *= size*0.7
		return Scheme(
			[c+(x+y), c+(-x+y), c+(-x-y), c+(x-y), c, c+cornersize*x, c+cornersize*z, junc],
			[(0,1,2), (0,2,3)],
			[(4,5,6)],
			[(0,1),(1,2),(2,3),(3,0),(4,6),(6,7)],
			)

class Near:
	def __init__(self, solid, pt, ref):
		self.solid, self.pt, self.ref = solid, pt, ref
		self.lastref = vec3(self.solid.transform() * vec4(self.ref,1))
	
	def params(self):
		return self.solid.position, self.solid.orientation
	
	def corrections(self):
		r = quat(self.solid.orientation) * self.ref
		ref = r + self.solid.position
		free = normalize(ref - self.lastref)		# approximation for the free direction
		print('free', ref - self.lastref)
		gap = self.pt - ref
		if not isnan(free):
			gap = project(gap, free)
		rot = cross(r, gap) / (dot(r,r) + 1e-15)
		#rot = vec3(0)
		#self.lastref = ref*0.2 + self.lastref*0.8
		return 0.2*gap, 0.2*rot
		

def mkwiredisplay(solid, constraints, size=None, color=None):
	# get center and size of the solid's scheme
	center = vec3(0)
	contrib = 0
	box = Box()
	for cst in constraints:
		if solid in cst.solids and cst.position:
			print(cst.position)
			pos = cst.position[cst.solids.index(solid)]
			contrib += 1
			center += pos
			box.union(pos)
	center /= contrib
	if not size:	size = length(box.width)/len(constraints) or 1
	print(size)
	
	# assemble junctions
	scheme = Scheme([], [], [], [], color, solid.transform())
	for cst in constraints:
		part = cst.scheme(solid, size, center)
		if part:	scheme.extend(part)
	
	return scheme

from .mathutils import glm
def makescheme(constraints, color=None):
	# collect solids informations
	solids = {}
	diag = vec3(0)
	for cst in constraints:
		for solid, pos in zip(cst.solids, cst.position):
			if id(solid) not in solids:
				solids[id(solid)] = info = [solid, [], vec3(0), 0]
			info[1].append(cst)
			v = pos - solid.position
			info[2] += v
			info[3] += 1
			diag = glm.max(diag, glm.abs(v))
	# get the junction size
	size = (max(diag) or 1) / len(constraints)
	
	for info in solids.values():
		scheme = Scheme([], [], [], [], color, solid.transform())
		center = info[2]/info[3]
		for cst in info[1]:
			scheme.extend(cst.scheme(info[0], size, center))
		info[0].visuals.append(scheme)



class Scheme:
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
	
	def display(self, scene):
		return WireDisplay(scene, self.points, self.transpfaces, self.opaqfaces, self.lines, self.color, self.transform),


class WireDisplay:
	renderindex = 1
	
	def __init__(self, scene, points, transpfaces, opaqfaces, lines, color=None, transform=fmat4(1)):
		ctx = scene.ctx
		self.transform = fmat4(*transform)
		self.color = fvec3(color or settings.display['wire_color'])
		
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open('shaders/uniformcolor.vert').read(),
						fragment_shader=open('shaders/uniformcolor.frag').read(),
						)
		self.uniformshader = scene.ressource('shader_uniformcolor', load)
		
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open('shaders/glowenvelope.vert').read(),
						fragment_shader=open('shaders/glowenvelope.frag').read(),
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
		
		scene.ctx.enable(mgl.DEPTH_TEST)
		scene.ctx.disable(mgl.CULL_FACE)
		if self.vb_opaqfaces:	self.va_opaqfaces.render(mgl.TRIANGLES)
		if self.vb_lines:		self.va_lines.render(mgl.LINES)
		
		scene.ctx.disable(mgl.DEPTH_TEST)
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

from math import inf
def solvekin(joints, fixed=(), precision=1e-4, maxiter=None, damping=0.5):
	# register solids and corrections
	solids = []
	register = {}
	counts = []
	indices = []
	fixed = set((id(solid) for solid in fixed))
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
		#corrmax = 0
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
			for c in corrections:
				v += c.momentum
				center += c.position
			v /= l
			center /= l
			for c in corrections:
				r = length(c.position-center) + 1e-15
				induced = cross(c.momentum-v, c.position-center) / r**2
				lind = length(induced)
				if lind > 1:	induced /= lind
				w += c.resulting - induced
				corrmax = max(corrmax, length(c.momentum), length(c.resulting)*r)
			w /= 2*l
			'''
			for c in corrections:
				center += c.position
			center /= l
			for c in corrections:
				v += c.locate(center).momentum
				w += c.resulting
			v /= l
			w /= l
			w2 = vec3(0)
			tot = 0
			for c in corrections:
				r = length(c.position-center) + 1e-15
				w2 += cross(c.momentum - v, c.position-center) /r
				tot += r
			w = (w - w2/tot)/2
			
			solid.position += v*damping
			#solid.orientation += w*damping
			#solid.orientation += inverse(quat(solid.orientation)) * (w*damping)
			nrot = quat(w*damping) * quat(solid.orientation)
			solid.orientation = angle(nrot)*axis(nrot)
			
		if maxiter and maxiter == itercount:	
		
			print()
			for solid in solids:
				print(solid.position, solid.orientation)
			return
			#raise constraints.SolveError('maximum iteration count reached with no solution found, err='+str(corrmax))
		itercount += 1
	
	
	
class Torsor:
	__slots__ = ('resulting', 'momentum', 'position')
	def __init__(self, resulting=None, momentum=None, position=None):
		self.resulting, self.momentum, self.position = resulting or vec3(0), momentum or vec3(0), position or vec3(0)
	def locate(self, pt):
		return Torsor(self.momentum, self.resulting + cross(self.momentum, pt-self.position), pt)
	#def transform(self):
		# donne la matrice de l'application affine de la position vers la resultante
		#return transform(self.position) * transform(self.resulting, self.momentum) * transform(-self.position)
	
	def __add__(self, other):
		if other.position != self.position:		other = other.locate(self.position)
		return Torsor(self.resulting+other.resulting, self.momentum+other.momentum, self.position)
	
	def __sub__(self, other):
		if other.position != self.position:		other = other.locate(self.position)
		return Torsor(self.resulting-other.resulting, self.momentum-other.momentum, self.position)
	
	def __neg__(self):
		return Torsor(-self.resulting, -self.momentum, self.position)

def comomentum(t1, t2):
	t2 = t2.locate(t1.position)
	return dot(t1.momentum, t2.resulting) + dot(t2.momentum, t1.resulting)


from PyQt5.QtCore import QEvent	
from . import view, text
from copy import copy

def store(dst, src):
	for i in range(len(dst)):
		dst[i] = src[i]

class Kinemanip:
	''' Display that holds a kinematic structure and allow the user to move it
	'''
	renderindex = 0
	
	def __init__(self, scene, csts, root, scheme=False):
		self.csts, self.root = csts, root
		self.locked = {id(root): root}
		self.primitives = {}
		self.debug_label = None
		self.reproblem()
		if scheme:
			makescheme(self.csts)
	
	def display(self, scene):
		for cst in self.csts:
			for prim in cst.solids:
				if id(prim) not in self.primitives:
					displays = list(prim.display(scene))
					self.primitives[id(prim)] = (prim, displays)
					for disp in displays:
						disp.control = lambda scene, grp, ident, evt, solid=prim:	self.start(solid, scene, ident, evt)	# lambda default argument uncapture the variable
						yield disp
		self.debug_label = text.TextDisplay(scene, vec3(0,0,0), "mouse", 8, (1,1,1))
		yield self.debug_label
	
	def render(self, scene):	pass
	def identify(self, scene, startident):	pass	
		
	def start(self, solid, scene, ident, pt):
		if solid is self.root:	return
		
		self.startpt = scene.ptat(pt)
		self.debug_label.position = fvec3(self.startpt)
		self.ptoffset = inverse(quat(solid.orientation)) * (solid.position - self.startpt)
		self.actsolid = solid
		self.moved = False
		return self.move
	
	def move(self, scene, evt):
		# we use the conjugated gradient algorithm, that seems more 
		# - CPU efficient for situations starting near to the solution 
		# - RAM efficient for larg kinematic problems (with tons of variables)
		method = 'CG'
		
		if evt.type() == QEvent.MouseMove:
			# unlock moving solid
			self.lock(self.actsolid, False)
			self.moved = True
			# displace the moved object
			pt = scene.ptfrom((evt.x(), evt.y()), self.startpt)
			self.debug_label.position = fvec3(pt)
			store(self.actsolid.position, pt+quat(self.actsolid.orientation)*self.ptoffset)
			# solve
			#try:	self.problem.solve(precision=1e-3, method=method, maxiter=50)
			try:	solvekin(self.csts, self.locked.values(), precision=1e-3, maxiter=50)
			except constraints.SolveError as err:	print(err)
		else:
			if not self.moved:
				self.lock(self.actsolid, id(self.actsolid) not in self.locked)
			else:
				# finish on a better precision
				try:	self.problem.solve(precision=1e-6, method=method)
				except constraints.SolveError as err:	print(err)
			scene.tool = None
		
		# assign new positions to displays
		for primitive,displays in self.primitives.values():
			trans = fmat4(*primitive.transform())
			for disp in displays:
				if hasattr(disp, 'transform'):
					disp.transform = trans
		scene.update()
		return True
	
	def lock(self, solid, lock):
		if solid is self.root:	return
		if lock:
			if id(solid) in self.locked:	return
			# add solid's variables to fixed
			self.locked[id(solid)] = solid
			# grey display for locked solid
			for disp in self.primitives[id(solid)][1]:
				disp.color = fvec3(0.5, 0.5, 0.5)
		else:
			if id(solid) not in self.locked:	return
			# remove solid's variables from fixed
			if id(solid) in self.locked:	del self.locked[id(solid)]
			# reset solid color
			for disp in self.primitives[id(solid)][1]:
				disp.color = fvec3(settings.display['schematics_color'])
		self.reproblem()
	
	def reproblem(self):
		fixed = []
		for locked in self.locked.values():
			fixed.append(locked.position)
			fixed.append(locked.orientation)
		self.problem = constraints.Problem(self.csts, fixed)
