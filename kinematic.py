from mathutils import fvec3, fmat4, vec3, vec4, mat3, mat4, quat, mat4_cast, quat_cast, \
						column, translate, inverse, isnan, \
						dot, cross, length, normalize, project, dirbase, \
						atan2, acos, angle, axis, angleAxis, \
						Box, transform
from mesh import Mesh
import generation
import primitives
import settings
import constraints
import numpy.core as np
import moderngl as mgl


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
	
	def params(self):
		return (self.solid.position, self.solid.orientation, *self.points)
	
	def corrections(self):
		center, rot = self.pose()
		corr = [rot*ref + center - pt    for pt,ref in zip(self.points, self.reference)]
		s = (center-self.solid.position, rot-self.solid.orientation, *corr)
		print('  insolid',s)
		return s
	'''
	def pose(self):
		center = sum(self.points)/len(self.points)
		rot = quat()
		for pt,ref in zip(self.points, self.reference):
			v, r = normalize(pt-ref), normalize(ref)
			c = cross(v,r)
			if not isnan(c):
				angle = acos(dot(v,r))
				print('  dot', dot(v,r), angle, c)
				rot += angleAxis(angle, c/sin(angle))
		rot = normalize(rot)
		#print('  pose', center, rot)
		return self.solid.position+center, self.solid.orientation+rot
	'''
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

class Pivot:
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1,s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	def params(self):
		return self.solids[0].position, self.solids[0].orientation, self.solids[1].position, self.solids[1].orientation

	def corrections(self):
		#print('  --')
		orients = (quat(self.solids[0].orientation), quat(self.solids[1].orientation))
		a1 = (orients[0] * self.axis[0][0] + self.solids[0].position,  orients[0] * self.axis[0][1])
		a2 = (orients[1] * self.axis[1][0] + self.solids[1].position,  orients[1] * self.axis[1][1])
		normal = normalize(a1[1] + a2[1])
		rot = cross(a1[1],a2[1])
		gap = a1[0] - a2[0]
		#print('  a1', a1)
		#print('  a2', a2)
		
		r = (orients[0]*self.axis[0][0], orients[1]*self.axis[1][0])
		s = (-gap,
			  rot + GAPROT*project(cross(r[0],-gap) / (dot(r[0],r[0]) + 1e-15), normal),
			  gap,
			 -rot + GAPROT*project(cross(r[1],gap) / (dot(r[1],r[1]) + 1e-15), normal),
			)
		#print('  pivot', s)
		return s
	
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
						generation.outline(primitives.Circle(
								(center-axis[1]*size*0.4, axis[1]), 
								size/4, 
								resolution=('div', 16),
						)))
			cyl.mergedoubles()
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
					[(l, l+2), (l+2, l+3)] + [(i-1,i) for i in range(1,l//2)] + [(i-1,i) for i in range(l//2+1,l)],
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
			c1 = generation.flatsurface(generation.outline(primitives.Circle(
							(center-axis[1]*size*0.5, -axis[1]), 
							radius, 
							resolution=('div', 16),
							)))
			c2 = generation.flatsurface(generation.outline(primitives.Circle(
							(center+axis[1]*size*0.5, axis[1]), 
							radius, 
							resolution=('div', 16),
							)))
			indices = [(i-1,i)  for i in range(1,len(c1.points))]
			s = Scheme([center-side, center+side, center+attach, junc], [], [], [(0,1), (2,3)])
			s.extend(Scheme(c1.points, c1.faces, [], indices))
			s.extend(Scheme(c2.points, c2.faces, [], indices))
			return s

class Plane:
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1, s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	def params(self):
		return self.solids[0].position, self.solids[0].orientation, self.solids[1].position, self.solids[1].orientation
		
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
	
	#def freedom(self):
		#return (self.solids[0].orientation, self.axis[0][1]), (self.solids[0].position, dirbase(self.axis[0][1])[:2])
	
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
		
				
def noproject(x,dir):	return x - project(x,dir)

def mkwiredisplay(solid, constraints, size=None, color=None):
	# get center and size of the solid's scheme
	center = vec3(0)
	contrib = 0
	box = Box()
	for cst in constraints:
		if solid in cst.solids and cst.position:
			contrib += 1
			center += cst.position[cst.solids.index(solid)]
			box.union(cst.position)
	center /= contrib
	if not size:	size = length(box.width)/len(constraints) or 1
	
	# assemble junctions
	scheme = Scheme([], [], [], [], color, solid.transform())
	for cst in constraints:
		part = cst.scheme(solid, size, center)
		if part:	scheme.extend(part)
	
	return scheme

def faceoffset(o):
	return lambda f: (f[0]+o, f[1]+o, f[2]+o)



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
		#self.opacfaces.extend(map(faceoffset(l), other.opaqfaces))		# create a faceoffset function
		#self.opacfaces.extend([f+l for f in other.opaqfaces])		# use ivec as faces and edges
		self.transpfaces.extend(((a+l,b+l,c+l) for a,b,c in other.transpfaces))
		self.opaqfaces.extend(((a+l,b+l,c+l) for a,b,c in other.opaqfaces))
		self.lines.extend(((a+l, b+l)  for a,b in other.lines))
	
	def display(self, scene):
		return WireDisplay(scene, self.points, self.transpfaces, self.opaqfaces, self.lines, self.color, self.transform),


class WireDisplay:
	renderindex = 1
	
	def __init__(self, scene, points, transpfaces, opaqfaces, lines, color=None, transform=None):
		ctx = scene.ctx
		self.transform = fmat4(transform or 1)
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
	
	def identify(self, scene, startident):
		viewmat = scene.view_matrix * self.transform
		scene.ident_shader['ident'] = startident
		scene.ident_shader['view'].write(viewmat)
		scene.ident_shader['proj'].write(scene.proj_matrix)
		
		if self.vb_transpfaces:		self.va_ident_faces.render(mgl.TRIANGLES)
		if self.vb_lines:			self.va_ident_lines.render(mgl.LINES)
		return 1



from PyQt5.QtCore import QEvent	
import view, text
from copy import copy

class ToolMove:
	renderindex = 0
	
	def __init__(self, scene, csts, root):
		self.csts, self.root = csts, root
		self.fixed = {id(self.root.position), id(self.root.orientation)}
		
		# build displays for params, keeping there indices to access their pose later
		self.params = []
		for cst in csts:
			self.params.extend(cst.solids)
		self.displays = []
	
	def display(self, scene):
		for param in self.params:
			grp = list(param.display(scene))
			self.displays.append(grp)
			for disp in grp:
				disp.control = lambda scene, grp, ident, evt, param=param:	self.start(param, scene, ident, evt)	# lambda default argument uncapture the variable
				yield disp
	
	def render(self, scene):	pass
	def identify(self, scene, startident):	pass	
		
	def start(self, solid, scene, ident, pt):
		if solid is self.root:	return
		
		point = scene.ptat(pt)
		print(point)
		
		self.startpt = point
		self.startpos = copy(solid.position)
		self.actsolid = solid
		self.moved = False
		
		#self.placement = Near(solid, point, vec3(inverse(solid.transform()) * vec4(point,1)))
		#self.csts.append(self.placement)
		return self.move
	
	def move(self, scene, evt):
		if evt.type() == QEvent.MouseMove:
			if self.actsolid is not self.root:
				self.lock(self.actsolid, False)
				self.moved = True
			
			pt = scene.ptfrom((evt.x(), evt.y()), self.startpt)
			self.actsolid.position = pt - self.startpt + self.startpos
			
			try:	constraints.solve(self.csts, fixed=self.fixed, precision=1e-3, maxiter=50)
			except constraints.SolveError as err:
				print(err)
			#for p in self.params:
				#print(id(p), p.position, p.orientation)
			
			i = 0
			for param,grp in zip(self.params, self.displays):
				trans = param.transform()
				for disp in grp:
					if isinstance(disp, (view.SolidDisplay, WireDisplay)):
						disp.transform = trans
					elif isinstance(disp, text.Text):
						disp.position = trans * disp.position
		else:
			if not self.moved:
				self.lock(self.actsolid, True)
				scene.update()
			#self.csts.pop()
			scene.tool = None
	
	def lock(self, solid, lock):
		if lock:
			self.fixed.add(id(self.actsolid.position))
			self.fixed.add(id(self.actsolid.orientation))
			for param,grp in zip(self.params, self.displays):
				if param is self.actsolid:
					for disp in grp:
						disp.color = fvec3(0.5, 0.5, 0.5)
		else:
			self.fixed.discard(id(self.actsolid.position))
			self.fixed.discard(id(self.actsolid.orientation))
			for param,grp in zip(self.params, self.displays):
				if param is self.actsolid:
					for disp in grp:
						disp.color = fvec3(settings.display['schematics_color'])
	

def test_constraints():
	# test kinematic solver
	from constraints import solve
	
	s0 = Solid()
	
	print('\ntest simple pivot')
	s1 = Solid(pose=(vec3(0), 2*vec3(1,1,0)))
	csts = [Pivot(s0,s1, (vec3(0,0,0), vec3(0,1,0)))]
	solve(csts,
			fixed={id(s0.position), id(s0.orientation)}, 
			afterset=lambda: print(s0.orientation, s1.orientation))
	
	print('\ntest simple plane')
	s1 = Solid(pose=(vec3(0), 2*vec3(1,1,0)))
	csts = [Plane(s0,s1, (vec3(0,0,0), vec3(0,1,0)))]
	solve(csts,
			fixed={id(s0.position), id(s0.orientation)}, 
			afterset=lambda: print(s0.orientation, s1.orientation))
	
	print('\ntest plane and pivot')
	s1 = Solid()
	s2 = Solid(pose=(vec3(2,0,1.5), 2*vec3(1,1,0)))
	csts = [
		Plane(s0,s1, (vec3(0), vec3(0,0,1))),  
		Pivot(s1,s2, (vec3(0,0,1), vec3(0,1,0)), (vec3(0,0,-1), vec3(0,1,0))),
		]
	solve(csts,
			fixed={id(s0.position), id(s0.orientation), id(s2.position)}, 
			afterset=lambda: print(' ', s1.position, '\t', s1.orientation, '\n ', s2.position, '\t', s2.orientation),
			maxiter=1000,
			precision=1e-4)

	'''
	print('\n test pivots and shared point')
	s0 = Solid()
	s1 = Solid()
	s2 = Solid()
	A = vec3(0.1, 0.2, 0.3)
	csts = [
		Pivot(s0,s1, (vec3(1,0,0),vec3(0,1,0))),
		Pivot(s0,s2, (vec3(-1,0,0),vec3(0,1,0))),
		InSolid(s1, [A], [vec3(0,0,1)]),
		InSolid(s2, [A], [vec3(0,0,1)]),
		]
	solve(csts, 
			fixed={id(s0.position), id(s0.orientation)}, 
			afterset=lambda: print(s1.orientation, s2.orientation))
	
	l12 = Pivot((0,x))
	l23 = Pivot((A,x))
	csts = [
		Solid(
	'''

def test_display():
	print('\n test display')
	s1 = Solid()
	s2 = Solid(pose=(vec3(0,0,0.5), vec3(1,0,0)))
	s3 = Solid()
	A = vec3(2,0,0)
	B = vec3(0,2,0)
	C = vec3(0,-1,1)
	x = vec3(1,0,0)
	y = vec3(0,1,0)
	csts = [Pivot(s1,s2, (A,x)), Pivot(s1,s2, (B,y)), Plane(s2,s3, (C,y), position=(C,C))]
	sc1 = mkwiredisplay(s1, csts, 1, color=(1, 1, 1))
	sc2 = mkwiredisplay(s2, csts, 1)
	sc3 = mkwiredisplay(s3, csts, 1)
	
	import sys
	from PyQt5.QtWidgets import QApplication
	from view import Scene
	
	app = QApplication(sys.argv)
	scn = Scene()
	scn.objs.append(sc1)
	scn.objs.append(sc2)
	scn.objs.append(sc3)
	
	scn.show()
	sys.exit(app.exec())

def test_manip():
	s0 = Solid()
	s1 = Solid()
	s2 = Solid(pose=(vec3(2,0,1.5), 2*vec3(1,1,0)))
	s3 = Solid()
	s4 = Solid()
	s5 = Solid()
	csts = [
		Plane(s0,s1, (vec3(0), vec3(0,0,1))),  
		Pivot(s1,s2, (vec3(0,0,1), vec3(1,0,0)), (vec3(0,0,-1), normalize(vec3(1,1,0)))),
		#Pivot(s2,s3, (vec3(0,0,2), normalize(vec3(1,1,0))), (vec3(0,0,0), vec3(1,0,0))),
		
		#Pivot(s1,s2, (vec3(0,0,1), vec3(1,0,0)), (vec3(0,0,-1), normalize(vec3(1,0,0)))),
		Pivot(s2,s3, (vec3(0,0,2), normalize(vec3(1,0,0))), (vec3(0,0,0), vec3(1,0,0))),
		Pivot(s3,s4, (vec3(0,0,2), normalize(vec3(1,0,0))), (vec3(0,0,0), vec3(1,0,0))),
		Pivot(s4,s5, (vec3(0,0,2), normalize(vec3(1,0,0))), (vec3(0,0,0), vec3(1,0,0))),
		]
		
	import sys
	from PyQt5.QtWidgets import QApplication
	from view import Scene
	
	app = QApplication(sys.argv)
	scn = Scene()
	s0.visuals.append(mkwiredisplay(s0, csts))
	s1.visuals.append(mkwiredisplay(s1, csts, color=(1,0.4,0.2)))
	s2.visuals.append(mkwiredisplay(s2, csts))
	s3.visuals.append(mkwiredisplay(s3, csts))
	s4.visuals.append(mkwiredisplay(s4, csts))
	s5.visuals.append(mkwiredisplay(s5, csts))
	#display_mechanism(scn, csts, s0)
	scn.add(ToolMove(scn, csts, s0))
	scn.show()
	sys.exit(app.exec())
	

if __name__ == '__main__':
	test_constraints()
	test_manip()
	#test_display()

