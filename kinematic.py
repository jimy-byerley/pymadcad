from mathutils import fvec3, fmat4, vec3, vec4, mat3, mat4, quat, mat4_cast, quat_cast, \
						column, translate, inverse, isnan, \
						dot, cross, length, normalize, project, dirbase, \
						atan2, acos, angle, axis, angleAxis, Box
from mesh import Mesh
import generation
import primitives
import settings
import numpy.core as np
import moderngl as mgl


class Torsor:
	__slots__ = ('momentum', 'resulting', 'position')
	def locate(pt):
		return Torsor(self.momentum, self.resulting + cross(self.momentum, pt-self.position))

class Solid:
	'''
		orientation - quat
		position - vec3
		mesh
	'''
	def __init__(self, *args, base=None, pose=None):
		if pose:
			self.position = pose[0]
			self.orientation = pose[1]
		else:
			self.position = vec3(0)
			self.orientation = vec3(0)
		self.visuals = args
	
	def transform(self):
		return mktransform(self.position, self.orientation)
	
	def display(self, scene):
		if mesh:	return SolidMeshDisplay(self.mesh)
		else:		return None
		
	
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
				print('  dot', dot(v,r), angle, c)
				rot += angleAxis(angle, c/sin(angle))
		rot = normalize(rot)
		#print('  pose', center, rot)
		return self.solid.position+center, self.solid.orientation+rot

def mktransform(*args):
	if len(args) == 1 and isinstance(args[0], tuple):
		args = args[0]
	if len(args) == 1:
		if isinstance(args[0], mat4):	return args[0]
		elif isinstance(args[0], mat3):	return mat4(args[0])
		elif isinstance(args[0], quat):	return mat4_cast(args[0])
		elif isinstance(args[0], vec3):	return translate(mat4(1), args[0])
	elif len(args) == 2:
		if   isinstance(args[0], vec3) and isinstance(args[1], mat3):	return translate(mat4(args[1]), args[0])
		elif isinstance(args[0], vec3) and isinstance(args[1], quat):	return translate(mat4_cast(args[1]), args[0])
		elif isinstance(args[0], vec3) and isinstance(args[1], vec3):	return translate(mat4_cast(quat(args[1])), args[0])
	elif isinstance(args[0], vec3) and len(args) == 3:			return mat4(mat3(args))
	elif isinstance(args[0], vec3) and len(args) == 4:			return translate(mat4(mat3(args[1:])), args[0])
	
	raise TypeError('a transformation must be a  mat3, mat4, quat, (O,mat3), (O,quat), (0,x,y,z)')

def rotbt(dir1, dir2):
	axis = cross(dir1, dir2)
	#angle = atan2(length(axis), dot(dir1,dir2))
	angle = acos(max(-1,min(1,dot(dir1, dir2))))
	if angle:	return angleAxis(angle, normalize(axis))
	else:		return quat()

cornersize = 0.1


class Pivot:	# TODO test corrections
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
		gap = a1[0] - a2[0]
		#print('  a1', a1)
		#print('  a2', a2)
		
		rot = cross(a1[1],a2[1])
		r = (orients[0]*self.axis[0][0], orients[1]*self.axis[1][0])
		s = (-gap,
			  rot + project(cross(r[0],-gap) / (dot(r[0],r[0]) + 1e-15), normal),
			  gap,
			 -rot + project(cross(r[1],gap) / (dot(r[1],r[1]) + 1e-15), normal),
			)
		#print('  pivot', s)
		return s
		
	def scheme(self, solid, size, junc):
		''' return primitives to render the junction from the given solid side
			size is the desired size of the junction
			junc is the point the junction is linked with the scheme by
		'''
		if solid is self.solids[0]:
			axis = self.axis[0]
			center = self.position[0]
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
			axis = self.axis[1]
			center = self.position[1]
			side = axis[1] * size * 0.5
			if dot(junc-axis[0], axis[1]) < 0:
				side = -side
			c1 = generation.flatsurface(generation.outline(primitives.Circle(
							(center-axis[1]*size*0.5, -axis[1]), 
							size/4, 
							resolution=('div', 16),
							)))
			c2 = generation.flatsurface(generation.outline(primitives.Circle(
							(center+axis[1]*size*0.5, axis[1]), 
							size/4, 
							resolution=('div', 16),
							)))
			indices = [(i-1,i)  for i in range(1,len(c1.points))]
			s = Scheme([junc, center+side, center-side], [], [], [(0,1), (1,2)])
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
		normal = normalize(a1[1] + a2[1])
		gap = project(a1[0] - a2[0], normal)
		#print('  a1', a1)
		#print('  a2', a2)
		
		rot = cross(a1[1],a2[1])
		r = (orients[0]*self.axis[0][0], orients[1]*self.axis[1][0])
		s = (-gap - noproject(cross(rot, r[0]), normal),
			  rot + project(cross(r[0],-gap) / (dot(r[0],r[0]) + 1e-15), normal),
			  gap - noproject(cross(-rot, r[1]), normal),
			 -rot + project(cross(r[1],gap) / (dot(r[1],r[1]) + 1e-15), normal),
			)
		#print('  plane', s)
		return s
	
	def scheme(self, solid, size, junc):
		print('axis', self.axis)
		if   solid is self.solids[0]:		center, normal = self.axis[0]
		elif solid is self.solids[1]:		center, normal = self.axis[1]
		else:	return
		center = self.position - project(self.position - center, normal)
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
	scheme = Scheme([], [], [], [], color)
	for cst in constraints:
		part = cst.scheme(solid, size, center)
		if part:	scheme.extend(part)
	
	return scheme

def faceoffset(o):
	return lambda f: (f[0]+o, f[1]+o, f[2]+o)

class Scheme:
	def __init__(self, points, transpfaces, opacfaces, lines, color=None):
		self.color = color or settings.display['schematics_color']
		self.points = points
		self.transpfaces = transpfaces
		self.opaqfaces = opacfaces
		self.lines = lines
	
	def extend(self, other):
		l = len(self.points)
		self.points.extend(other.points)
		#self.opacfaces.extend(map(faceoffset(l), other.opaqfaces))		# create a faceoffset function
		#self.opacfaces.extend([f+l for f in other.opaqfaces])		# use ivec as faces and edges
		self.transpfaces.extend(((a+l,b+l,c+l) for a,b,c in other.transpfaces))
		self.opaqfaces.extend(((a+l,b+l,c+l) for a,b,c in other.opaqfaces))
		self.lines.extend(((a+l, b+l)  for a,b in other.lines))
	
	def display(self, scene):
		scene.objs.append(WireDisplay(scene, self.points, self.transpfaces, self.opaqfaces, self.lines, self.color))


class WireDisplay:
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
		self.vb_transpfaces = ctx.buffer(np.array(transpfaces, dtype='u4', copy=False))
		self.vb_opaqfaces = ctx.buffer(np.array(opaqfaces, dtype='u4', copy=False))
		self.vb_lines = ctx.buffer(np.array(lines, dtype='u4', copy=False))
		
		self.va_transpfaces = ctx.vertex_array(
				self.transpshader,
				[(self.vb_vertices, '3f4 3f4', 'v_position', 'v_normal')],
				self.vb_transpfaces,
				)
		self.va_opaqfaces = ctx.vertex_array(
				self.uniformshader,
				[(self.vb_vertices, '3f4 12x', 'v_position')],
				self.vb_opaqfaces,
				)
		self.va_lines = ctx.vertex_array(
				self.uniformshader,
				[(self.vb_vertices, '3f4 12x', 'v_position')],
				self.vb_lines,
				)
	
	def render(self, scene):
		viewmat = scene.view_matrix * self.transform
		
		#scene.ctx.blend_func = mgl.SRC_ALPHA, mgl.ONE_MINUS_SRC_ALPHA
		#scene.ctx.blend_equation = mgl.FUNC_ADD
		
		self.uniformshader['color'].write(self.color)
		self.uniformshader['view'].write(viewmat)
		self.uniformshader['proj'].write(scene.proj_matrix)
		self.transpshader['color'].write(self.color)
		self.transpshader['view'].write(viewmat)
		self.transpshader['proj'].write(scene.proj_matrix)
		
		scene.ctx.enable(mgl.DEPTH_TEST)
		scene.ctx.disable(mgl.CULL_FACE)
		self.va_opaqfaces.render(mgl.TRIANGLES)
		self.va_lines.render(mgl.LINES)
		
		scene.ctx.disable(mgl.DEPTH_TEST)
		scene.ctx.enable(mgl.CULL_FACE)
		self.va_transpfaces.render(mgl.TRIANGLES)
	
	def identify(self, scene, startident):
		pass	# 1 ident for the whole solid
		
		#viewmat = scene.view_matrix * self.transform
		#scene.ident_shader['start_ident'] = startident
		#scene.ident_shader['view'].write(scene.view_matrix)
		#scene.ident_shader['proj'].write(scene.proj_matrix)
		#self.va_idents.render(mgl.TRIANGLES)

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

if __name__ == '__main__':

	# test kinematic solver
	from constraints import *
	
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

	# test schematics display
	
	print('\n test display')
	s1 = Solid()
	s2 = Solid()
	s3 = Solid()
	A = vec3(2,0,0)
	B = vec3(0,2,0)
	C = vec3(0,-1,1)
	x = vec3(1,0,0)
	y = vec3(0,1,0)
	csts = [Pivot(s1,s2, (A,x)), Pivot(s1,s2, (B,y)), Plane(s2,s3, (C,y), position=C)]
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
	
