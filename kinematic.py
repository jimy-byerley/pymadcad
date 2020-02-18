from mathutils import fvec3, vec3, mat3, mat4, quat, mat4_cast, quat_cast, column, dot, length, normalize, project, dirbase, Box
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
	def __init__(self, mesh=None, base=None, pose=None):
		transform = base or pose or mat4(1)
		transform = mktransform(transform)
		self.position = vec3(column(transform, 3))
		self.orientation = quat_cast(mat3(transform))
		self.mesh = mesh
	
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
		return self.points, self.solid.position, self.solid.center
	
	def corrections(self):
		center, rot = self.pose()
		corr = [rot*ref + center - pt    for pt,ref in zip(self.points, self.reference)]
		return corr, center-self.solid.center, rot-self.solid.orientation
	
	def pose(self):
		center = sum(self.points)/len(self.points)
		rot = quat()
		for pt,ref in zip(self.points, self.reference):
			v, r = normalize(pt-ref), normalize(ref)
			angle = acos(dot(v,r))
			rot += angleAxis(angle, cross(v,r)/sin(angle))
		rot = normalize(rot)
		return self.solid.center+solid.center, normalize(self.solid.orientation+mat3_cast(rot))

def mktransform(obj):
	msg = 'a transformation must be a  mat3, mat4, quat, (O,mat3), (O,quat), (0,x,y,z)'
	if isinstance(obj, mat4):	return obj
	elif isinstance(obj, mat3):	return mat4(obj)
	elif isinstance(obj, quat):	return mat4_cast(obj)
	elif isinstance(obj, vec3):	return translate(mat4(1), obj)
	elif isinstance(obj, tuple):
		if isinstance(obj[0], vec3) and isinstance(obj[1], mat3):	return translate(mat4(obj[1]), obj[0])
		elif isinstance(obj[0], vec3) and isinstance(obj[1], quat):	return translate(mat4_cast(obj[1]), obj[0])
		elif isinstance(obj[0], vec3) and len(obj) == 3:			return mat4(mat3(obj))
		elif isinstance(obj[0], vec3) and len(obj) == 4:			return translate(mat4(mat3(obj[1:])), obj[0])
		else:
			raise TypeError(msg)
	else:
		raise TypeError(msg)

def rotbt(dir1, dir2):
	axis = cross(dir1, dir2)
	angle = acos(dot(dir1,dir2))
	return angleAxis(angle, axis * sin(angle))

cornersize = 0.1

class Pivot:	# TODO test corrections
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1,s2)
		self.axis = (a1, a2 or a1)
		self.position = position or a1[0]
	def params(self):
		return s1.center, s1.orientation, s2.center, s2.orientation
	def corrections(self):
		trans = inverse(self.solids[1].transform()) * self.solids[0].transform()
		gap = trans * self.axis[0][0] - self.axis[1][0]
		rot = rotbt(mat3(trans) * self.axis[0][1], self.axis[1][1])
		return (
			inverse(trans) * gap - self.solid[0].orientation, 
			inverse(self.solid[0].orientation) * rot - self.solid[1].orientation, 
			-gap, 
			rot,
			)
		
	def scheme(self, solid, size, junc):
		''' return primitives to render the junction from the given solid side
			size is the desired size of the junction
			junc is the point the junction is linked with the scheme by
		'''
		if solid is self.solids[0]:
			axis = self.axis[0]
			center = self.position
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
			center = self.position
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
	def __init__(self, s1, s2, n1, n2=None, position=None):
		self.solids = (s1, s2)
		self.normals = (n1, n2 or n1)
		self.position = position
	def params(self):
		return s1.center, s1.orientation, s2.center, s2.orientation
	def corrections(self):
		trans = inverse(self.solids[1].transform()) * self.solids[0].transform()
		gap = project(trans * self.axis[0][0] - self.axis[1][0], self.axis[1][1])
		rot = rotbt(mat3(trans) * self.axis[0][1], self.axis[1][1])
		return (
			inverse(trans) * gap - self.solid[0].orientation, 
			inverse(self.solid[0].orientation) * rot - self.solid[1].orientation, 
			-gap, 
			rot,
			)
	
	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:
			normal = self.normals[0]
		elif solid is self.solids[1]:
			normal = -self.normals[1]
		else:
			return
		center = self.position or project(junc, normal)
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
				
				


def mkwiredisplay(solid, constraints, size=None, color=None):
	# get center and size of the solid's scheme
	center = vec3(0)
	contrib = 0
	box = Box()
	for cst in constraints:
		if solid in cst.solids and cst.position:
			contrib += 1
			center += cst.position
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
	def __init__(self, scene, points, transpfaces, opaqfaces, lines, color=None):
		ctx = scene.ctx
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
		#scene.ctx.blend_func = mgl.SRC_ALPHA, mgl.ONE_MINUS_SRC_ALPHA
		scene.ctx.blend_equation = mgl.FUNC_ADD
		
		self.uniformshader['color'].write(self.color)
		self.uniformshader['view'].write(scene.view_matrix)
		self.uniformshader['proj'].write(scene.proj_matrix)
		self.transpshader['color'].write(self.color)
		self.transpshader['view'].write(scene.view_matrix)
		self.transpshader['proj'].write(scene.proj_matrix)
		
		scene.ctx.enable(mgl.DEPTH_TEST)
		scene.ctx.disable(mgl.CULL_FACE)
		self.va_opaqfaces.render(mgl.TRIANGLES)
		self.va_lines.render(mgl.LINES)
		
		scene.ctx.disable(mgl.DEPTH_TEST)
		scene.ctx.enable(mgl.CULL_FACE)
		self.va_transpfaces.render(mgl.TRIANGLES)
	
	def identify(self, scene, startident):
		pass

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
	s1 = Solid()
	s2 = Solid()
	s3 = Solid()
	A = vec3(2,0,0)
	B = vec3(0,2,0)
	C = vec3(0,-1,1)
	x = vec3(1,0,0)
	y = vec3(0,1,0)
	csts = [Pivot(s1,s2, (A,x)), Pivot(s1,s2, (B,y)), Plane(s2,s3, y, position=C)]
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
	
