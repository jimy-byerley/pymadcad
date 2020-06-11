import numpy.core as np
import moderngl as mgl

from .common import ressourcedir
from .mathutils import (fvec3, fmat4, vec3, vec4, mat3, mat4, quat, mat4_cast, quat_cast,
						column, translate, inverse, isnan,
						dot, cross, length, normalize, project, noproject, dirbase, distance,
						atan2, acos, angle, axis, angleAxis,
						pi, inf, glm,
						Box, transform)
from .mesh import Mesh, Wire, web, Web
from . import generation
from . import primitives
from . import settings
from . import constraints


__all__ = ['Torsor', 'comomentum', 'Solid', 'Kinemanip', 'solvekin',
			'makescheme', 'Scheme', 'WireDisplay',
			'Pivot', 'Plane', 'Track', 'Linear']


'''
onesolid = Solid(mesh1, base=R0)	# solide avec visuel, maillage a transformer pour passer dans la base adhoc
othersolid = Solid(mesh2, pose=R1)	 # solide avec visuel, maillage prétransformé
onceagain = Solid()                 # solide vide, base bar defaut (transformation = I4)
csts = [
	Pivot(onesolid, othersolid, (O,x), (O,z)),
	Gliding(onesolid, othersolid, (C,x), C),	# pivot glissant
	Plane(onesolid, othersolid, z, x),	# plan
	Track(onesolid, othersolid, x, y),		# glissiere
	Ball(onesolid, othersolid, P, P)		# rotule
	Punctiform(onesolid, ithersolid, (O,x), P)	# ponctuelle
	Ringlin(onesolid, othersolid, (O,z), P)		# lineaire annulaire
	Linear(onesolid, othersolid, (O,z), (O,x))	# lineaire
	
	Screw(onesolid, othersolid, (B,x), (D,y), 0.2),	# helicoidale
	Gear(onesolid, onceagain, (O,z), (O,z), 2, offset=0),	# engrenage
	Rack(onesolid, onceagain, (O,z), x, 1, offset=0),	# cremaillere
	
	InSolid(onesolid, A, B, C, D, E),	# points sur solide
	]
'''

class Torsor(object):
	''' a 3D torsor is a mathematical object defined as follow:
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
	def locate(self, pt):
		''' gets the same torsor, but expressed for an other location '''
		return Torsor(self.resulting, self.momentum + cross(self.resulting, pt-self.position), pt)
	
	def transform(self, mat):
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
	
	Attributes defined here:
		* orientation - quat  as rotation from local to world space
		* position - vec3	as displacement from local to world
		* visuals	- list of objects to display using the solid's pose
		* name - optional name to display on the scheme
	'''
	def __init__(self, *args, base=None, pose=None, name=None):
		if pose:
			self.position = pose[0]
			self.orientation = quat(pose[1])
		else:
			self.position = vec3(0)
			self.orientation = quat()
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
		self.reference = reference or [p*solid.transform() for p in points]	# if no reference position is provided, then we use the current position in the solid
		# remove origin from references
		center = sum(reference)/len(reference)
		for p in self.reference:	p -= center
	
	slvvars = 'solid', 'points',

	def pose(self):
		center = sum(self.points)/len(self.points)
		rot = vec3(0)
		for pt,ref in zip(self.points, self.reference):
			v, r = normalize(pt-ref), normalize(ref)
			c = cross(v,r)
			if not glm.any(isnan(c)):
				angle = acos(dot(v,r))
				rot += angleAxis(angle, c/sin(angle))
		rot = normalize(rot)
		#print('  pose', center, rot)
		return self.solid.position+center, self.solid.orientation+rot

cornersize = 0.1

def solidtransform_axis(solid, obj):
	rot = solid.orientation
	return (rot*obj[0] + solid.position, rot*obj[1])
def solidtransform_base(solid, obj):
	rot = solid.orientation
	return (rot*obj[0] + solid.position, rot*obj[1], rot*obj[2])

class Pivot:
	''' Junction for rotation only around an axis
	
		classical definition:	Pivot (axis)
		no additional information is required by the initial state
		this class holds an axis for each side
	'''
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1,s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	slvvars = 'solids',
	def fit(self):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		return distance(a0[0],a1[0])**2 + 1-dot(a0[1],a1[1])
	
	def corrections(self):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		r = cross(a0[1], a1[1])
		t = a1[0] - a0[0]
		#return Torsor(r,t,a0[0]), Torsor(-r,-t,a1[0]) 	# velocity torsor
		return Torsor(t,r,a0[0]), Torsor(-t,-r,a1[0]) 	# force torsor
	
	def transmitable(self, action):
		axis = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Torsor(action.resulting, noproject(action.momentum, normal), action.position)
		
	def scheme(self, solid, size, junc):
		''' return primitives to render the junction from the given solid side
			size is the desired size of the junction
			junc is the point the junction is linked with the scheme by
		'''
		if solid is self.solids[0]:
			radius = size/4
			axis = self.axis[0]
			center = axis[0] + project(self.position[0]-axis[0], axis[1])
			cyl = generation.extrusion(
						axis[1]*size * 0.8, 
						web(primitives.Circle(
								(center-axis[1]*size*0.4, axis[1]), 
								size/4, 
								resolution=('div', 16),
						)))
			l = len(cyl.points)
			v = junc - center
			v = normalize(noproject(v,axis[1]))
			if glm.any(isnan(v)):
				v,_,_ = dirbase(axis[1])
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
			center = axis[0] + project(self.position[1]-axis[0], axis[1])
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
	''' Joint for translation in 2 directions and rotation around the third direction 
		
		classical definition:	Plane (direction vector)
		the initial state requires an additional distance between the solids
		this class holds an axis for each side, the axis origins are constrained to share the same projections on the normal
	'''
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1, s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	slvvars = 'solids',
	def fit(self):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		normal = normalize(a0[1] + a1[1])
		return dot(a0[0]-a1[0], normal) **2 - dot(a0[1], a1[1])
	
	def corrections(self):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		r = cross(a0[1], a1[1])
		t = project(a1[0] - a0[0], normalize(a0[1] + a1[1]))
		
		#return Torsor(r,t,a0[0]), Torsor(-r,-t,a1[0]) 	# velocity torsor
		return Torsor(t,r,a0[0]), Torsor(-t,-r,a1[0]) 	# force torsor
	
	def transmitable(self, action):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Torsor(project(action.resulting, normal), noproject(action.momentum, normal), action.position)
	
		
	
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

class Track:
	''' Joint for translation only in a direction 
		
		classical definition:  Track (direction vector)
		the initial state requires more parameters: the relative placements of the solids
		this class holds a base for each of the solids, bases are (0,X,Y)  where X,Y are constrained to keep the same direction across bases, and the bases origins lays on their common Z axis
	'''
	def __init__(self, s1, s2, b1, b2=None, position=None):
		self.solids = (s1, s2)
		self.bases = (b1, b2 or b1)
		self.position = (self.bases[0][0], self.bases[1][0])
	
	slvvars = 'solids',
	def fit(self):
		b0, b1 = solidtransform_base(self.solids[0], self.bases[0]), solidtransform_base(self.solids[1], self.bases[1])
		t = b1[0] - b0[0]
		x = normalize(b0[1]+b1[1])
		y = normalize(b0[2]+b1[2])
		return (	1-dot(cross(b0[1],b0[2]), cross(b1[1],b1[2]))
				+	dot(t,x) **2
				+	dot(t,y) **2
				)
	
	def corrections(self):
		b0, b1 = solidtransform_base(self.solids[0], self.bases[0]), solidtransform_base(self.solids[1], self.bases[1])
		z0, z1 = cross(b0[1],b0[2]), cross(b1[1],b1[2])
		r = (	cross(z0,z1)
			+	cross(b0[1], b1[1])
			+	cross(b0[2], b1[2])
			)
		t = b1[0] - b0[0]
		return (
			Torsor(noproject( t, z1),  r,b0[0]), 
			Torsor(noproject(-t, z0), -r,b1[0]),
			)
	
	def transmitable(self, action):
		z0, z1 = cross(b0[1],b0[2]), cross(b1[1],b1[2])
		normal = normalize(z0 + z1)
		return Torsor(noproject(action.resulting, normal), action.momentum, action.position)
	
	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:
			o,x,y = self.bases[0]
			y = normalize(noproject(y,x))
			z = normalize(cross(x,y))
			s = 0.25*size
			line = Web(
						[(x+y)*s, (-x+y)*s, (-x-y)*s, (x-y)*s],
						[(0,1),(1,2),(2,3),(3,0)],
						)
			line.transform(-size/2*z)
			ext = generation.extrusion(size*z, line)
			l = len(ext.points)
			v = junc - o
			if abs(dot(v,x)) > abs(dot(v,y)):
				v = x if dot(v,x)>0 else -x
			else:
				v = y if dot(v,y)>0 else -y
			p = o + v*0.25*size
			ext.points.append(p)
			ext.points.append(p + z*size*cornersize)
			ext.points.append(p + v*size*cornersize)
			ext.points.append(junc)
			return Scheme(
					ext.points, 
					ext.faces, 
					[(l,l+1,l+2)], 
					[	(l, l+2),
						(0,1),(1,2),(2,3),(3,0),
						(0,4), (1,5), (2,6), (3,7), 
						(4,5),(5,6),(6,7),(7,4)],
					)
		
		elif solid is self.solids[1]:
			o,x,y = self.bases[1]
			y = noproject(y,x)
			z = cross(x,y)
			s = 0.15*size
			line = Web(
						[(x+y)*s, (-x+y)*s, (-x-y)*s, (x-y)*s],
						[(0,2),(1,3)],
						)
			line.transform(-size/2*z)
			ext = generation.extrusion(size*z, line)
			l = len(ext.points)
			v = junc - o
			if abs(dot(v,x)) > abs(dot(v,y)):
				v = x if dot(v,x)>0 else -x
			else:
				v = y if dot(v,y)>0 else -y
			p = o + z*size/2
			ext.points.append(p)
			ext.points.append(junc)
			return Scheme(
					ext.points, 
					[], 
					[], 
					[	(l, l+1),
						(0,2), (1,3),
						(0,4), (1,5), (2,6), (3,7),
						(4,6), (5,7)],
					)

class Gliding:
	''' Joint for rotation and translation around an axis 
		
		classical definition:  Gliding pivot (axis)
		the initial state doesn't require more data
		this class holds an axis for each side
	'''
	def __init__(self, s1, s2, a1, a2=None, position=None):
		self.solids = (s1,s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	slvvars = 'solids',
	def fit(self):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		return distance(a0[0],a1[0])**2 + 1-dot(a0[1],a1[1])
	
	def corrections(self):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0]), solidtransform_axis(self.solids[1], self.axis[1])
		r = cross(a0[1], a1[1])
		t = a1[0] - a0[0]
		return (
				Torsor(noproject( t,a0[1]),  r,a0[0]), 
				Torsor(noproject(-t,a1[1]), -r,a1[0]),
				) # force torsor
	
	def transmitable(self, action):
		axis = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Torsor(
				noproject(action.resulting, normal), 
				noproject(action.momentum, normal), 
				action.position)
		
	def scheme(self, solid, size, junc):
		''' return primitives to render the junction from the given solid side
			size is the desired size of the junction
			junc is the point the junction is linked with the scheme by
		'''
		if solid is self.solids[0]:
			radius = size/4
			axis = self.axis[0]
			center = axis[0] + project(self.position[0]-axis[0], axis[1])
			cyl = generation.extrusion(
						axis[1]*size, 
						web(primitives.Circle(
								(center-axis[1]*size*0.5, axis[1]), 
								size/4, 
								resolution=('div', 16),
						)))
			l = len(cyl.points)
			v = junc - center
			v = normalize(noproject(v,axis[1]))
			if glm.any(isnan(v)):
				v,_,_ = dirbase(axis[1])
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
			center = axis[0] + project(self.position[1]-axis[0], axis[1])
			attach = side = axis[1] * size
			return Scheme([center-side, center+side, center+attach, junc], [], [], [(0,1), (2,3)])

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
		scheme = Scheme([], [], [], [], color, solid.transform())
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
	
	def display(self, scene):
		return WireDisplay(scene, self.points, self.transpfaces, self.opaqfaces, self.lines, self.color, self.transform),


class WireDisplay:
	''' wireframe display for schemes, like kinematic schemes '''
	renderindex = 1
	
	def __init__(self, scene, points, transpfaces, opaqfaces, lines, color=None, transform=fmat4(1)):
		ctx = scene.ctx
		self.transform = fmat4(*transform)
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

def solvekin(joints, fixed=(), precision=1e-4, maxiter=None, damping=0.9):
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

'''
class Problem:
	def __init__(self, joints, fixed=()):
		self.joints = joints
		self.fixed = fixed
	
		self.solids = []
		self.register = {id(s) for s in self.solids}
		self.corrdispatch = []
		self.propagate = []
		self.contribs = [0] * len(self.solids)
	
	def solve(self, precision=1e-4, maxiter=None, damping=0.5):
		reached = inf
		itercount = 0
		while reached > precision and (not maxiter or maxiter > itercount):
			reached = self.step(damping)
		return reached
	
	def step(self, damping):
		corrs = [Torsor(position=centers[i])  for i in range(len(self.solids))]
		barys = [vec3(0)	for i in range(len(self.solids))]
		
		# sum contributions
		for joint,dispatch in zip(self.joints, self.corrdispatch):
			for contrib,dst in zip(joint.corrections(), dispatch):
				corrs[dst] += contrib.locate(centers[dst])
				barys[dst] += contrib.position
		for i,ncontrib in enumerate(self.contribs):
			corrs[i] /= ncontrib
			barys[i] /= ncontrib
		for i,coor in enumerate(corrs):
			corrs[i] = coor.locate(barys[i])
		
		# propagate across joints
		shared = [Torsor(position=center[i])	for i in range(len(self.solids))]
		for propagation,contrib in zip(self.propagations,corr):
			for src,dst,transmitter in propagation:
				contrib = transmitter(contrib)
				shared[dst] += contrib
		# assign propagated
		for dst,share in enumerate(shared):
			corrs[dst] += share
		
		# assign corrections
		corrmax = 0
		for solid,corr in zip(self.solids, corrs):
			solid.position += corr.resulting*damping
			solid.orientation = quat(corr.momentum*damping) * solid.orientation
			corrmax = max(corrmax, abs(a), length(corr.resulting))
		
		return corrmax
'''

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
	
	def __init__(self, csts, root, scheme=False):
		self.csts, self.root = csts, root
		self.locked = {id(root): root}
		self.primitives = {}
		self.debug_label = None
		self.reproblem()
		if scheme:
			makescheme(self.csts)
	
	def display(self, scene):
		yield self
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
			self.startpt = self.actsolid.position - quat(self.actsolid.orientation)*self.ptoffset
			self.debug_label.position = fvec3(pt)
			store(self.actsolid.position, pt+quat(self.actsolid.orientation)*self.ptoffset)
			# solve
			#try:	self.problem.solve(precision=1e-3, method=method, maxiter=50)
			try:	solvekin(self.csts, self.locked.values(), precision=1e-2, maxiter=50)
			except constraints.SolveError as err:	pass
		else:
			if not self.moved:
				self.lock(self.actsolid, id(self.actsolid) not in self.locked)
			else:
				# finish on a better precision
				#try:	self.problem.solve(precision=1e-4, method=method)
				try:	solvekin(self.csts, self.locked.values(), precision=1e-4, maxiter=1000)
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
