from .mathutils import (vec3, mat3, mat4, quat, quat,
						isnan, inverse,
						dot, cross, distance, length, normalize, project, noproject, dirbase,
						pi, inf, glm,
						transform,
						)
from .kinematic import Torsor, WireDisplay, Scheme
from .mesh import Mesh, Wire, web, Web
from . import generation, primitives

__all__ = ['Pivot', 'Plane', 'Track', 'Gliding']

'''
TODO:
	Ball		rotule
	Punctiform	ponctuelle (sphere-plan)
	Ringlin		linéaire annulaire
	Linear		linéaire
	Screw		helicoidale
	Gear		engrenage
	Rack		crémaillere
	
	Screw(onesolid, othersolid, (B,x), (D,y), 0.2),	# helicoidale
	Gear(onesolid, onceagain, (O,z), (O,z), 2, offset=0),	# engrenage
	Rack(onesolid, onceagain, (O,z), x, 1, offset=0),	# cremaillere
'''

	
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
