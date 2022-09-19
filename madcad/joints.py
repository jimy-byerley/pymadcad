# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .primitives import isaxis
from .kinematic import Screw, WireDisplay, Scheme, Joint
from .mesh import Mesh, Wire, web, Web
from . import generation, primitives

__all__ = ['Pivot', 'Plane', 'Track', 'Gliding', 'Ball', 'Punctiform', 'Gear', 'Helicoid']

'''
TODO:
	Ring		linéaire annulaire
	Hinge		linéaire
	Rack		crémaillere
	
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
	if len(obj) == 3:	return (rot*obj[0] + solid.position, rot*obj[1], rot*obj[2])
	if len(obj) == 4:	return (rot*obj[0] + solid.position, rot*obj[1], rot*obj[2], rot*obj[3])
	
	
class Welded(Joint):
	def __init__(self, s1, s2, transform=None):
		self.solids = (s1, s2)
		self.transform = transform or affineInverse(s1.pose) * s2.pose
		self.position = (vec3(0), vec3(0))
		
	def corrections(self):
		p1 = dmat4x3(self.solids[0].pose * self.transform)
		p2 = dmat4x3(self.solids[1].pose)
		t = p2[3] - p1[3]
		r = cross(p1[0], p2[0]) + cross(p1[1], p2[1]) + cross(p1[2], p2[2])
		return Screw(t,r,p1[3]), Screw(-t,-r,p2[3])
		
	def scheme(self, solid, size, junc=None) -> Scheme:
		return Scheme([], [], [], [])

class Pivot(Joint):
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
		#return Screw(r,t,a0[0]), Screw(-r,-t,a1[0]) 	# velocity torsor
		return Screw(t,r,a0[0]), Screw(-t,-r,a1[0]) 	# force torsor
	
	def transmitable(self, action):
		axis = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Screw(action.resulting, noproject(action.momentum, normal), action.position)
		
	def scheme(self, solid, size, junc=None) -> Scheme:
		''' return a Scheme to render the junction from the given solid side
			size is the desired size of the junction
			junc is the point the junction is linked with the scheme by
		'''
		if solid is self.solids[0]:
			radius = size/4
			axis = self.axis[0]
			center = axis[0] + project(self.position[0]-axis[0], axis[1])
			cyl = generation.extrusion(
						axis[1]*size * 0.8, 
						primitives.Circle(
								(center-axis[1]*size*0.4, axis[1]), 
								size/4, 
								resolution=('div', 16),
						))
			sch = Scheme(cyl.points, cyl.faces, [], list(cyl.outlines_unoriented()))
			if junc:
				v = normalize(noproject(junc - center, axis[1]))
				if not isfinite(v):
					v,_,_ = dirbase(axis[1])
				p = center + v*size/4
				sch.extend(Scheme(
							[p, p + axis[1]*size*cornersize, p + v*size*cornersize, junc],
							[], 
							[(0,1,2)],
							[(0,2), (2,3)],
							))
			return sch
					
			
			#axis = self.axis[0]
			#center = axis[0] + project(self.position[0]-axis[0], axis[1])
			#cyl = generation.extrusion(
						#axis[1]*size * 0.8, 
						#primitives.Circle(
								#(center-axis[1]*size*0.4, axis[1]), 
								#size/4, 
								#resolution=('div', 16),
						#))
			
			#sch = (scheme.Scheme(space=scheme.world, color=fvec4(settings.display['schematics_color'],1))
					#.add(cyl, shader='ghost')
					#.add(cyl.outlines(), shader='line')
					#)
			
			#v = junc - center
			#v = normalize(noproject(v,axis[1]))
			#if isfinite(v):
				#x,_,_ = dirbase(axis[1])
			#p = center + v*size/4
			#sch.add(
				#Mesh([p, p+axis[1]*size*cornersize, p + x*size*cornersize], [(0,1,2)]),
				#shader='fill')
			#sch.add([p, junc], shader='line')
				
			#return sch
					
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

class Planar(Joint):
	''' Joint for translation in 2 directions and rotation around the third direction 
		
		classical definition:	Planar (direction vector)
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
		
		#return Screw(r,t,a0[0]), Screw(-r,-t,a1[0]) 	# velocity torsor
		return Screw(t,r,a0[0]), Screw(-t,-r,a1[0]) 	# force torsor
	
	def transmitable(self, action):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Screw(project(action.resulting, normal), noproject(action.momentum, normal), action.position)
	
		
	
	def scheme(self, solid, size, junc=None):
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

class Track(Joint):
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
			Screw(noproject( t, z1),  r,b0[0]), 
			Screw(noproject(-t, z0), -r,b1[0]),
			)
	
	def transmitable(self, action):
		z0, z1 = cross(b0[1],b0[2]), cross(b1[1],b1[2])
		normal = normalize(z0 + z1)
		return Screw(noproject(action.resulting, normal), action.momentum, action.position)
	
	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:
			o,x,y = self.bases[0]
			y = normalize(noproject(y,x))
			z = normalize(cross(x,y))
			s = 0.25*size
			line = Web(
						[(x-y)*s, (x+y)*s, (x+y)*s, (-x+y)*s, (-x+y)*s, (-x-y)*s, (-x-y)*s, (x-y)*s],
						[(0,1),(2,3),(4,5),(6,7)],
						)
			line.transform(o-size/2*z)
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
					[	(l+2, l+3),
						(0,1),(2,3),(4,5),(6,7),
						(0,8), (2,10), (4,12), (6,14), 
						(8,9),(10,11),(12,13),(14,15)],
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
			line.transform(o-size/2*z)
			ext = generation.extrusion(size*z, line)
			l = len(ext.points)
			v = junc - o
			v = z if dot(v, z) > 0 else -z
			p = o + v*size/2
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

class Gliding(Joint):
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
				Screw(noproject( t,a0[1]),  r,a0[0]), 
				Screw(noproject(-t,a1[1]), -r,a1[0]),
				) # force torsor
	
	def transmitable(self, action):
		axis = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Screw(
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


class Ball(Joint):
	''' Joint for rotation all around a point.
	
		classical definition: Ball (point)
		the initial state doen't require more data
		the class holds a point for each side
	'''
	def __init__(self, s1, s2, p1, p2=None):
		self.solids = (s1,s2)
		self.points = (p1, p2 or p1)
		self.position = self.points
	
	slvvars = 'solids',
	def fit(self):
		p0 = self.solids[0].orientation*self.points[0] + self.solids[0].position
		p1 = self.solids[1].orientation*self.points[1] + self.solids[1].position
		return distance(p0, p1)**2
	
	def corrections(self):
		p0 = self.solids[0].orientation*self.points[0] + self.solids[0].position
		p1 = self.solids[1].orientation*self.points[1] + self.solids[1].position
		return (
			Screw(p1-p0, vec3(0), p0),
			Screw(p0-p1, vec3(0), p1),
			) # force torsor
	
	def scheme(self, solid, size, junc=vec3(0,0,1)):
		if solid is self.solids[0]:
			r = 0.3*size
			p = self.points[0]
			sph = generation.icosphere(p, r, resolution=('rad',0.2))
			z = normalize(junc-p)
			if not isfinite(z):
				z = vec3(0,0,1)
			l = len(sph.points)
			sph.points.append(p + r*z)
			sph.points.append(p + r*z + 0.1*size*z)
			sph.points.append(junc)
			return Scheme(sph.points, sph.faces, [], [(l+0, l+1), (l+1, l+2)])
		elif solid is self.solids[1]:
			r = 0.4*size
			angle = 1.2
			p = self.points[1]
			z = normalize(junc-p)
			if not isfinite(z):		z = vec3(0,0,1)
			x,y,z = dirbase(z)
			prof = primitives.ArcCentered((p,y), p+z*r, p+r*(x*sin(angle)+z*cos(angle)))
			sph = generation.revolution(2*pi, (p,z), prof)
			sph.finish()
			l = len(sph.points)
			sph.points.append(p + r*z)
			sph.points.append(p + r*z + 0.1*size*z)
			sph.points.append(junc)
			return Scheme(sph.points, sph.faces, [], 
					[(l+0, l+1), (l+1, l+2)] + list(sph.outlines_unoriented()) )
from madcad.nprint import nprint
class Punctiform(Joint):
	''' Joint for rotation all around a point belonging to a plane.
	
		classical definition: Punctiform/Sphere-Plane (axis)
		the initial state does't require more data
		the class holds a normal axis for the plane side, and a point for the sphere side (that defaults to the axis'origin)
	'''
	def __init__(self, s1, s2, a1, p2=None):
		self.solids = (s1,s2)
		self.axis = a1
		self.point = p2 or a1[0]
		self.position = (self.axis[0], self.point)
	
	slvvars = 'solids',
	def fit(self):
		a = solidtransform_axis(self.solids[0], self.axis)
		p = self.solids[1].orientation*self.point + self.solids[1].position
		return project(p-a[0], a[1]) **2
	
	def corrections(self):
		a = solidtransform_axis(self.solids[0], self.axis)
		p = self.solids[1].orientation*self.point + self.solids[1].position
		t = Screw(project(p-a[0], a[1]), vec3(0), p)
		return (t, -t) # force torsor
	
	def scheme(self, solid, size, junc):
		r = 0.2*size
		if solid is self.solids[0]:
			center, normal = self.axis
			if dot(junc-center, normal) < 0:	normal = -normal
			x,y,z = dirbase(normal)
			c = center + r*z
			x *= size*0.7
			y *= size*0.7
			return Scheme(
				[c+(x+y), c+(-x+y), c+(-x-y), c+(x-y), c, c+cornersize*x, c+cornersize*z, junc],
				[(0,1,2), (0,2,3)],
				[(4,5,6)],
				[(0,1),(1,2),(2,3),(3,0),(4,6),(6,7)],
				)
		elif solid is self.solids[1]:
			sph = generation.icosphere(self.point, r, resolution=('rad', 0.2))
			z = normalize(junc-self.point)
			l = len(sph.points)
			sph.points.append(self.point + z*r)
			sph.points.append(junc)
			return Scheme(sph.points, sph.faces, [], [(l+0, l+1)])


class Gear(Joint):
	''' Gear interaction between two solids.
	
		`ratio` is the factor from the rotation of s1 to the rotation of s2
		The two pinions are considered circular, but no assumption is made on their radius or relative inclination.
		The interaction is symetric.
	'''
	def __init__(self, s1, s2, ratio, a1, a2=None, position=None):
		self.ratio = ratio
		self.solids = (s1, s2)
		self.axis = (a1, a2 or a1)
		self.position = position or (self.axis[0][0], self.axis[1][0])
	
	slvvars = 'solids',
	def fit(self):
		#t0 = dot(angleAxis(self.solids[0].orientation)*axis(self.solids[0].orientation), self.axis[0][1])
		#t1 = dot(angleAxis(self.solids[1].orientation)*axis(self.solids[1].orientation), self.axis[1][1])
		#return (t0 - t1) **2
		indev
	
	def corrections(self):
		b0 = solidtransform_base(self.solids[0], (self.axis[0][0], *dirbase(self.axis[0][1])))
		b1 = solidtransform_base(self.solids[1], (self.axis[1][0], *dirbase(self.axis[1][1])))
		# direction of the contact point from the center
		v = b1[0]-b0[0]
		# get the rotation of both solids relatively to this orientation
		t0, t1 = atan2(dot(v,b0[2]), dot(v,b0[1])), atan2(dot(v,b1[2]), dot(v,b1[1]))
		# correct cyclic orientation reset of the fastest solid
		if abs(self.ratio) < 1:
			m = (t1/self.ratio - t0 +pi) %(2*pi) -pi
		else:
			p = (t1 - t0*self.ratio +pi) %(2*pi) -pi
			m = p/self.ratio
		# m is the rotation delta for solid 0
		return (
			Screw(vec3(0), -m * b0[3], b0[0]),
			Screw(vec3(0), m*self.ratio * b1[3], b1[0]),
			)
		
	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:		i0,i1,n = 0,1,self.ratio
		elif solid is self.solids[1]:	i0,i1,n = 1,0,1/self.ratio
		else:	return
		
		a0, a1 = solidtransform_axis(self.solids[0],self.axis[0]), solidtransform_axis(self.solids[1],self.axis[1])
		x,y,z = dirbase(self.axis[i0][1], align=junc-self.axis[i0][0])
		o = self.axis[i0][0] + project(self.position[i0]-self.axis[i0][0], z)
		
		# radial vector
		rl = length(noproject(a0[0]-a1[0], a1[1])) / (dot(a0[1],a1[1]) - 1/self.ratio)
		if solid is self.solids[1]:
			rl /= abs(self.ratio)
		r = o + x * rl
		
		# tooth depth vector
		angle = min(1, dot(	a0[1] if solid is self.solids[0] else a1[1], 
						normalize(mix(a0[1], a1[1], abs(self.ratio)/(1+abs(self.ratio))))
						))
		d = angle*z + sqrt(1-angle**2)*x
		b = size*0.4 * d
		w = size*0.08 * cross(d,y)
		if self.ratio > 0 and (	abs(self.ratio) > 1 and solid is self.solids[0] 
							or	abs(self.ratio) < 1 and solid is self.solids[1]):
			b, w = -b, -w
		
		profile = Web([r+b+w, r+b, r+b, r, r-b, r-b, r-b+w], [(0,1),(2,3),(3,4),(5,6)])
		surf = generation.revolution(2*pi, self.axis[i0], profile, resolution=('rad',0.1))
		l = len(profile.points)
		sch = Scheme(surf.points, surf.faces, [], [(i,i+l) for i in range(3, len(surf.points)-l, l)])
		sch.extend(Scheme([junc, mix(o,r,0.8), r], [], [], [(0,1),(1,2)]))
		return sch


class Helicoid(Joint):
	''' Screw a solid into an other.
	
		`step` is the translation distance needed for that one solid turn by 2*pi around the other
		The interaction is symetric.
	'''
	def __init__(self, s0, s1, step, b0, b1=None, position=None):
		self.step = step	# m/tr
		self.solids = s0, s1
		if isaxis(b0):			b0 = b0[0], *dirbase(b0[1])[:2]
		if b1 and isaxis(b1):	b1 = b1[0], *dirbase(b1[1])[:2]
		self.bases = b0, b1 or b0
		self.position = position or (self.bases[0][0], self.bases[1][0])
		
	slvvars = 'solids',
	def fit(self):
		indev

	def corrections(self):
		(o0,x0,y0), (o1,x1,y1) = solidtransform_base(self.solids[0], self.bases[0]), solidtransform_base(self.solids[1], self.bases[1])
		z0, z1 = cross(x0,y0), cross(x1,y1)
		z = normalize(z0 + z1)
		delta = o1 - o0
		pos = dot(-delta,z)
		angle = atan2(dot(y1,x0), dot(x1,x0))
		
		gap_angle = (pos%self.step)/self.step * 2*pi - angle
		if gap_angle > pi:	gap_angle -= 2*pi
		if abs(gap_angle) > 0.5:	gap_angle = 0
		gap_pos = (angle/(2*pi)*self.step - pos) % self.step
		if gap_pos > self.step/2:	gap_pos -= self.step
		
		r = cross(z0, z1) + gap_angle*z
		t = gap_pos*z
		return Screw(t+noproject(delta,z0), r, o0), Screw(-t-noproject(delta,z1), -r, o1)	# force torsor

	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:
			radius = size/4
			o,x,y = self.bases[0]
			z = cross(x,y)
			center = o + project(self.position[0]-o, z)
			cyl = generation.extrusion(
						z*size, 
						web(primitives.Circle(
								(center-z*size*0.5, z), 
								radius, 
								resolution=('div', 16),
						)))
			l = len(cyl.points)
			v = junc - center
			v = normalize(noproject(v,z))
			if glm.any(isnan(v)):
				v,_,_ = dirbase(z)
			p = center + v*size/4
			cyl.points.append(p)
			cyl.points.append(p + z*size*cornersize)
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
			radius = size/6
			o,x,y = self.bases[1]
			z = cross(x,y)
			center = o + project(self.position[1]-o, z)
			attach = side = z * size
			
			heli = []
			div = 32
			a = 4*pi
			for i in range(-div,div+1):
				t = i/div
				heli.append( radius*(cos(a*t)*x + sin(a*t)*y) + center + side*t )
			heli.extend([center+side, junc])
			heli = web(Wire(heli))
			return Scheme(heli.points, [], [], heli.edges)

