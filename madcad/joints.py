# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .primitives import isaxis, Axis
from .kinematic import Screw, Joint
from .mesh import Mesh, Wire, web, Web
from . import generation, primitives

__all__ = ['Pivot', 'Planar', 'Track', 'Gliding', 'Ball', 'Punctiform', 'Ring', 'Hinge', 
			'Cam', 'Contact',
			'Rack', 'Gear', 'Helicoid']


cornersize = 0.1

def solidtransform_axis(solid, obj):
	rot = solid.orientation
	return (rot*obj[0] + solid.position, rot*obj[1])
def solidtransform_base(solid, obj):
	rot = solid.orientation
	if len(obj) == 3:	return (rot*obj[0] + solid.position, rot*obj[1], rot*obj[2])
	if len(obj) == 4:	return (rot*obj[0] + solid.position, rot*obj[1], rot*obj[2], rot*obj[3])
	

def drotate(angle, axis):
	# derivative of sin and cos an argument translation of pi/2
	m = rotate(angle+0.5*pi, axis)
	# remove homogeneous one and axis proper space
	m -= mat4(outerProduct(axis, axis) / length2(axis))
	return m
	


class Pivot(Joint):
	bounds = (-inf, inf)
	default = 0
	
	def __init__(self, solids, axis: Axis, local=None):
		self.solids = solids
		self.axis = axis
		local = local or axis
		self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
		self.position = axis[0]
		
	def __repr__(self):
		return '{}({}, Axis(({:.3g},{:.3g},{:.3g}), ({:.3g},{:.3g},{:.3g})))'.format(
			self.__class__.__name__, self.solids, *self.axis[0], *self.axis[1])
	
	def direct(self, angle) -> mat4:
		return self.post * rotate(angle, Z) * self.pre
		
	def inverse(self, matrix, close=None):
		m = hinverse(self.post) * mat3(matrix) * hinverse(self.pre)
		angle = atan2(m[1][0] - m[0][1], m[0][0] + m[1][1])
		angle += (2*pi) * ((angle + pi - close) // (2*pi))
		return angle
		
	def grad(self, angle, delta=1e-6):
		# the actual gradient is the same matrix but with a null homogeneous coordinate
		# leaving it to 1 is fine here because the kinematic solver will squeeze it anyway
		return self.post * drotate(angle, Z) * self.pre
		
	def transmit(self, force, parameters=None, velocity=None) -> Screw:
		l = force.locate(self.axis[0])
		return Screw(l.resulting, project(l.momentum, self.axis[1]))
		
	def schemes(self, size, attach_start, attach_end):
		radius = size/4
		axis = self.axis
		sch = Scheme()
		cylinder = gt.cylinder(
						axis[0]-axis[1]*size*0.4, 
						axis[0]+axis[1]*size*0.4, 
						radius=size/4, 
						resolution=('div',16),
						)
		sch.add(cylinder, shader='ghost')
		sch.add(cylinder.outlines(), shader='fill')
		if junc:
			v = normalize(noproject(junc - axis[0], axis[1]))
			if not isfinite(v):
				v,_,_ = dirbase(axis[1])
			p = center + v*size/4
			sch.extend(gt.flatsurface([p, p + axis[1]*size*cornersize, p + v*size*cornersize]), shader='ghost')
			sch.add([p, p + v*size*cornersize, junc], shader='fill')
		yield sch
		
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
		s1 = gt.flatsurface(c1)
		s2 = gt.flatsurface(c2)
		l = len(c1.points)
		indices = [(i-1,i)  for i in range(1,l-1)]
		indices.append((l-1,0))
		
		sch = Scheme([center-side, center+side, center+attach, junc], [], [], [(0,1), (2,3)])
		sch.extend(Scheme(c1.points, s1.faces, [], c1.edges()))
		sch.extend(Scheme(c2.points, s2.faces, [], c2.edges()))
		yield s


class Planar(Joint):
	''' Joint for translation in 2 directions and rotation around the third direction 
		
		Classical definition:	Planar (direction vector)
		the initial state requires an additional distance between the solids
		this class holds an axis for each side, the axis origins are constrained to share the same projections on the normal
	'''
	def __init__(self, solids, axis, local=None):
		self.solids = (s1, s2)
		self.axis = axis
		self.pre = transpose(mat3(quat(local or axis, Z)))
		self.post = transpose(mat3(quat(Z, axis)))
		self.offset = mat4(quat(axis, local or axis))
		self.position = axis[0]
		
	bounds = ((-inf, -inf), (inf, inf))
	
	def direct(self, position: vec2):
		return mat4(self.post) * translate(vec3(position)) * mat4(self.pre)
		
	def inverse(self, matrix, close=None):
		m = mat4(transpose(self.post)) * matrix * mat4(transpose(self.pre))
		return vec2(m[3])
		
	def grad(self, position: vec2, delta=1e-6):
		return (
			mat4(self.post) * translate(X) * mat4(self.pre),
			mat4(self.post) * translate(Y) * mat4(self.pre),
			)
	
	def transmitable(self, force, parameters=None, velocity=None):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Screw(project(action.resulting, normal), noproject(action.momentum, normal), action.position)
	
	
	def scheme(self, size, junc=None):
		(center, normal), position = self.axis[0], self.position[0]
		(center, normal), position = self.axis[1], self.position[1]
		
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
		
		Classical definition:  Track (direction vector)
		the initial state requires more parameters: the relative placements of the solids
		this class holds a base for each of the solids, bases are (0,X,Y)  where X,Y are constrained to keep the same direction across bases, and the bases origins lays on their common Z axis
	'''
	def __init__(self, solids, axis, local=None):
		self.solids = (s1, s2)
		self.axis = axis
		self.offset = mat4(quat(axis, local or axis))
		self.position = axis[0]
	
	bounds = (-inf, inf)
	
	def direct(self, translation):
		return translate(self.axis[1]*translation)
		
	def inverse(self, matrix, close=None):
		m = matrix * hinverse(self.offset)
		return dot(vec3(m[3])-self.axis[0], self.axis[1])
		
	def grad(self, translation, delta=1e-6):
		return translate(self.axis[1])
	
	def transmit(self, force, parameters=None, velocity=None):
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
		
		Classical definition:  Gliding pivot (axis)
		the initial state doesn't require more data
		this class holds an axis for each side
	'''
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		self.axis = axis
		local = local or axis
		self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		self.post = translate(axis[0]) * mat4(quat(Z, local[0]))
	
	bounds = ((-inf, -inf), (inf, inf))
	
	def direct(self, paramaters):
		return self.post * translate(parameters[1]*Z) * rotateZ(parameters[0]) * self.pre
		
	def inverse(self, matrix, close=None):
		m = hinverse(self.post) * matrix * hinverse(self.pre)
		return atan2(m[0][0]-m[1][0], m[0][1]+m[1][1]), m[3][2]
		
	def grad(self, parameters, delta=1e-6):
		return (
			self.post * translate(parameters[1]) * rotateZ(parameters[0]+pi/2) * self.pre,
			self.post * translate(Z) * rotateZ(parameters[0]) * self.pre,
			)
	
	def transmit(self, force, parameters=None, velocity=None):
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
	
		Classical definition: Ball (point)
		the initial state doen't require more data
		the class holds a point for each side
	'''
	def __init__(self, solids, center, local=None):
		self.solids = solids
		self.center = center
		self.post = center
		self.pre = -(local or center)
		
	bounds = (vec3(-1), vec3(1))
	
	def direct(self, angle):
		return translate(self.post) * rotate(1, angle) * translate(self.pre)
	
	def inverse(self, matrix, close=None):
		m = quat(matrix)
		return glm.angle(m) * glm.axis(m)
		
	def transmit(self, force, parameters=None, velocity=None):
		return Screw(vec3(0), force.momentum, self.center)
	
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


class Punctiform(Joint):
	''' Joint for rotation all around a point belonging to a plane.
	
		Classical definition: Punctiform/Sphere-Plane (axis)
		The initial state does not require more data
		The class holds a normal axis for the plane side, and a point for the sphere side (that defaults to the axis' origin)
	'''
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		self.axis = axis
		local = local or axis
		self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
		
	bounds = ([-inf]*5, [inf]*5)
	
	def direct(self, parameters):
		rotation = parameters[0:3]
		translation = parameters[3:5]
		return self.post * translate(vec3(translation)) * rotate(1, rotation) * self.pre
		
	def inverse(self, matrix, close=None):
		m = hinverse(self.post) * matrix * hinverse(self.pre)
		rotation = quat(m)
		return (
			*(glm.angle(m) * glm.axis(m)),
			*vec2(m[3]),
			)
	
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


class Ring(Joint):
	pass
	

class Hinge(Joint):
	pass


class Project(Joint):
	def __init__(self, solids, projection:mat4, target:vec3):
		indev

	
class Rack(Joint):
	pass
	
class Gear(Joint):
	''' Gear interaction between two solids.
	
		`ratio` is the factor from the rotation of s1 to the rotation of s2
		The two pinions are considered circular, but no assumption is made on their radius or relative inclination.
		The interaction is symetric.
	'''
	def __init__(self, solids, ratio, axis, local=None):
		self.solids = solids
		self.ratio = ratio
		self.axis = axis
		local = local or axis
		self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
		
	def direct(self, parameters):
		indev
	
	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:		i0,i1,n = 0, 1, self.ratio
		elif solid is self.solids[1]:	i0,i1,n = 1, 0, 1/self.ratio
		else:	return
		
		a0, a1 = solidtransform_axis(self.solids[0],self.axis[0]), solidtransform_axis(self.solids[1],self.axis[1])
		align = junc-self.axis[i0][0]
		if length2(align) == 0:		align = vec3(1,0,0)
		x,y,z = dirbase(self.axis[i0][1], align=align)
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
		
		d = angle*z + sqrt(max(0, 1-angle**2))*x
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


class Cam(Joint):
	pass
	
	
class Contact(Joint):
	pass
