# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .kinematic.solver import Joint
from .kinematic.displays import scale_solid, world_solid, kinematic_color
from .mesh import Mesh, Wire, wire, web, Web
from .scheme import Scheme
from . import generation as gt
from . import generation, primitives, settings

__all__ = ['Revolute', 'Planar', 'Prismatic', 'Cylindrical', 'Ball', 'PointSlider', 'EdgeSlider', 'Ring',
			'Cam', 'Contact',
			'Rack', 'Gear', 'Helicoid']


cornersize = 0.1	

def drotate(angle, axis):
	# derivative of sin and cos an argument translation of pi/2
	m = rotate(angle+0.5*pi, axis)
	# remove homogeneous one and axis proper space
	m -= mat4(outerProduct(axis, axis) / length2(axis))
	return m
	
def dtranslate(direction):
	return translate(direction) - mat4()


class Revolute(Joint):
	''' Joint for revolution around an axis
	'''
	bounds = (-inf, inf)
	default = 0
	
	def __init__(self, solids, axis: Axis, local=None, default=None):
		self.solids = solids
		local = local or axis
		
		if default is not None:
			self.default = default
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
			self.position = axis[3].xyz, local[3].xyz
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
			self.position = axis[0], local[0]
		else:
			raise TypeError('Revolute can only be placed on an Axis or a mat4')
		
	def __repr__(self):
		return '{}({}, Axis(({:.3g},{:.3g},{:.3g}), ({:.3g},{:.3g},{:.3g})))'.format(
			self.__class__.__name__, self.solids, *self.post[3].xyz, *self.post[2].xyz)
	
	def direct(self, angle) -> mat4:
		return self.post * rotate(angle, Z) * self.pre
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * mat3(matrix) * affineInverse(self.pre)
		angle = atan2(m[1][0] - m[0][1], m[0][0] + m[1][1])
		angle += (2*pi) * ((angle + pi - close) // (2*pi))
		return angle
		
	def grad(self, angle, delta=1e-6):
		return self.post * drotate(angle, Z) * self.pre
		
	def transmit(self, force, parameters=None, velocity=None) -> Screw:
		l = force.locate(self.axis[0])
		return Screw(l.resulting, project(l.momentum, self.axis[1]))
		
	def scheme(self, index, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = size/4
		sch = Scheme()
		resolution = ('div', 16)
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size))
		cylinder = gt.cylinder(
						-z*size*0.4, 
						+z*size*0.4, 
						radius=size/4, 
						resolution=resolution,
						fill=False,
						)
		sch.add(cylinder, shader='ghost')
		sch.add(cylinder.outlines(), shader='line')
		if attach_start:
			v = normalize(noproject(attach_start - o, z))
			if not isfinite(v):
				v = x
			p = v*size/4
			sch.add(gt.cone(p+size*cornersize*v, p, size*cornersize, fill=False, resolution=('div', 4)), shader='fill')
			sch.set(shader='line')
			sch.add(p + v*size*cornersize)
			sch.add(attach_start, space=world_solid(self.solids[0]))
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size))
		c1 = primitives.Circle(
						(-z*size*0.5, -z), 
						radius, 
						resolution=resolution,
						).mesh()
		c2 = primitives.Circle(
						(+z*size*0.5, z), 
						radius, 
						resolution=resolution,
						).mesh()
		# sch.add([z*size*0.5, o+z*size*0.5], shader='line')
		sch.add(c1, shader='line')
		sch.add(c2, shader='line')
		sch.add(gt.flatsurface(c1), shader='ghost')
		sch.add(gt.flatsurface(c2), shader='ghost')
		
		if attach_end:
			sch.set(shader='line')
			side = z * size * 0.5
			if dot(attach_end-o, side) < 0:
				side = -side
			if dot(attach_end, side)**2 < length2(attach_end)*length2(side) * 0.25:
				radial = normalize(noproject(attach_end-o, z))
				sch.add(side + radial*radius)
				sch.add(side + radial*2*radius)
			else:
				sch.add(side)
			sch.add(attach_end, space=world_solid(self.solids[1]))
		
		return sch

		
class Planar(Joint):
	''' Joint for translation in 2 directions and rotation around the third direction 
		
		Classical definition:	Planar (direction vector)
		the initial state requires an additional distance between the solids
		this class holds an axis for each side, the axis origins are constrained to share the same projections on the normal
	'''
		
	bounds = (vec3(-inf), vec3(inf))
	default = vec3(0, 0, 0)
	
	def __init__(self, solids, axis, local=None, default=None):
		self.solids = solids
		local = local or axis
		
		if default is not None:
			self.default = default
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
			self.position = axis[3].xyz, local[3].xyz
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
			self.position = axis[0], local[0]
		else:
			raise TypeError('Planar can only be placed on an Axis or a mat4')
	
	def __repr__(self):
		return '{}({}, Axis(({:.3g},{:.3g},{:.3g}), ({:.3g},{:.3g},{:.3g})))'.format(
			self.__class__.__name__, self.solids, *self.post[3].xyz, *self.post[2].xyz)
	
	def direct(self, position: tuple):
		return mat4(self.post) * translate(vec3(position.xy, 0)) * rotate(position.z, Z) * mat4(self.pre)
		
	def inverse(self, matrix, close=None):
		m = mat4(transpose(self.post)) * matrix * mat4(transpose(self.pre))
		angle = atan2(m[0][1], m[0][0])
		angle += (2*pi) * ((angle + pi - close) // (2*pi))
		return vec3(m[3].xy, angle)
		
	def grad(self, position: vec3, delta=1e-6):
		return (
			self.post * translate(X) * rotate(position.z, Z) * self.pre,
			self.post * translate(Y) * rotate(position.z, Z) * self.pre,
			self.post * translate(vec3(position.xy,0)) * drotate(position.z, Z) * self.pre,
			)
	
	def transmitable(self, force, parameters=None, velocity=None):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Screw(project(action.resulting, normal), noproject(action.momentum, normal), action.position)
	
	
	def scheme(self, index, maxsize, attach_start, attach_end):		
		size = settings.display['joint_size']
		sch = Scheme()
		gap = 0.1
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size),
			)
		square = gt.parallelogram(x*size, y*size, origin=o-gap*size*z, align=0.5)
		sch.add(square.outlines(), shader='line')
		sch.add(square, shader='ghost')
		
		if attach_start:
			sch.add(gt.cone(o-size*(gap+cornersize)*z, o-size*gap*z, size*cornersize, fill=False, resolution=('div', 4)), shader='fill')
			sch.set(shader='line')
			sch.add(o-size*(gap+cornersize)*z)
			sch.add(attach_start, space=world_solid(self.solids[0]))
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = index[self.solids[1]],
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size),
			)
		square = gt.parallelogram(-x*size, y*size, origin=o+gap*size*z, align=0.5)
		sch.add(square.outlines(), shader='line')
		sch.add(square, shader='ghost')
		
		if attach_end:
			sch.add(gt.cone(o+size*(gap+cornersize)*z, o+size*gap*z, size*cornersize, fill=False, resolution=('div', 4)), shader='fill')
			sch.set(shader='line')
			sch.add(o+size*(gap+cornersize)*z)
			sch.add(attach_end, space=world_solid(self.solids[1]))
		
		return sch


class Prismatic(Joint):
	''' Joint for translation only in a direction 
		
		Classical definition:  Prismatic (direction vector)
		the initial state requires more parameters: the relative placements of the solids
		this class holds a base for each of the solids, bases are (0,X,Y)  where X,Y are constrained to keep the same direction across bases, and the bases origins lays on their common Z axis
	'''
	
	bounds = (-inf, inf)
	default = 0
	
	def __init__(self, solids, axis: Axis, local=None, default=None):
		self.solids = solids
		local = local or axis
		
		if default is not None:
			self.default = default
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
			self.position = axis[3].xyz, local[3].xyz
		elif isinstance(axis, vec3):
			self.post = mat4(quat(Z, axis))
			self.pre = mat4(quat(local, Z))
			self.position = axis, local
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
			self.position = axis[0], local[0]
		else:
			raise TypeError('Track can only be placed on a vec3, Axis or mat4')
	
	def __repr__(self):
		return '{}({}, vec3({:.3g},{:.3g},{:.3g}))'.format(
			self.__class__.__name__, self.solids, *self.post[3].xyz)
	
	def direct(self, translation):
		return self.post * translate(translation*Z) * self.pre
		
	def inverse(self, matrix, close=None):
		return (affineInverse(self.post) * matrix * affineInverse(pre))[2][2]
		
	def grad(self, translation, delta=1e-6):
		return self.post * dtranslate(Z) * self.pre
	
	def transmit(self, force, parameters=None, velocity=None):
		z0, z1 = cross(b0[1],b0[2]), cross(b1[1],b1[2])
		normal = normalize(z0 + z1)
		return Screw(noproject(action.resulting, normal), action.momentum, action.position)
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size),
			)
		profile = gt.parallelogram(0.4*size*x, 0.4*size*y, origin=o, align=0.5, fill=False)
		profile.tracks = typedlist(range(len(profile.edges)))
		exterior = gt.extrusion(profile, size*z, alignment=0.5)
		exterior.splitgroups()
		sch.add(exterior, shader='ghost')
		sch.add(exterior.outlines(), shader='line')
		if attach_start:
			v = attach_start - o
			if abs(dot(v,x)) > abs(dot(v,y)):
				v = x if dot(v,x)>0 else -x
			else:
				v = y if dot(v,y)>0 else -y
			p = o + v*0.2*size
			sch.add(gt.cone(p+size*cornersize*v, p, size*cornersize, fill=False, resolution=('div', 4)), shader='fill')
			sch.set(shader='line')
			sch.add(p+size*cornersize*v)
			sch.add(attach_start, space=world_solid(self.solids[0]))
	
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size),
			)
		interior = gt.extrusion(Web([
			o-0.2*size*(x+y),
			o+0.2*size*(x+y),
			o-0.2*size*(x-y),
			o+0.2*size*(x-y),
			],
			[(0,1), (2,3)],
			), size*z, alignment=0.5)
		sch.add(interior.outlines(), shader='line')
		if attach_end:
			v = attach_end - o
			v = z if dot(v, z) > 0 else -z
			p = o + v*size/2
			sch.add(p)
			sch.add(attach_end, space=world_solid(self.solids[1]))
			
		return sch


class Cylindrical(Joint):
	''' Joint with rotation and translation around an axis
	'''
	bounds = (vec2(-inf), vec2(inf))
	default = vec2(0)
	
	def __init__(self, solids, axis: Axis, local=None, default=None):
		self.solids = solids
		local = local or axis
		
		if default is not None:
			self.default = default
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
			self.position = axis[3].xyz, local[3].xyz
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
			self.position = axis[0], local[0]
		else:
			raise TypeError('Pivot can only be placed on an Axis or a mat4')
		
	def __repr__(self):
		return '{}({}, Axis(({:.3g},{:.3g},{:.3g}), ({:.3g},{:.3g},{:.3g})))'.format(
			self.__class__.__name__, self.solids, *self.post[3].xyz, *self.post[2].xyz)
	
	def direct(self, parameters) -> mat4:
		return self.post * rotate(parameters[0], Z) * translate(parameters[1]*Z) * self.pre
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * mat4(matrix) * affineInverse(self.pre)
		angle = atan2(m[1][0] - m[0][1], m[0][0] + m[1][1])
		angle += (2*pi) * ((angle + pi - close) // (2*pi))
		return angle, m[2][2]
		
	def grad(self, parameters, delta=1e-6):
		return (
			self.post * drotate(parameters[0], Z) * self.pre,
			self.post * dtranslate(Z) * self.pre,
			)
		
	def transmit(self, force, parameters=None, velocity=None) -> Screw:
		l = force.locate(self.axis[0])
		return Screw(l.resulting, project(l.momentum, self.axis[1]))
		
	def scheme(self, index, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = size/4
		sch = Scheme()
		resolution = ('div', 16)
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size))
		cylinder = gt.cylinder(
						o-z*size*0.4, 
						o+z*size*0.4, 
						radius=size/4, 
						resolution=resolution,
						fill=False,
						)
		sch.add(cylinder, shader='ghost')
		sch.add(cylinder.outlines(), shader='line')
		if attach_start:
			v = normalize(noproject(attach_start - o, z))
			if not isfinite(v):
				v = x
			p = v*size/4
			sch.add(gt.cone(p+size*cornersize*v, p, size*cornersize, fill=False, resolution=('div', 4)), shader='fill')
			sch.set(shader='line')
			sch.add(p + v*size*cornersize)
			sch.add(attach_start, space=world_solid(self.solids[0]))
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size))
		sch.add([o-z*size*0.5, o+z*size*0.5], shader='line')
		
		sch.set(shader='fill')
		if attach_end:
			sch.set(shader='line')
			side = z * size * 0.5
			if dot(attach_end-o, side) < 0:
				side = -side
			if dot(attach_end, side)**2 < length2(attach_end)*length2(side) * 0.25:
				radial = normalize(noproject(attach_end-o, z))
				sch.add(side + radial*radius)
				sch.add(side + radial*2*radius)
			else:
				sch.add(side)
			sch.add(attach_end, space=world_solid(self.solids[1]))
			
		return sch

class Ball(Joint):
	''' Joint for rotation all around a point.
	
		Classical definition: Ball (point)
		the initial state doen't require more data
		the class holds a point for each side
	'''
	def __init__(self, solids, center, local=None, default=None):
		self.solids = solids
		local = local or center
		
		if default is not None:
			self.default = default
		
		if isinstance(center, mat4):
			self.post = center
			self.pre = affineInverse(local)
			self.position = center[3].xyz, local[3].xyz
		elif isinstance(center, vec3):
			self.post = translate(center)
			self.pre = translate(-local)
			self.position = center, local
		else:
			raise TypeError("ball can only be placed on a vec3 or mat4")
	
	bounds = (quat(-2,-2,-2,-2), quat(2,2,2,2))
	default = quat()
			
	def normalize(self, orient):
		return normalize(quat(orient))
	
	def direct(self, orient):
		return self.post * mat4(normalize(quat(orient))) * self.pre
	
	def inverse(self, matrix, close=None):
		return quat(affineInverse(self.post) * matrix * affineInverse(self.pre))
		
	def grad(self, orient):
		a,b,c,d = orient
		return (
			self.post * mat4( 
				 2*a,  2*d, -2*c, 0,
				-2*d,  2*a,  2*b, 0,
				 2*c, -2*b,  2*a, 0,
				 0,   0,     0,   0,
				) * self.pre,
			self.post * mat4(
				 2*b,  2*c,  2*d, 0,
				 2*c, -2*b,  2*a, 0,
				 2*d, -2*a, -2*b, 0,
				 0,    0,    0,   0,
				) * self.pre,
			self.post * mat4(
				-2*c,  2*b, -2*a, 0,
				 2*b,  2*c,  2*d, 0,
				 2*a,  2*d, -2*c, 0,
				 0,    0,    0,   0,
				) * self.pre,
			self.post * mat4(
				-2*d,  2*a,  2*b, 0,
				-2*a, -2*d,  2*c, 0,
				 2*b,  2*c,  2*d, 0,
				 0,    0,    0,   0,
				) * self.pre,
			)
	
	def transmit(self, force, parameters=None, velocity=None):
		return Screw(vec3(0), force.momentum, self.center)
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		from .primitives import ArcCentered
		
		size = settings.display['joint_size']
		radius = size/3
		sch = Scheme()
		resolution = ('div', 16)
		
		center = self.post[3].xyz
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(center), maxsize/size))
		if attach_start:
			x,y,z = dirbase(normalize(center - attach_start))
		else:
			x,y,z = mat3()
		profile = ArcCentered(Axis(O,y), x*radius, -z*radius).mesh(resolution=resolution)
		emisphere = gt.revolution(profile, Axis(O,z), resolution=resolution)
		sch.add(emisphere, shader='ghost')
		sch.add(emisphere.outlines(), shader='line')
		if attach_start:
			sch.set(shader='line')
			sch.add(-z*radius)
			sch.add(attach_start, space=world_solid(self.solids[0]))
		
		center = affineInverse(self.pre)[3].xyz
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(center), maxsize/size))
		sch.add(gt.icosphere(O, radius*0.8, resolution=('div', 3)), shader='ghost')
		if attach_end:
			sch.set(shader='line')
			sch.add(radius*0.8*normalize(attach_end-center))
			sch.add(attach_end, space=world_solid(self.solids[1]))
		
		return sch


class PointSlider(Joint):
	''' Joint for rotation all around a point belonging to a plane.
	
		Classical definition: Punctiform/Sphere-Plane (axis)
		The initial state does not require more data
		The class holds a normal axis for the plane side, and a point for the sphere side (that defaults to the axis' origin)
	'''
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		self.axis = axis
		local = local or axis
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		else:
			raise TypeError("PointSlider can only be placed on Axis or mat4")
	
	default = [0]*5
	bounds = ([-inf]*5, [inf]*5)
	
	def direct(self, parameters):
		translation = vec2(parameters[0:2])
		rotation = quat(parameters[2:5])
		return self.post * translate(vec3(translation, 0)) * mat4(rotation) * self.pre
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * matrix * affineInverse(self.pre)
		return (
			*vec2(m[3]),
			*quat(m),
			)
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = size * 0.2
		resolution = ('div', 16)
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size),
			)
		square = gt.parallelogram(x*size, y*size, origin=o+radius*z, align=0.5)
		sch.add(square.outlines(), shader='line')
		sch.add(square, shader='ghost')
		
		if attach_start:
			attach = o+radius*z
			sch.add([attach, attach+z*0.1*size, attach+(x+y)*0.1*size], shader='fill')
			sch.add([attach+cornersize*size*z, attach_start], shader='line')
		
		center = affineInverse(self.pre)[3].xyz
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(center), maxsize/size),
			)
		sch.add(gt.icosphere(center, radius, resolution), shader='ghost')
		
		if attach_end:
			x,y,z = dirbase(attach_end)
			attach = radius*z
			sch.add([attach, attach+z*cornersize*size, attach+(x+y)*cornersize*size], shader='fill')
			sch.add([attach+cornersize*size*z, attach_start], shader='line')
		
		return sch


class EdgeSlider(Joint):
	''' Joint for rotation all around a point belonging to a plane.
	
		Classical definition: Punctiform/Sphere-Plane (axis)
		The initial state does not require more data
		The class holds a normal axis for the plane side, and a point for the sphere side (that defaults to the axis' origin)
	'''
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		self.axis = axis
		local = local or axis
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
		else:
			raise TypeError("EdgeSlider can only be placed on Axis or mat4")
	
	default = [0]*4
	bounds = ([-inf]*4, [inf]*4)
	
	def direct(self, parameters):
		translation = vec2(parameters[0:2])
		rotation = quat(vec3(*parameters[2:4], 0))
		return self.post * translate(vec3(translation, 0)) * mat4(rotation) * self.pre
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * matrix * affineInverse(self.pre)
		q = quat(m)
		return (
			*vec2(m[3]),
			pitch(q),
			roll(q),
			)
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = 0.2*size
		resolution = ('div', 16)
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size),
			)
		square = gt.parallelogram(x*size, y*size, origin=o+radius*z, align=0.5)
		sch.add(square.outlines(), shader='line')
		sch.add(square, shader='ghost')
		
		if attach_start:
			attach = o+radius*z
			sch.add([attach, attach+cornersize*size*z, attach+cornersize*size*(x+y)], shader='fill')
			sch.add([o+cornersize*size*z, attach_start], shader='line')
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size),
			)
		cylinder = gt.cylinder(o-x*0.5*size, o+x*0.5*size, radius, fill=False, resolution=resolution)
		sch.add(cylinder, shader='ghost')
		sch.add(cylinder.outlines(), shader='line')
		
		if attach_end:
			x,y,z = dirbase(noproject(attach_end, x))
			attach = radius*z
			sch.add([attach, attach+z*cornersize*size, attach+(x+y)*cornersize*size], shader='fill')
		
		return sch
	

class Ring(Joint):
	''' ring joint '''
	def __init__(self, solids, center, local=None):
		self.solids = solids
		local = local or center
		
		if isinstance(center, mat4):
			self.pre = center
			self.post = local
		elif isinstance(center, vec3):
			self.pre = translate(-center)
			self.post = translate(local)
		else:
			raise TypeError("ball can only be placed on a vec3 or mat4")
	
	bounds = ((-inf, -1, -1, -1, -1), (inf, 1, 1, 1, 1))
	default = (0, 1, 0, 0, 0)
	
	def direct(self, orient):
		return self.post * translate(params[0]) * mat4(quat(params[1:5])) * self.pre
	
	def inverse(self, matrix, close=None):
		return quat(affineInverse(self.post) * matrix * affineInverse(self.pre))
	
	def transmit(self, force, parameters=None, velocity=None):
		return Screw(vec3(0), force.momentum, self.center)
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		from .primitives import ArcCentered
		
		size = settings.display['joint_size']
		radius = size/3
		sch = Scheme()
		resolution = ('div', 16)
		
		center = self.post[3].xyz
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(center), maxsize/size))
		if attach_start:
			x,y,z = dirbase(attach_end - center)
		else:
			x,y,z = mat3()
		profile = ArcCentered(Axis(center,y), center+x*radius, center-z*radius).mesh(resolution=resolution)
		emisphere = gt.revolution(profile, Axis(center,z), resolution)
		sch.add(emisphere, shader='ghost')
		sch.add(emisphere.outlines(), shader='line')
		
		center = affineInverse(self.pre)[3].xyz
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(center), maxsize/size))
		sch.add(gt.icosphere(center, radius*0.8, resolution), shader='ghost')
		
		return sch

	
class Universal(Joint):
	''' universal joint '''
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		local = local or axis
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		else:
			raise TypeError('Universal joint can only be placed on an Axis or a mat4')
	
	bounds = (vec2(-inf), vec2(inf))
	default = vec2(0)
	
	def direct(self, orient):
		return self.post * mat4(quat(vec3(orient, 0))) * self.pre
	
	def inverse(self, matrix, close=None):
		m = quat(affineInverse(self.post) * matrix * affineInverse(self.pre))
		return vec2(pitch(m), yaw(m))
		
	def grad(self, orient):
		pitch, yaw = orient
		return (
			self.post * drotate(pitch, X) * self.pre,
			self.post * drotate(pitch, Y) * self.pre,
			)
			
	def scheme(self, index, maxsize, attach_start, attach_end):
		from .primitives import ArcCentered
		
		size = settings.display['joint_size']
		radius = size/3
		sch = Scheme()
		resolution = ('div', 16)
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size))
		if attach_start:
			o = self.post[3].xyz
			x,y,z = dirbase(attach_end - center, x)
			sch.add(attach_start, space=world_solid(self.solids[0]))
			sch.add(o - z*radius)
		sch.add(ArcCentered(Axis(o,x), y*radius, -y*radius).mesh(resolution), shader='line')
		sch.add([y, -y])
		
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size))
		x,y,z,o = affineInverse(self.pre)
		if attach_end:
			o = affineInverse(self.pre)[3].xyz
			x,y,z = dirbase(attach_end - center, y)
			sch.add(attach_end, space=world_solid(self.solids[1]))
			sch.add(o - z*radius)
		sch.add(ArcCentered(Axis(o,y), x*radius, -x*radius).mesh(resolution), shader='line')
		sch.add([x, -x])
		
		return sch
		
	
	
class ConstantVelocity(Joint):
	''' not implemented yet '''
	pass
	

class Project(Joint):
	''' not implemented yet '''
	def __init__(self, solids, projection:mat4, target:mat4):
		indev

	
class Rack(Joint):
	''' Rack to gear interaction
		
		`radius` is the ratio between the gear rotation and the rack translation
		
		Attributes:
			radius:  ratio between the gear rotation and the rack translation
			rack:    rack axis tangent to the gear
			axis:    gear axis
	'''
	def __init__(self, solids, radius:float, rack:Axis, axis):
		self.solids = solids
		self.radius = radius
		self.axis = axis
		self.rack = rack
		self.pre = mat4(quat(axis[1], Z)) * translate(-axis[0])
		self.post = translate(rack[0]) * mat4(quat(X, rack[1]))
		self.position = self.rack[0], self.axis[0]
		
	bounds = (-inf, inf)
	default = 0
	
	def direct(self, translation):
		return self.post * translate(translation*X) * rotate(-translation/self.radius,Z) * self.pre
	
	def grad(self, translation):
		return (
			+ self.post * dtranslate(X) * rotate(-translation/self.radius,Z) * self.pre
			+ self.post * translate(translation*X) * drotate(-translation/self.radius,Z)/-self.radius * self.pre
			)
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * matrix * affineInverse(self.pre)
		return atan(*m[0].xy)
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		from .generation import revolution
		from .primitives import Circle, Segment
		sch = Scheme()

		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = world_solid(self.solids[0]))
		sch.add([o-self.radius*y-self.radius*x, o-self.radius*y+self.radius*x], shader='line')
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		h = 0.1*self.radius*z
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = world_solid(self.solids[1]))
		sch.add(Circle(Axis(o,z), self.radius).mesh(), shader='line')
		sch.add(revolution(Segment(o+self.radius*x+h, o+self.radius*x-h), Axis(o,z)), shader='ghost')
		
		return sch

		
	

class Gear(Joint):
	''' Gear interaction between two solids.
	
		`ratio` is the factor from the rotation of s1 to the rotation of s2
		The two pinions are considered circular, but no assumption is made on their radius or relative inclination.
		The interaction is symetric.
		
		Attributes:
			ratio:  the gear rotation transmission ratio
			centerline: the relative positioning of the gears axis, could be 'float', `vec3`, `mat3`, `quat`, `mat4`
			axis:  first gear axis
			local:  second gear axis
	'''
	def __init__(self, solids, ratio:float, centerline:mat4, axis, local):
		self.solids = solids
		self.ratio = ratio
		if isinstance(centerline, (int,float)):
			centerline = centerline*X
		self.centerline = transform(centerline)
		self.axis = axis
		self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
		self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
		self.position = self.axis[0], local[0]
		
	bounds = (-inf, inf)
	default = 0
		
	def direct(self, angle):
		return self.post * rotate(angle,Z) * self.centerline * rotate(-self.ratio*angle,Z) * self.pre
		
	def grad(self, angle):
		return (
			+ self.post * drotate(angle,Z) * self.centerline * rotate(-self.ratio*angle,Z) * self.pre
			+ self.post * rotate(angle,Z) * self.centerline * drotate(-self.ratio*angle,Z)*-self.ratio * self.pre
			)
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * matrix * affineInverse(self.pre)
		angle = atan(*m[0].xy) / (1+self.ratio)
		return (angle - close + pi) % (2*pi) - pi
	
	def scheme(self, index, maxsize, attach_start, attach_end):
		from .generation import revolution
		from .primitives import Circle, Segment
		sch = Scheme()
		
		# radial vector
		r0 = length(self.centerline[3].xy) / abs(self.centerline[2].z - 1/self.ratio)
		r1 = r0 / abs(self.ratio)
		
		x,y,z,o = dmat4x3(self.post)
		h = 0.1*min(r0, r1) * (z * r0 + self.centerline[2].xyz * r1) / (r0 + r1)
		sch.set(
			track = index[self.solids[0]], 
			color = kinematic_color(index[self.solids[0]]),
			space = world_solid(self.solids[0]))
		sch.add(Circle(Axis(o,z), r0).mesh(), shader='line')
		sch.add(revolution(Segment(o+r0*x+h, o+r0*x-h), Axis(o,z)), shader='ghost')
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		h = 0.1*min(r0, r1) * (self.centerline[2].xyz * r0 + z * r1) / (r0 + r1)
		sch.set(
			track = index[self.solids[1]], 
			color = kinematic_color(index[self.solids[1]]),
			space = world_solid(self.solids[1]))
		sch.add(Circle(Axis(o,z), r1).mesh(), shader='line')
		sch.add(revolution(Segment(o+r1*x+h, o+r1*x-h), Axis(o,z)), shader='ghost')
		
		return sch


class Helicoid(Joint):
	''' Screw a solid into an other.
	
		`step` is the translation distance needed for that one solid turn by 2*pi around the other
		The interaction is symetric.
		
		not implemented yet
	'''
	def __init__(self, s0, s1, step, b0, b1=None, position=None):
		self.step = step	# m/tr
		self.solids = s0, s1
		if isaxis(b0):			b0 = b0[0], *dirbase(b0[1])[:2]
		if b1 and isaxis(b1):	b1 = b1[0], *dirbase(b1[1])[:2]
		self.bases = b0, b1 or b0
		self.position = position or (self.bases[0][0], self.bases[1][0])
	
	def direct(self, depth):
		indev
		
	def inverse(self, matrix, close=None):
		indev
	
	
	def scheme(self, solid, size, junc):
		if solid is self.solids[0]:
			radius = size/4
			o,x,y = self.bases[0]
			z = cross(x,y)
			center = o + project(self.position[0]-o, z)
			cyl = generation.extrusion(
						web(primitives.Circle(
							(center-z*size*0.5, z), 
							radius, 
							resolution=('div', 16),
							)),
						z*size, 
						)
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
	''' not implemented yet '''
	pass
	
	
class Contact(Joint):
	''' not implemented yet '''
	pass

