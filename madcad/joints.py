# This file is part of pymadcad,  distributed under license LGPL v3

from dataclasses import dataclass
import numpy as np

from .mathutils import *
from .primitives import isaxis, Axis
from .kinematic import Screw, Joint, scale_solid, world_solid
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
	dtype = np.dtype(float)
	
	def __init__(self, solids, axis: Axis, local=None):
		self.solids = solids
		local = local or axis
		
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
		return self.post * rotate(float(angle), Z) * self.pre
		
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
		
	def scheme(self, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = size/4
		sch = Scheme()
		resolution = ('div', 16)
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(track=0, space=scale_solid(self.solids[0], fvec3(o), maxsize/size))
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
			sch.add(gt.flatsurface(wire([p, p + z*size*cornersize, p + v*size*cornersize])), shader='fill')
			sch.set(shader='line')
			sch.add(p)
			sch.add(p + v*size*cornersize)
			sch.add(attach_start, space=world_solid(self.solids[0]))
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(track=1, space=scale_solid(self.solids[1], fvec3(o), maxsize/size))
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
		sch.add([z*size*0.5, o+z*size*0.5], shader='line')
		sch.add(c1, shader='line')
		sch.add(c2, shader='line')
		sch.add(gt.flatsurface(c1), shader='ghost')
		sch.add(gt.flatsurface(c2), shader='ghost')
		
		if attach_end:
			sch.set(shader='line')
			side = z * size * 0.5
			if dot(attach_end-o, side) < 0:
				side = -side
			if dot(attach_end-o-side, side) < 0:
				radial = normalize(noproject(attach_end-o, z))
				sch.add(side + radial*radius)
				sch.add(side + radial*2*radius)
				# sch.add(radial*2*radius)
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
	dtype = np.dtype((float, 3))
	
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		local = local or axis
		
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
	
	def direct(self, position: vec3):
		return mat4(self.post) * translate(vec3(position.xy)) * rotate(position.z, Z) * mat4(self.pre)
		
	def inverse(self, matrix, close=None):
		m = mat4(transpose(self.post)) * matrix * mat4(transpose(self.pre))
		angle = atan2(m[0][1], m[0][0])
		angle += (2*pi) * ((angle + pi - close) // (2*pi))
		return vec3(m[3].xy, angle)
		
	def grad(self, position: vec3, delta=1e-6):
		return (
			self.post * translate(X) * rotate(position.z, Z) * self.pre,
			self.post * translate(Y) * rotate(position.z, Z) * self.pre,
			self.post * translate(position.xy) * drotate(position.z, Z) * self.pre,
			)
	
	def transmitable(self, force, parameters=None, velocity=None):
		a0,a1 = solidtransform_axis(self.solids[0], self.axis[0])
		normal = normalize(a0[1] + a1[1])
		return Screw(project(action.resulting, normal), noproject(action.momentum, normal), action.position)
	
	
	def scheme(self, maxsize, attach_start, attach_end):		
		size = settings.display['joint_size']
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = 0, 
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size),
			)
		square = gt.parallelogram(x*size, y*size, origin=o+0.1*size*z, align=0.5)
		sch.add(square.outlines(), shader='line')
		sch.add(square, shader='ghost')
		
		if attach_start:
			sch.add([o, o+z*0.1*size, o+(x+y)*0.1*size], shader='fill')
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = 1,
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size),
			)
		square = gt.parallelogram(-x*size, y*size, origin=o-0.1*size*z, align=0.5)
		sch.add(square.outlines(), shader='line')
		sch.add(square, shader='ghost')
		
		if attach_end:
			sch.add([o, o+z*0.1*size, o+(x+y)*0.1*size], shader='fill')
		
		return sch


class Prismatic(Joint):
	''' Joint for translation only in a direction 
		
		Classical definition:  Track (direction vector)
		the initial state requires more parameters: the relative placements of the solids
		this class holds a base for each of the solids, bases are (0,X,Y)  where X,Y are constrained to keep the same direction across bases, and the bases origins lays on their common Z axis
	'''
	dtype = np.dtype(float)
	
	def __init__(self, solids, axis: Axis, local=None):
		self.solids = solids
		local = local or axis
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
		elif isinstance(axis, vec3):
			self.post = mat4(quat(Z, axis))
			self.pre = mat4(quat(local, Z))
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
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
	
	def scheme(self, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = 0, 
			space = scale_solid(self.solids[0], fvec3(o), maxsize/size),
			)
		profile = gt.parallelogram(0.4*size*x, 0.4*size*y, origin=o, align=0.5, fill=False)
		profile.tracks = typedlist(range(len(profile.edges)))
		exterior = gt.extrusion(size*z, profile, alignment=0.5)
		exterior.splitgroups()
		sch.add(exterior, shader='ghost')
		sch.add(exterior.outlines(), shader='line')
		if attach_start:
			v = attach_start - o
			if abs(dot(v,x)) > abs(dot(v,y)):
				v = x if dot(v,x)>0 else -x
			else:
				v = y if dot(v,y)>0 else -y
				p = o + v*0.25*size
				sch.add([p + z*size*cornersize, attach_start], shader='line')
				sch.add([p, p + z*size*cornersize, v*size*cornersize], shader='fill')
	
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(
			track = 1, 
			space = scale_solid(self.solids[1], fvec3(o), maxsize/size),
			)
		interior = gt.extrusion(size*z, Web([
			o-0.15*size*(x+y),
			o+0.15*size*(x+y),
			o-0.15*size*(x-y),
			o+0.15*size*(x-y),
			],
			[(0,1), (2,3)],
			), alignment=0.5)
		# sch.add(interior, shader='ghost')
		sch.add(interior.outlines(), shader='line')
		if attach_end:
			v = junc - o
			v = z if dot(v, z) > 0 else -z
			p = o + v*size/2
			sch.add([p, attach_end])
			
		return sch


class Cylindrical(Joint):
	dtype = np.dtype([('rotation', float), ('translation', float)])
	
	def __init__(self, solids, axis: Axis, local=None):
		self.solids = solids
		local = local or axis
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
		elif isinstance(axis, Axis):
			self.post = translate(axis[0]) * mat4(quat(Z, axis[1]))
			self.pre = mat4(quat(local[1], Z)) * translate(-local[0])
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
		
	def grad(self, angle, delta=1e-6):
		return (
			self.post * drotate(angle, Z) * self.pre,
			self.post * dtranslate(Z) * self.pre,
			)
		
	def transmit(self, force, parameters=None, velocity=None) -> Screw:
		l = force.locate(self.axis[0])
		return Screw(l.resulting, project(l.momentum, self.axis[1]))
		
	def scheme(self, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = size/4
		sch = Scheme()
		resolution = ('div', 16)
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(track=0, space=scale_solid(self.solids[0], fvec3(o), maxsize/size))
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
			sch.add(gt.flatsurface(wire([p, p + z*size*cornersize, p + v*size*cornersize])), shader='fill')
			sch.set(shader='line')
			sch.add(p)
			sch.add(p + v*size*cornersize)
			sch.add(attach_start, space=world_solid(self.solids[0]))
		
		x,y,z,o = dmat4x3(affineInverse(self.pre))
		sch.set(track=1, space=scale_solid(self.solids[1], fvec3(o), maxsize/size))
		sch.add([o-z*size*0.5, o+z*size*0.5], shader='line')
		
		sch.set(shader='fill')
		if attach_end:
			sch.set(shader='line')
			side = z * size * 0.5
			if dot(attach_end-o, side) < 0:
				side = -side
			if dot(attach_end-o-side, side) < 0:
				radial = normalize(noproject(attach_end-o, z))
				sch.add(side + radial*radius)
				sch.add(side + radial*2*radius)
				# sch.add(radial*2*radius)
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
	
	dtype = np.dtype((float, 4))
	bounds = (quat(-1, -1, -1, -1), quat(1, 1, 1, 1))
	default = quat()
	
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
	
	def direct(self, orient):
		return self.post * mat4(quat(normalize(orient))) * self.pre
	
	def inverse(self, matrix, close=None):
		return quat(affineInverse(self.post) * matrix * affineInverse(self.pre))
	
	def transmit(self, force, parameters=None, velocity=None):
		return Screw(vec3(0), force.momentum, self.center)
	
	def scheme(self, maxsize, attach_start, attach_end):
		from .primitives import ArcCentered
		
		size = settings.display['joint_size']
		radius = size/3
		sch = Scheme()
		resolution = ('div', 16)
		
		center = self.post[3].xyz
		sch.set(track=0, space=scale_solid(self.solids[0], fvec3(center), maxsize/size))
		if attach_end:
			x,y,z = dirbase(attach_end - center)
		else:
			x,y,z = mat3()
		profile = ArcCentered(Axis(center,y), center+x*radius, center-z*radius).mesh(resolution=resolution)
		emisphere = gt.revolution(2*pi, Axis(center,z), profile, resolution)
		sch.add(emisphere, shader='ghost')
		sch.add(emisphere.outlines(), shader='line')
		
		center = affineInverse(self.pre)[3].xyz
		sch.set(track=1, space=scale_solid(self.solids[1], fvec3(center), maxsize/size))
		sch.add(gt.icosphere(center, radius*0.8, resolution), shader='ghost')
		
		return sch


class PointSlider(Joint):
	''' Joint for rotation all around a point belonging to a plane.
	
		Classical definition: Punctiform/Sphere-Plane (axis)
		The initial state does not require more data
		The class holds a normal axis for the plane side, and a point for the sphere side (that defaults to the axis' origin)
	'''
	dtype = np.dtype([('translation', float, 2), ('rotation', float, 4)])
	bounds = ((-inf, quat(-1, -1, -1, -1)), (inf, quat(1, 1, 1, 1)))
	default = (0, quat())
	
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
	
	def direct(self, parameters):
		translation = vec2(parameters['translation'])
		rotation = quat(parameters['rotation'])
		return self.post * translate(vec3(translation, 0)) * mat4(rotation) * self.pre
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * matrix * affineInverse(self.pre)
		return (
			*vec2(m[3]),
			*quat(m),
			)
	
	def scheme(self, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = size * 0.2
		resolution = ('div', 16)
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = 0, 
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
		sch.set(track=1, space=scale_solid(self.solids[1], fvec3(center), maxsize/size))
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
	dtype = np.dtype([('translation', float, 2), ('rotation', float, 2)])
	
	def __init__(self, solids, axis, local=None):
		self.solids = solids
		self.axis = axis
		local = local or axis
		
		if isinstance(axis, mat4):
			self.post = axis
			self.pre = affineInverse(local)
		else:
			raise TypeError("EdgeSlider can only be placed on Axis or mat4")
		
	def direct(self, parameters):
		translation = vec2(parameters['translation'])
		rotation = quat(vec3(parameters['rotation'], 0))
		return self.post * translate(vec3(translation, 0)) * mat4(rotation) * self.pre
		
	def inverse(self, matrix, close=None):
		m = affineInverse(self.post) * matrix * affineInverse(self.pre)
		q = quat(m)
		return (
			*vec2(m[3]),
			pitch(q),
			roll(q),
			)
	
	def scheme(self, maxsize, attach_start, attach_end):
		size = settings.display['joint_size']
		radius = 0.2*size
		resolution = ('div', 16)
		sch = Scheme()
		
		x,y,z,o = dmat4x3(self.post)
		sch.set(
			track = 0, 
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
		sch.set(track=1, space=scale_solid(self.solids[1], fvec3(o), maxsize/size))
		cylinder = gt.cylinder(o-x*0.5*size, o+x*0.5*size, radius, fill=False, resolution=resolution)
		sch.add(cylinder, shader='ghost')
		sch.add(cylinder.outlines(), shader='line')
		
		if attach_end:
			x,y,z = dirbase(noproject(attach_end, x))
			attach = radius*z
			sch.add([attach, attach+z*cornersize*size, attach+(x+y)*cornersize*size], shader='fill')
		
		return sch
	

class Ring(Joint):
	dtype = np.dtype([('translation', float), ('rotation', float, 4)])
	default = (0, quat())
	bounds = ((-inf, quat(-1, -1, -1, -1)), (inf, quat(1, 1, 1, 1)))

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
	
	
	def direct(self, orient):
		return self.post * translate(params['translation']) * mat4(quat(params['rotation'])) * self.pre
	
	def inverse(self, matrix, close=None):
		return quat(affineInverse(self.post) * matrix * affineInverse(self.pre))
	
	def transmit(self, force, parameters=None, velocity=None):
		return Screw(vec3(0), force.momentum, self.center)
	
	def scheme(self, maxsize, attach_start, attach_end):
		from .primitives import ArcCentered
		
		size = settings.display['joint_size']
		radius = size/3
		sch = Scheme()
		resolution = ('div', 16)
		
		center = self.post[3].xyz
		sch.set(track=0, space=scale_solid(self.solids[0], fvec3(center), maxsize/size))
		if attach_end:
			x,y,z = dirbase(attach_end - center)
		else:
			x,y,z = mat3()
		profile = ArcCentered(Axis(center,y), center+x*radius, center-z*radius).mesh(resolution=resolution)
		emisphere = gt.revolution(2*pi, Axis(center,z), profile, resolution)
		sch.add(emisphere, shader='ghost')
		sch.add(emisphere.outlines(), shader='line')
		
		center = affineInverse(self.pre)[3].xyz
		sch.set(track=1, space=scale_solid(self.solids[1], fvec3(center), maxsize/size))
		sch.add(gt.icosphere(center, radius*0.8, resolution), shader='ghost')
		
		return sch

	
class Universal(Joint):
	dtype = np.dtype((float, 2))
	
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
	
	def direct(self, orient):
		return self.post * mat4(quat(vec3(orient, 0))) * self.pre
	
	def inverse(self, matrix, close=None):
		m = quat(affineInverse(self.post) * matrix * affineInverse(self.pre))
		return vec2(pitch(m), yaw(m))
	
	
class ConstantVelocity(Joint):
	pass
	

class Project(Joint):
	def __init__(self, solids, projection:mat4, target:mat4):
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
