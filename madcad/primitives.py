# This file is part of pymadcad,  distributed under license LGPL v3

''' Definition of 3D primitive objects

	Primitives are parametrized objects, that can be baked into a mesh/web/wire object. A primitive object must have the following signature:
	
		class SomePrimitive:
			# method baking the primitive in some general-purpose 3D object
			def mesh(self) -> Mesh/Web/Wire:
				...
			
			# for the solver
			# primitive attributes the solver has to consider as variables or variable container
			slvvars = 'fields', 'for', 'solver', 'variables'
			# optional method constraining the primitive parameters (to keep points on a circle for instance)
			def fit(self) -> err**2 as float:
				...


	Curve resolution
	----------------

	Some primitive types are curves, the discretisation is important for visual as well as for result quality (remember that even if something looks like a perfect curves, it's still polygons).
	The resolution (subdivision) of curve is done following the following cirterions present in the 'settings' module

	specification priority order:
		
		1. optional argument `resolution` passed to `primitive.mesh()` or to `web()` or `wire()`
		2. optional attribute `resolution` of the primitive object
		3. value of `settings.primitives['curve_resolution']` at bake time.

	specification format:
		
		('fixed', 16)   # fixed amount of 16 subdivisions
		('rad', 0.6)    # max polygon angle is 0.6 rad
		('radm', 0.6)
		('radm2', 0.6)

'''

from math import sqrt
from .mathutils import vec3, mat3, normalize, anglebt, project, noproject, cos, sin, atan2, pi, length, distance, cross, vec2, mat2, determinant, inverse, dot, atan, acos, dirbase
from . import settings
from . import displays
from .mesh import Wire

class Primitive(object):
	__slots__ = ('annotations',)
	def __init__(self):
		self.annotations = {}
	def display(self):	
		return None
	def annotate(self, key, value):	
		self.annotations[key] = value
	def transform(self, trans):
		return self
	
	def mesh(self, resolution=None):
		return NotImplemented

def isprimitive(obj):
	''' return True if obj match the signature for primitives '''
	return hasattr(obj, 'mesh') and hasattr(obj, 'slvvars')
		
Vector = Point = vec3
#class Vector(vec3):	pass	
#class Point(vec3): pass


class Axis(object):
	''' Mimic the behavior of a tuple, but with the primitive signature. '''
	__slots__ = ('origin', 'dir', 'interval')
	def __init__(self, origin, direction, interval=None):
		self.origin, self.direction = origin, direction
		self.interval = interval
	def __getitem__(self, i):
		if i==0:	return self.origin
		elif i==1:	return self.direction
		else:		raise IndexError('an axis has only 2 components')
	
	slvvars = ('origin', 'dir')
	def slv_tangent(self, pt):
		return self.direction
	
	def display(self, scene):
		yield displays.AxisDisplay(scene, (self.origin, self.direction), self.interval)

class Segment(object):
	''' segment from a to b '''
	__slots__ = ('a', 'b')
	def __init__(self, a, b):
		self.a, self.b = a,b
		
	@property
	def direction(self):
		return normalize(self.b-self.a)
		
	slvvars = 'a', 'b'
	def slv_tangent(self, pt):
		return self.direction
	
	def mesh(self):
		return Wire([self.a, self.b], group='flat')
	def display(self, scene):
		return self.mesh().display(scene)

class ArcThrough(object):	
	''' arc from a to c, passing through b '''
	__slots__ = ('a', 'b', 'c', 'resolution')
	
	def __init__(self, a,b,c, resolution=None):
		self.a, self.b, self.c = a,b,c
		self.resolution = resolution
	
	@property
	def center(self):
		a,c = self.a-self.b, self.c-self.b
		l1,l2 = length(a),length(c)
		t = dot(a,c)/(l1*l2)
		h = 0.5 * (l1/l2-t) * 1/sqrt(1-t**2)
		n = normalize(cross(self.a-self.b, self.c-self.b))
		return (self.c+self.b)/2 + cross(n, self.b-self.c) * h
	
	@property
	def radius(self):
		return length(self.a-self.center)
	
	@property
	def axis(self):
		return (self.center, normalize(cross(self.a-self.b, self.c-self.b)))
	
	def tangent(self, pt):
		c = self.center
		return normalize(cross(pt-c, cross(self.a-c, self.c-c)))
	
	slvvars = ('a', 'b', 'c')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		center = self.center
		z = normalize(cross(self.c-self.b, self.a-self.b))
		return mkarc((center, z), self.a, self.c, resolution or self.resolution)
	def display(self, scene):
		return self.mesh().display(scene)

class ArcCentered(object):
	''' arc from a to b, centered around the origin of the axis.
	
		An axis is requested instead of a point (that would be more intuitive), to solve the problem when a,b, center are aligned
	'''
	__slots__ = ('axis', 'a', 'b', 'resolution')
	def __init__(self, axis, a, b, resolution=None):
		self.axis = axis
		self.a, self.b = a, b
		self.resolution = resolution
	
	@property
	def center(self):
		return self.axis[0]
	
	@property
	def radius(self):
		return (distance(self.axis[0], self.a) + distance(self.axis[0], self.b)) /2
	
	def tangent(self, pt):
		return cross(normalize(pt - self.axis[0]), self.axis[1])
	
	slvvars = ('axis', 'a', 'b')
	slv_tangent = tangent
	def fit(self):
		return (	dot(self.a-self.axis[0], self.axis[1]) **2 
				+	dot(self.b-self.axis[0], self.axis[1]) **2
				+   (distance(self.a,self.axis[0]) - distance(self.b,self.axis[0])) **2
				+	(length(self.axis[1])-1) **2
				)
	
	def mesh(self, resolution=None):
		return mkarc(self.axis, self.a, self.b, resolution or self.resolution)
	def display(self, scene):
		return self.mesh().display(scene)

def mkarc(axis, start, end, resolution=None):
	center, z = axis
	v = noproject(start-center, z)
	r = length(v)
	x = v/r
	y = cross(z, x)
	angle = atan2(dot(end-center,y), dot(end-center,x)) % (2*pi)
	div = settings.curve_resolution(angle*r, angle, resolution) +2
	pts = []
	for i in range(div):
		a = angle * i/(div-1)
		pts.append(x*r*cos(a) + y*r*sin(a) + center)
	return Wire(pts, group='arc')
	

class TangentEllipsis(object):
	__slots__ = ('a', 'b', 'c', 'resolution')
	def __init__(self, a,b,c, resolution=None):
		self.a, self.b, self.c = a,b,c
		self.resolution = resolution
	
	def axis(self):
		return (self.a+self.c - self.b, normalize(cross(self.a-self.b, self.c-self.b)))
	
	#def tangent(self, pt):
		
	
	slvvars = ('a', 'b', 'c')
	
	def mesh(self, resolution=None):
		''' axis directions doesn't need to be normalized nor oriented '''
		origin = self.b
		x = self.a - origin
		y = self.b - origin
		angleapprox = acos(dot(x,-y))
		div = settings.curve_resolution(distance(self.a,self.b), angleapprox, self.resolution or resolution)
		pts = []
		for i in range(div+1):
			u = pi/2 * i/div
			pts.append(x*a*(1-cos(u)) + y*b*(1-sin(u)) + origin)
		return Wire(pts, group='ellipsis')
	def display(self, scene):
		return self.mesh().display(scene)

class Circle(object):
	''' circle centered around the axis origin, with the given radius, in an orthogonal plane to the axis direction '''
	__slots__ = ('axis', 'radius', 'alignment', 'resolution')
	def __init__(self, axis, radius, alignment=vec3(1,0,0), resolution=None):
		self.axis, self.radius = axis, radius
		self.alignment = alignment
		self.resolution = resolution
	
	def center(self, pt):
		return self.axis[0]
	
	def fit(self):
		return (length(self.axis[1])-1) **2
	
	def tangent(self, pt):
		return normalize(cross(pt-self.axis[0], self.axis[1]))
	
	slvvars = ('axis', 'radius')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		center = self.axis[0]
		x,y,z = dirbase(self.axis[1], self.alignment)
		angle = 2*pi
		r = self.radius
		div = settings.curve_resolution(angle*r, angle, self.resolution or resolution)
		pts = []
		for i in range(div):
			a = angle * i/div
			pts.append(x*r*cos(a) + y*r*sin(a) + center)
		indices = list(range(div))
		indices.append(0)
		return Wire(pts, indices, group='arc')
	def display(self, scene):
		return self.mesh().display(scene)
