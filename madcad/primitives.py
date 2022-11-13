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
from .mathutils import *
from . import settings
from . import displays
from . import mesh



def isprimitive(obj):
	''' return True if obj match the signature for primitives '''
	return hasattr(obj, 'mesh') and hasattr(obj, 'slvvars')
		
Vector = Point = vec3


class Axis(object):
	''' Mimic the behavior of a tuple, but with the primitive signature. '''
	__slots__ = ('origin', 'direction', 'interval')
	def __init__(self, origin, direction, interval=None):
		self.origin, self.direction = origin, direction
		self.interval = interval
	
	def __getitem__(self, i):
		''' behave like the axis was a tuple (origin, direction) '''
		if i==0:	return self.origin
		elif i==1:	return self.direction
		else:		raise IndexError('an axis has only 2 components')
		
	def flip(self) -> 'Axis':
		''' switch the axis direction '''
		return Axis(self.origin, -self.direction, self.interval)
	
	def offset(self, increment) -> 'Axis':
		''' move the axis origin along its direction '''
		return Axis(self.origin + self.direction*increment, self.direction, self.interval)
		
	def transform(self, transform) -> 'Axis':
		''' move the axis by the given transformation '''
		if isinstance(transform, (float,int)):		return Axis(transform*self.origin, self.direction, self.interval)
		elif isinstance(transform, vec3):			return Axis(transform+self.origin, self.direction, self.interval)
		elif isinstance(transform, (mat3, quat)):	return Axis(transform*self.origin, normalize(transform*self.direction), self.interval)
		elif isinstance(transform, (mat4)):			return Axis(transform*self.origin, normalize(mat3(transform)*self.direction), self.interval)
		raise TypeError('transform must be one of float, vec3, mat3, quat, mat4')
	
	slvvars = ('origin', 'direction')
	def slv_tangent(self, pt):
		return self.direction
		
	def __repr__(self):
		return 'Axis({}, {})'.format(self.origin, self.direction)
	
	def display(self, scene):
		return displays.AxisDisplay(scene, (self.origin, self.direction), self.interval)
			
def isaxis(obj):
	''' return True if the given object is considered to be an axis.
		An axis can be an instance of `Axis` or a tuple `(vec3, vec3)`
	'''
	return isinstance(obj, Axis) or isinstance(obj, tuple) and len(obj)==2 and isinstance(obj[0],vec3) and isinstance(obj[1],vec3)


class Segment(object):
	''' segment from a to b '''
	__slots__ = ('a', 'b')
	def __init__(self, a, b):
		self.a, self.b = a,b
		
	@property
	def direction(self):
		return normalize(self.b-self.a)
		
	@property
	def origin(self):
		return mix(self.a, self.b, 0.5)
		
	slvvars = 'a', 'b'
	def slv_tangent(self, pt):
		return self.direction
	
	def mesh(self):
		return mesh.Wire([self.a, self.b], groups=[None])
		
	def __repr__(self):
		return 'Segment({}, {})'.format(self.a, self.b)
	
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
		''' tangent to the closest point of the curve to pt '''
		c = self.center
		return normalize(cross(pt-c, cross(self.a-c, self.c-c)))
	
	slvvars = ('a', 'b', 'c')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		center = self.center
		z = normalize(cross(self.c-self.b, self.a-self.b))
		return mkarc((center, z), self.a, self.c, resolution or self.resolution)
		
	def __repr__(self):
		return 'Axis({}, {}, {})'.format(self.a, self.b, self.c)
	
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
		''' tangent to the closest point of the curve to pt '''
		return cross(normalize(pt - self.axis[0]), self.axis[1])
	
	slvvars = ('axis', 'a', 'b')
	slv_tangent = tangent
	def fit(self):
		return (dot(self.a-self.axis[0], self.axis[1]) **2,
				dot(self.b-self.axis[0], self.axis[1]) **2,
				(distance(self.a,self.axis[0]) - distance(self.b,self.axis[0])) **2,
				(length(self.axis[1])-1) **2,
				)
	
	def mesh(self, resolution=None):
		return mkarc(self.axis, self.a, self.b, resolution or self.resolution)
		
	def __repr__(self):
		return 'ArcCentered({}, {}, {})'.format(self.axis, self.a, self.b)
	
	def display(self, scene):	
		return self.mesh().display(scene)
		
class ArcTangent(object):
	''' An arc always tangent to `Segment(a,b)` and `Segment(c,b)`. The solution is unique.'''
	__slots = 'a', 'b', 'c'
	def __init__(self, a, b, c, resolution=None):
		self.a, self.b, self.c = a,b,c
		self.resolution = resolution
	
	@property
	def center(self):
		n = normalize(self.a + self.c - 2*self.b)
		return mix(	unproject(self.a-self.b, n),
					unproject(self.c-self.b, n),
					0.5)	+ self.b
	
	@property
	def radius(self):
		c = self.center
		return mix(distance(c,self.a), distance(c,self.c), 0.5)
	
	@property
	def axis(self):
		return self.center, normalize(cross(self.b-self.a, self.c-self.a))
	
	def tangent(self, pt):
		''' tangent to the closest point of the curve to pt '''
		z = cross(self.a-self.b, self.c-self.b)
		return normalize(cross(pt-self.center, z))
	
	slvvars = 'a', 'b', 'c'
	slv_tangent = tangent
	
	def fit(self):
		return (distance(self.a, self.b) - distance(self.c, self.b)) **2,
	
	def mesh(self, resolution=None):
		return mkarc(self.axis, self.a, self.c, resolution or self.resolution)
		
	def __repr__(self):
		return 'ArcTangent({}, {}, {})'.format(self.a, self.b, self.c)
	
	def display(self, scene):	
		return self.mesh().display(scene)

def mkarc(axis, start, end, resolution=None):
	center, z = axis
	center = center + dot(z, (start+end)/2-center) * z
	v = start-center
	r = length(v)
	x = v/r
	y = cross(z, x)
	angle = atan2(dot(end-center,y), dot(end-center,x)) % (2*pi)
	div = settings.curve_resolution(angle*r, angle, resolution)
	pts = [start]
	for i in range(1,div+1):
		a = angle * i/(div+1)
		pts.append(x*r*cos(a) + y*r*sin(a) + center)
	pts.append(end)
	return mesh.Wire(pts, groups=[None])

class TangentEllipsis(object):
	''' An quater of ellipsis always tangent to `Segment(a,b)` and `Segment(c,b)`. The solution is unique.
	'''
	__slots__ = ('a', 'b', 'c', 'resolution')
	def __init__(self, a,b,c, resolution=None):
		self.a, self.b, self.c = a,b,c
		self.resolution = resolution
	
	@property
	def axis(self):
		return (self.center, normalize(cross(self.a-self.b, self.c-self.b)))
		
	@property
	def center(self):
		return self.a + self.c - self.b
	
	def tangent(self, pt):
		''' tangent to the closest point of the curve to pt '''
		c,z = self.axis
		return normalize(cross(z, pt - c))
	
	slvvars = ('a', 'b', 'c')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		''' axis directions doesn't need to be normalized nor oriented '''
		origin = self.b
		x = origin - self.a
		y = origin - self.c
		div = settings.curve_resolution(distance(self.a,self.c), anglebt(x,-y), self.resolution or resolution)
		pts = [self.a]
		for i in range(div+1):
			t = pi/2 * i/(div+1)
			pts.append(x*sin(t) + y*cos(t) + origin-x-y)
		pts.append(self.c)
		return mesh.Wire(pts, groups=[None])
		
	def __repr__(self):
		return 'TangentEllipsis({}, {}, {})'.format(self.c, self.b, self.c)
	
	def display(self, scene):	
		return self.mesh().display(scene)

class Circle(object):
	''' circle centered around the axis origin, with the given radius, in an orthogonal plane to the axis direction '''
	__slots__ = ('axis', 'radius', 'alignment', 'resolution')
	def __init__(self, axis, radius, alignment=vec3(1,0,0), resolution=None):
		self.axis, self.radius = axis, radius
		self.alignment = alignment
		self.resolution = resolution
	
	@property
	def center(self):
		return self.axis[0]
	
	def fit(self):
		return (length(self.axis[1])-1) **2,
	
	def tangent(self, pt):
		''' tangent to the closest point of the curve to pt '''
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
		return mesh.Wire(pts, indices, groups=[None])
		
	def __repr__(self):
		return 'Circle({}, {})'.format(self.axis, self.radius)
	
	def display(self, scene):	
		return self.mesh().display(scene)
		
		
import numpy.core as np
def glmarray(array, dtype='f4'):
	''' create a numpy array from a list of glm vec '''
	buff = np.empty((len(array), len(array[0])), dtype=dtype)
	for i,e in enumerate(array):
		buff[i][:] = e
	return buff

		
class Interpolated(object):
	''' interpolated curve passing through the given points (3rd degree bezier spline) 

		the tangent in each point is determined by the direction between adjacent points
		the point weights is how flattened is the curve close to the point tangents
	'''
	__slots__ = 'points', 'weights', 'resolution'
	def __init__(self, points, weights=None, resolution=None):
		self.points = points
		self.weights = weights or [1] * len(self.points)
		self.resolution = resolution
		
	def mesh(self, resolution=None):
		pts = self.points
		if not pts:		return Wire()
		
		# get tangent to each point
		tas = [self.weights[i-1] * length(pts[i]-pts[i-1]) * normalize(pts[i]-pts[i-2])	
					for i in range(2,len(pts))]
		tbs = [self.weights[i-1] * length(pts[i-2]-pts[i-1]) * normalize(pts[i-2]-pts[i])	
					for i in range(2,len(pts))]
		tas.insert(0, tbs[0] - 2*project(tbs[0], pts[1]-pts[0]))
		tbs.append(tas[-1] - 2*project(tas[-1], pts[-2]-pts[-1]))
		
		# stack points to curve
		curve = []
		for i in range(len(pts)-1):
			a,b = pts[i], pts[i+1]
			ta,tb = tas[i], tbs[i]
			# tangent to the curve in its inflexion point
			mid = 1.25*(b-a) + 0.25*(tb-ta)
			# get resolution
			div = 1 + settings.curve_resolution(
							length(ta) + length(tb), 
							anglebt(-tb, mid) + anglebt(ta, mid),
							self.resolution or resolution)
			
			# append the points for this segment
			for i in range(div+1):
				curve.append(interpol2((a,ta), (b,tb), i/(div+1)))
		
		curve.append(b)
		return mesh.Wire(curve, groups=[None])
		
	def box(self):
		return boundingbox(self.points)
		
	def __repr__(self):
		return 'Interpolated({}, {})'.format(self.points, self.weights)

	def display(self, scene):
		return displays.SplineDisplay(scene, glmarray(self.points), glmarray(self.mesh().points))
			
class Softened(object):
	''' interpolated curve tangent to each segment midpoint (3rd degree bezier curve)
	
		the points weights is the weight in the determination of each midpoint
	'''
	__slots__ = 'points', 'weights', 'resolution'
	def __init__(self, points, weights=None, resolution=None):
		self.points = points
		self.weights = weights or [1] * len(self.points)
		self.resolution = resolution
		
	def mesh(self, resolution=None):
		pts = self.points
		if not pts:		return Wire()
		
		# find midpoints
		mid = [	(self.weights[i-1]*pts[i-1] + self.weights[i]*pts[i]) 
					/ (self.weights[i-1]+self.weights[i])
				for i in range(1,len(pts)) ]
		mid[0] = pts[0]
		mid[-1] = pts[-1]
		
		# stack points to curve
		curve = []
		for i in range(len(pts)-2):
			a,b = mid[i],  mid[i+1]
			ta,tb = 2*(pts[i+1]-a),  2*(pts[i+1]-b)
			# get resolution
			div = 1 + settings.curve_resolution(
						length(ta) + length(tb), 
						anglebt(ta, -tb),
						self.resolution or resolution)
			
			# append the points for this segment
			for i in range(div+1):
				curve.append(interpol2((a,ta), (b,tb), i/(div+1)))
		
		curve.append(b)
		return mesh.Wire(curve, groups=[None])
		
	def box(self):
		return boundingbox(self.points)
		
	def __repr__(self):
		return 'Softened({}, {})'.format(self.points, self.weights)

	def display(self, scene):
		return displays.SplineDisplay(scene, glmarray(self.points), glmarray(self.mesh().points))
	
