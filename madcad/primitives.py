# This file is part of pymadcad,  distributed under license LGPL v3

''' Definition of 3D primitive objects

	Primitives are parameterized objects, that can be baked into a mesh/web/wire object. A primitive object must have the following signature:
	
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

	Specification priority order:
		
		1. Optional argument `resolution` passed to `primitive.mesh()` or to `web()` or `wire()`
		2. Optional attribute `resolution` of the primitive object
		3. Value of `settings.resolution` at bake time.

	Specification format:
		
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

__all__ = [
	'Axis', 'Vector', 'Point', 'isaxis',
	'Segment', 'ArcThrough', 'ArcCentered', 'ArcTangent', 'TangentEllipsis', 'Circle', 'Ellipsis', 
	'Interpolated', 'Softened',
	] 


def isprimitive(obj):
	''' Return True if obj match the signature for primitives '''
	return hasattr(obj, 'mesh') and hasattr(obj, 'slvvars')


class Segment(object):
	''' Segment from a to b '''
	__slots__ = ('a', 'b')
	def __init__(self, a, b):
		self.a, self.b = a,b
		
	def __call__(self, t):
		return mix(self.a, self.b, t)
		
	@property
	def direction(self):
		return normalize(self.b-self.a)
		
	@property
	def origin(self):
		return mix(self.a, self.b, 0.5)
		
	slvvars = 'a', 'b'
	def slv_tangent(self, pt):
		return self.direction
	
	def mesh(self, resolution=None):
		return mesh.Wire([self.a, self.b], groups=[None])
		
	def __repr__(self):
		return 'Segment({}, {})'.format(self.a, self.b)
	
	def display(self, scene):	
		return self.mesh().display(scene)

class ArcThrough(object):	
	''' Arc from a to c, passing through b '''
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
		''' Tangent to the closest point of the curve to pt '''
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
	''' Arc from a to b, centered around the origin of the axis.
	
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
		''' Tangent to the closest point of the curve to pt '''
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
		''' Tangent to the closest point of the curve to pt '''
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
		
	def __call__(self, t):
		x = 0.5*pi*t
		return (  (self.b - self.a)*sin(x) 
		        + (self.b - self.c)*cos(x)
		        + self.a + self.c - 2*self.b
		        )
	
	@property
	def axis(self):
		return (self.center, normalize(cross(self.a-self.b, self.c-self.b)))
		
	@property
	def center(self):
		return self.a + self.c - self.b
	
	def tangent(self, pt):
		''' Tangent to the closest point of the curve to pt '''
		c,z = self.axis
		return normalize(cross(z, pt - c))
	
	slvvars = ('a', 'b', 'c')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		''' Axis directions doesn't need to be normalized nor oriented '''
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
	''' Circle centered around the axis origin, with the given radius, in an orthogonal plane to the axis direction '''
	__slots__ = ('axis', 'radius', 'alignment', 'resolution')
	def __init__(self, axis: Axis, radius: float, alignment=vec3(1,0,0), resolution=None):
		self.axis, self.radius = axis, radius
		self.alignment = alignment
		self.resolution = resolution
	
	@property
	def center(self):
		return self.axis[0]
	
	def fit(self):
		return (length(self.axis[1])-1) **2,
	
	def tangent(self, pt):
		''' Tangent to the closest point of the curve to pt '''
		return normalize(cross(pt-self.axis[0], self.axis[1]))
	
	slvvars = ('axis', 'radius')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		x,y,z = dirbase(self.axis[1], self.alignment)
		angle = 2*pi
		div = settings.curve_resolution(angle*self.radius, angle, self.resolution or resolution)
		return mesh.Wire(typedlist(
			self.center + self.radius * (x*cos(t) + y*sin(t))
			for t in linrange(0, angle, div=div, end=False)
			)).close()
		
	def __repr__(self):
		return 'Circle({}, {})'.format(self.axis, self.radius)
	
	def display(self, scene):	
		return self.mesh().display(scene)

class Ellipsis(object):
	''' Ellipsis centered around the given point, with the given major and minor semi axis '''
	__slots__ = ('center', 'minor', 'major', 'resolution')
	def __init__(self, center: vec3, minor: vec3, major: vec3, resolution=None):
		self.center = center
		self.minor = minor
		self.major = major
		self.resolution = resolution
		
	@property
	def axis(self):
		''' the ellipsis axis, deduces from its major and minor semi axis '''
		return Axis(self.center, normalize(cross(self.minor, self.major)))
		
	slvvars = ('center', 'minor', 'major')
		
	def mesh(self, resolution=None):
		angle = 2*pi
		radius = sqrt(max(length2(self.minor), length2(self.major)))
		div = settings.curve_resolution(angle*radius, angle, self.resolution or resolution)
		return mesh.Wire(typedlist(
			self.center + self.minor * cos(t) + self.major * sin(t)
			for t in linrange(0, angle, div=div, end=False)
			)).close()
			
	def __repr__(self):
		return 'Ellipsis({}, {}, {})'.format(self.center, self.minor, self.major)
		
	def display(self, scene):
		return self.mesh().display(scene)
		
import numpy.core as np
def glmarray(array, dtype='f4'):
	''' Create a numpy array from a list of glm vec '''
	buff = np.empty((len(array), len(array[0])), dtype=dtype)
	for i,e in enumerate(array):
		buff[i][:] = e
	return buff

		
class Interpolated(object):
	''' Interpolated curve passing through the given points (3rd degree bezier spline) 

		The tangent in each point is determined by the direction between adjacent points
		The point weights is how flattened is the curve close to the point tangents
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
	''' Interpolated curve tangent to each segment midpoint (3rd degree bezier curve)
	
		The points weights is the weight in the determination of each midpoint
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
	
