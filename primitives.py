from math import sqrt
from mathutils import vec3, mat3, normalize, anglebt, project, cos, sin, atan2, pi, length, cross, vec2, mat2, determinant, inverse, dot, atan, acos, dirbase
import settings

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

		
Vector = Point = vec3
#class Vector(vec3):	pass	
#class Point(vec3): pass


class Axis(object):
	__slots__ = ('origin', 'dir', 'interval')
	def __init__(self, origin, dir, interval=None):
		self.origin, self.dir = origin, dir
		self.interval = interval
	
	def direction(self):
		return self.dir
	
	slv_vars = ('origin', 'dir')
	def slv_tangent(self, pt):
		return self.dir

class Segment(object):
	__slots__ = ('a', 'b')
	def __init__(self, a, b):
		self.a, self.b = a,b
	def direction(self):
		return normalize(self.b-self.a)
		
	slv_vars = 'a', 'b'
	def slv_tangent(self, pt):
		return self.direction()
	
	def mesh(self):
		return [self.a, self.b], 'flat'

class ArcThrough(object):	
	__slots__ = ('a', 'b', 'c', 'resolution')
	
	def __init__(self, a,b,c, resolution=None):
		self.a, self.b, self.c = a,b,c
		self.resolution = resolution
	
	def center(self):
		a,c = self.a-self.b, self.c-self.b
		l1,l2 = length(a),length(c)
		t = dot(a,c)/(l1*l2)
		h = 0.5 * (l1/l2-t) * 1/sqrt(1-t**2)
		n = normalize(cross(self.a-self.b, self.c-self.b))
		return (self.c+self.b)/2 + cross(n, self.b-self.c) * h
		
	def radius(self):
		return (self.a-self.center()).norm()
	
	def axis(self):
		return (self.center(), normalize(cross(self.a-self.b, self.c-self.b)))
	
	def tangent(self, pt):
		c = self.center()
		return normalize(cross(pt-c, self.a-c))
	
	slv_vars = ('a', 'b', 'c')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		center = self.center()
		z = normalize(cross(self.c-self.b, self.a-self.b))
		return mkarc((center, z), self.a, self.c, resolution or self.resolution)

class ArcCentered(object):
	__slots__ = ('axis', 'a', 'b', 'resolution')
	def __init__(self, axis, a, b, resolution=None):
		self.axis = axis
		self.a, self.b = a,b
		self.resolution = resolution
	
	def radius(self):
		return distance(self.center, self.a)
	
	def tangent(self, pt):
		return normalize(pt - self.axis[0])
	
	slv_vars = ('axis', 'a', 'b')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		return mkarc(self.axis, self.a, self.b, resolution or self.resolution)

def mkarc(axis, start, end, resolution=None):
	center, z = axis
	r = length(start-center)
	x = normalize(start-center)
	y = cross(axis[1], x)
	angle = atan2(dot(end-center,y), dot(end-center,x)) % (2*pi)
	print('angle', angle)
	div = settings.curve_resolution(angle*r, angle, resolution)
	pts = []
	for i in range(div+1):
		a = angle * i/div
		pts.append(x*r*cos(a) + y*r*sin(a) + center)
	return pts, 'arc'
	

class TangentEllipsis(object):
	__slots__ = ('a', 'b', 'c', 'resolution')
	def __init__(self, a,b,c, resolution=None):
		self.a, self.b, self.c = a,b,c
		self.resolution = resolution
	
	def axis(self):
		return (self.a+self.c - self.b, normalize(cross(self.a-self.b, self.c-self.b)))
	
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
		return pts, 'ellipsis'

class Circle(object):
	__slots__ = ('axis', 'radius', 'alignment', 'resolution')
	def __init__(self, axis, radius, alignment=vec3(1,0,0), resolution=None):
		self.axis, self.radius = axis, radius
		self.alignment = alignment
		self.resolution = resolution
	
	def center(self, pt):
		return self.axis[0]
	
	def tangent(self, pt):
		return normalize(cross(pt-self.axis[0], self.axis[1]))
	
	slv_vars = ('axis', 'radius')
	slv_tangent = tangent
	
	def mesh(self, resolution=None):
		center = self.axis[0]
		x,y,z = dirbase(self.axis[1], self.alignment)
		angle = 2*pi
		r = self.radius
		div = settings.curve_resolution(angle*r, angle, self.resolution or resolution)
		pts = []
		for i in range(div+1):
			a = angle * i/div
			pts.append(x*r*cos(a) + y*r*sin(a) + center)
		return pts, 'arc'


if __name__ == '__main__':
	a = Arc(vec3(-1,0,0), vec3(0,1,0), vec3(1,0,0))
	print(a.center())
