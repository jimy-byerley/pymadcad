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

class Segment(object):
	__slots__ = ('a', 'b')
	def __init__(self, a, b):
		self.a, self.b = a,b
	def direction(self):
		return normalize(self.b-self.a)
	
	def mesh(self):
		return [self.a, self.b], 'flat'

class Arc(object):	
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
	
	def mesh(self, resolution=None):
		center = self.center()
		r = length(self.a-center)
		x = normalize(self.a-center)
		y = self.b-center
		y = normalize(y - project(y,x))
		c = self.c-center
		angle = atan2(dot(c,y), dot(c,x)) % (2*pi)
		div = settings.curve_resolution(angle*r, angle, self.resolution or resolution)
		pts = [self.a]
		for i in range(1,div):
			a = angle * i/div
			pts.append(x*r*cos(a) + y*r*sin(a) + center)
		pts.append(self.c)
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
