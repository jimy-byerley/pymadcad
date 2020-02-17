from math import sqrt
from mathutils import vec3, mat3, normalize, anglebt, project, cos, sin, pi, length, cross, vec2, mat2, determinant, inverse, dot, atan, acos
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
	#__slots__ = ('a', 'b', 'center')
	#def __init__(self, c, a, b):
		#self.center = c
		#self.a, self.b = a,b
	#def radius(self):
		#return (self.a-self.center).norm()
	
	__slots__ = ('a', 'b', 'c')
	def __init__(self, a,b,c):
		self.a, self.b, self.c = a,b,c
	
	def center(self):
		a,c = self.a-self.b, self.c-self.b
		l1,l2 = length(a),length(c)
		t = dot(a,c)/(l1*l2)
		h = 0.5 * (l1/l2-t) * 1/sqrt(1-t**2)
		n = normalize(cross(self.a-self.b, self.c-self.b))
		return (self.c+self.b)/2 + cross(n, self.b-self.c) * h
	
	def mesh(self, resolution=None):
		center = self.center()
		angle = anglebt(self.a-center, self.c-center)
		r = length(self.a-center)
		div = settings.curve_resolution(angle, angle*r, resolution)
		x = normalize(self.a-center)
		y = self.b-center
		y = normalize(y - project(y,x))
		pts = [self.a]
		for i in range(1,div):
			a = angle * i/div
			pts.append(x*r*cos(a) + y*r*sin(a) + center)
		pts.append(self.c)
		return pts, 'arc'


def tangentellipsis(axis1, axis2, resolution=None):
	''' axis directions doesn't need to be normalized nor oriented '''
	x,y = -axis1[1], -axis2[1]
	a,b,_ = inverse(mat3(x,y,cross(x,y))) * (axis2[0]-axis1[0])
	a = -a
	angleapprox = acos(dot(x,-y))
	origin = axis1[0] - a*x
	div = settings.curve_resolution(angleapprox*min(a,b), angleapprox, resolution)
	pts = []
	for i in range(div+1):
		u = pi/2 * i/div
		pts.append(x*a*(1-cos(u)) + y*b*(1-sin(u)) + origin)
	return pts

def dirbase(dir, align=vec3(1,0,0)):
	x = normalize(align - project(align, self.axis[1]))
	if length(x) < NUMPREC:		
		align = vec3(align[1],-align[0],align[2])
		x = normalize(align - project(align, self.axis[1]))
	y = normalize(cross(self.axis[1], x))
	return x,y,dir

class Circle:
	def __init__(self, axis, radius, alignment=vec3(1,0,0)):
		self.axis, self.radius = axis, radius
		self.alignment = alignment
	def mesh(self, resolution=None):
		center = self.axis[0]
		x,y,z = dirbase(self.axis[1], self.alignment)
		angle = 2*pi
		div = settings.curve_resolution(angle*radius, angle, resolution)
		pts = []
		for i in range(div+1):
			a = angle * i/div
			pts.append(x*r*cos(a) + y*r*sin(a) + center)
		return pts, 'arc'


if __name__ == '__main__':
	a = Arc(vec3(-1,0,0), vec3(0,1,0), vec3(1,0,0))
	print(a.center())
