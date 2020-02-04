from math import sqrt
from mathutils import vec3, normalize, anglebt, project, cos, sin, pi, length, cross, vec2, mat2, determinant, inverse, dot, atan, acos
import settings

class Primitive(object):
	__slots__ = ('annotations',)
	def __init__(self):
		self.annotations = {}
	def display(self):	
		return None
	def annotate(self, key, value):	
		self.annotations[key] = value
	
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
		n = normalize(cross(self.a-self.b, self.c-self.b))
		# (pi,ri) est une mediatrice
		p1, p2 = (self.b+self.a)/2, (self.b+self.c)/2
		r1, r2 = cross(n,self.b-self.a), cross(n,self.c-self.b)
		return axisintersection((p1,r1), (p2,r2))
	
	def center(self):
		a,c = self.a-self.b, self.c-self.b
		l1,l2 = length(a),length(c)
		t = dot(a,c)
		d = (l1/l2-t) * 1/sin(acos(t))
		n = normalize(cross(self.a-self.b, self.c-self.b))
		return (self.c+self.b)/2 + cross(n, self.b-self.c) * d/2
		
	
	def mesh(self):
		center = self.center()
		angle = anglebt(self.a-center, self.c-center)
		r = length(self.a-center)
		div = settings.curve_resolution(angle, 2*angle*r)
		x = normalize(self.a-center)
		y = self.b-center
		y = normalize(y - project(y,x))
		pts = []
		for i in range(div+1):
			a = angle * i/div
			pts.append(x*r*cos(a) + y*r*sin(a) + center)
		return pts, 'arc'

def axisintersection(a1, a2):
	p1,r1 = a1
	p2,r2 = a2
	# pseudo inverse du systeme: choix de la reduction a opérer
	m1 = mat2(r1[0], r2[0], r1[1], r2[1])
	m2 = mat2(r1[0], r2[0], r1[2], r2[2])
	m3 = mat2(r1[1], r2[1], r1[2], r2[2])
	dm1 = abs(determinant(m1))
	dm2 = abs(determinant(m2))
	dm3 = abs(determinant(m3))
	if dm1 >= dm2 and dm1 >= dm3:		x1,x2 = inverse(m1)*vec2(p1-p2)
	elif dm2 >= dm3:					x1,x2 = inverse(m2)*vec2(p1[0]-p2[0], p1[2]-p2[2])
	else:								x1,x2 = inverse(m3)*vec2(p1[1]-p2[1], p1[2]-p2[2])
	return p1 - x1*r1


if __name__ == '__main__':
	a = Arc(vec3(-1,0,0), vec3(0,1,0), vec3(1,0,0))
	print(a.center())
