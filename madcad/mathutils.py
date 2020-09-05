# This file is part of pymadcad,  distributed under license LGPL v3

''' Regroupement des fonctions et classes math de pymadcad '''

import glm
from glm import *
del version, license
from math import pi, inf, atan2
from copy import deepcopy
max = __builtins__['max']
min = __builtins__['min']
any = __builtins__['any']
all = __builtins__['all']


vec2 = dvec2
mat2 = dmat2
vec3 = dvec3
mat3 = dmat3
vec4 = dvec4
mat4 = dmat4
quat = dquat

NUMPREC = 1e-13
COMPREC = 1-NUMPREC

# numerical precision of floats used (float32 here, so 7 decimals, so 1e-6 when exponent is 1)
#NUMPREC = 1e-6
#COMPREC = 1-NUMPREC

def norminf(x):
	''' norm L infinite  ie.  max(abs(x), abs(y), abs(z)) '''
	return max(glm.abs(x))

def norm1(x):
	''' norm L1  ie.  abs(x)+abs(y)+abs(z) '''
	return sum(glm.abs(x))

norm2 = length

def anglebt(x,y) -> float:
	''' angle between two vectors '''
	n = length(x)*length(y)
	return acos(min(1,max(-1, dot(x,y)/n)))	if n else 0
	
def arclength(p1, p2, n1, n2):
	''' length of an arc between p1 and p2, normal to the given vectors in respective points '''
	c = max(0, dot(n1,n2))
	if abs(c-1) < NUMPREC:	return 0
	v = p1-p2
	return sqrt(dot(v,v) / (2-2*c)) * acos(c)

def project(vec, dir) -> vec3:
	''' component of `vec` along `dir`, equivalent to :code:`dot(vec, dir) * dir` '''
	return dot(vec, dir) * dir
	
def noproject(x,dir) -> vec3:	
	''' components of `vec` not along `dir`, equivalent to :code:`x - project(x,dir)` '''
	return x - project(x,dir)

def unproject(vec, dir) -> vec3:
	''' return the vector in the given direction as if `vec` was its projection on it, equivalent to :code:`dot(vec,vec) / dot(vec,dir) * dir` '''
	return dot(vec,vec) / dot(vec,dir) * dir

def perpdot(a:vec2, b:vec2) -> float:
	return -a[1]*b[0] + a[0]*b[1]

def perp(v:vec2) -> vec2:
	''' perpendicular vector to the given vector '''
	return vec2(-v[1], v[0])
	
def dirbase(dir, align=vec3(1,0,0)):
	''' returns a base using the given direction as z axis (and the nearer vector to align as x) '''
	x = align - project(align, dir)
	if not length(x) > NUMPREC:
		align = vec3(align[2],-align[0],align[1])
		x = align - project(align, dir)
	if not length(x) > NUMPREC:
		return vec3(1,0,0),vec3(0,1,0),vec3(0,0,1)
	x = normalize(x)
	y = cross(dir, x)
	return x,y,dir

def scaledir(dir, factor=None) -> mat3:
	''' return a mat3 scaling in the given direction, with the given factor (1 means original scale) 
		if factor is None, the length of dir is used, but it can leads to precision loss on direction when too small.
	'''
	if factor is None:
		factor = length(dir)
		dir /= factor
	return mat3(1) + (factor-1)*mat3(dir[0]*dir, dir[1]*dir, dir[2]*dir)

def transform(*args) -> mat4:
	''' create an affine transformation matrix.
		
		supported inputs:
		
		*	mat4
		*	vec3                                    - translation only
		*	quat, mat3, mat4                        - rotation only
		*	(vec3,vec3), (vec3,mat3), (vec3,quat)   - translation and rotation
		*	(vec3,vec3,vec3)                        - base of vectors for rotation
		*	(vec3,vec3,vec3,vec3)                   - translation and base of vectors for rotation
	'''
	if len(args) == 1 and isinstance(args[0], (tuple,list)):
		args = args[0]
	if len(args) == 1:
		if isinstance(args[0], mat4):	return args[0]
		elif isinstance(args[0], mat3):	return mat4(args[0])
		elif isinstance(args[0], quat):	return mat4_cast(args[0])
		elif isinstance(args[0], vec3):	return translate(mat4(1), args[0])
	elif len(args) == 2:
		if isinstance(args[0], vec3):
			if   isinstance(args[1], mat3):		m = mat4(args[1])
			elif isinstance(args[1], quat):		m = mat4_cast(args[1])
			elif isinstance(args[1], vec3):		m = mat4_cast(quat(args[1]))
			m[3] = vec4(args[0], 1)
			return m
	elif isinstance(args[0], vec3) and len(args) == 3:			
		return mat4(mat3(*args))
	elif isinstance(args[0], vec3) and len(args) == 4:
		m = mat4(mat3(*args[1:]))
		m[3] = vec4(args[0], 1)
		return m
	
	raise TypeError('a transformation must be a  mat3, mat4, quat, (O,mat3), (O,quat), (0,x,y,z)')


def interpol1(a, b, x):
	''' 1st order polynomial interpolation '''
	return (1-x)*a + x*b

def interpol2(a, b, x):
	''' 3rd order polynomial interpolation 
		a and b are iterable of successive derivatives of a[0] and b[0]
	'''
	return (	2*x*(1-x)  * interpol1(a[0],b[0],x)		# linear component
			+	x**2       * (b[0] + (1-x)*b[1])		# tangent
			+	(1-x)**2   * (a[0] + x*a[1])	# tangent
			)

spline = interpol2

def interpol2tri(pts, ptangents, a,b):
	''' cubic interpolation like interpol2, but interpolates over a triangle (2d parameter space) '''
	A,B,C = pts
	ta,tb,tc = ptangents
	c = 1-a-b
	P = a*A + b*B + c*C +  a*b*c * (ta[0] + ta[1] + tb[0] + tb[1] + tc[0] + tc[1])
	return (	0
			+	a**2 * (A + b*ta[0] + c*ta[1])
			+	b**2 * (B + c*tb[0] + a*tb[1])
			+	c**2 * (C + a*tc[0] + b*tc[1])
			+	2*(b*c + c*a + a*b) * P
			)
		
# distances:

distance_pp = distance

def distance_pa(pt, axis):
	''' point - axis distance '''
	return length(noproject(pt-axis[0], axis[1]))

def distance_pe(pt, edge):
	''' point - edge distance '''
	dir = edge[1]-edge[0]
	l = length(dir)
	x = dot(pt-edge[0], dir)/l**2
	if   x < 0:	return distance(pt,edge[0])
	elif x > 1:	return distance(pt,edge[1])
	else:
		return length(noproject(pt-edge[0], dir/l))

def distance_aa(a1, a2):
	''' axis - axis distance '''
	return dot(a1[0]-a2[0], normalize(cross(a1[1], a2[1])))

def distance_ae(axis, edge):
	''' axis - edge distance '''
	x = axis[1]
	z = normalize(cross(x, edge[1]-edge[0]))
	y = cross(z,x)
	s1 = dot(edge[0]-axis[0], y)
	s2 = dot(edge[1]-axis[0], y)
	if s1*s2 < 0:
		return dot(edge[0]-axis[1], z)
	elif abs(s1) < abs(s2):
		return distance_pa(edge[0], axis)
	else:
		return distance_pa(edge[1], axis)


#-- algorithmic functions ---------

def dichotomy_index(l, index, key=lambda x:x):
	''' use dichotomy to get the index of `index` in a list sorted in ascending order
		key can be used to specify a function that gives numerical value for list elements
	'''
	start,end = 0, len(l)
	while start < end:
		mid = (start+end)//2
		val = key(l[mid])
		if val < index:		start =	mid+1
		elif val > index:	end =	mid
		else:	return mid
	return start

def find(iterator, predicate):
	for e in iterator:
		if predicate(e):	return e

class Box:
	__slots__ = ('min', 'max')
	def __init__(self, min=None, max=None, center=vec3(0), width=vec3(-inf)):
		if min and max:			self.min, self.max = min, max
		else:					self.min, self.max = center-width/2, center+width/2
	
	@property
	def center(self):
		''' mid coordinates of the box '''
		return (self.min + self.max) /2
	@property
	def width(self):
		''' diagonal vector of the box '''
		return self.max - self.min
	
	def isvalid(self):
		''' return True if the box defines a valid space (min coordinates <= max coordinates) '''
		return any(self.min <= self.max)
	def isempty(self):
		''' return True if the box contains a non null volume '''
		return any(self.min >= self.max)
	
	def __add__(self, other):
		if isinstance(other, vec3):		return Box(self.min + other, self.max + other)
		elif isinstance(other, Box):	return Box(other.min, other.max)
		else:
			return NotImplemented
	def __iadd__(self, other):
		if isinstance(other, vec3):		
			self.min += other
			self.max += other
		elif isinstance(other, Box):	
			self.min += other.min
			self.max += other.max
		else:
			return NotImplemented
	def __sub__(self, other):
		if isinstance(other, vec3):		return Box(self.min - other, self.max - other)
		elif isinstance(other, Box):	return Box(other.min, other.max)
		else:
			return NotImplemented
	def __or__(self, other):	return deepcopy(self).union(other)
	def __and__(self, other):	return deepcopy(self).intersection(other)
	def union(self, other):
		if isinstance(other, vec3):
			self.min = glm.min(self.min, other)
			self.max = glm.max(self.max, other)
		elif isinstance(other, Box):
			self.min = glm.min(self.min, other.min)
			self.max = glm.max(self.max, other.max)
		else:
			return NotImplemented
		return self
	def intersection(self, other):
		if isinstance(other, vec3):
			self.min = glm.max(self.min, other)
			self.max = glm.min(self.max, other)
		elif isinstance(other, Box):
			self.min = glm.max(self.min, other.min)
			self.max = glm.min(self.max, other.max)
		else:
			return NotImplemented
		for i in range(3):
			if self.min[i] > self.max[i]:
				self.min[i] = self.max[i] = (self.min[i]+self.max[i])/2
				break
		return self
	def __bool__(self):
		for i in range(3):
			if self.min[i] >= self.max[i]:	return False
		return True
	def __repr__(self):
		return '{}({}, {})'.format(self.__class__.__name__, repr(self.min), repr(self.max))

def boundingbox(*args, ignore=True) -> Box:
	''' return a bounding box for the objects passed
		will search recursively in sub iterables
		if ignore is False, a TypeError is raised when one of the passed objects doesn't contribute to the bounding box
	'''
	box = Box(vec3(inf), vec3(-inf))
	for obj in args:
		if hasattr(obj, 'box'):
			part = obj.box()
		elif isinstance(obj, vec3):
			part = obj
		elif isinstance(obj, tuple) and isinstance(obj[0], vec3):
			part = obj[0]
		elif hasattr(obj, '__iter__'):
			for e in obj:
				box.union(boundingbox(e))
			continue
		else:
			if not ignore:	raise TypeError('unable to get a boundingbox from', obj)
			continue
		box.union(part)
	return box

