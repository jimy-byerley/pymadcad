# This file is part of pymadcad,  distributed under license LGPL v3

''' Regroupement des fonctions et classes math de pymadcad '''

import glm
from glm import *
del version, license
from math import pi, inf, nan, atan2
from copy import deepcopy
max = __builtins__['max']
min = __builtins__['min']
any = __builtins__['any']
all = __builtins__['all']
round = __builtins__['round']


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

def isfinite(x):
	return not (glm.any(isinf(x)) or glm.any(isnan(x)))

def norminf(x):
	''' norm L infinite  ie.  `max(abs(x), abs(y), abs(z))` '''
	return max(glm.abs(x))
# norm L1  ie.  `abs(x) + abs(y) + abs(z)`
norm1 = l1Norm
# norm L2  ie.  `sqrt(x**2 + y**2 + z**2)`   the usual distance also known as euclidian distance or manhattan distance
norm2 = length

def anglebt(x,y) -> float:
	''' angle between two vectors 
	
		the result is not sensitive to the lengths of x and y
	'''
	n = length(x)*length(y)
	return acos(min(1,max(-1, dot(x,y)/n)))	if n else 0
	
def arclength(p1, p2, n1, n2):
	''' length of an arc between p1 and p2, normal to the given vectors in respective points '''
	c = max(0, dot(n1,n2))
	if abs(c-1) < NUMPREC:	return 0
	v = p1-p2
	return sqrt(dot(v,v) / (2-2*c)) * acos(c)

def project(vec, dir) -> vec3:
	''' component of `vec` along `dir`, equivalent to :code:`dot(vec,dir) / dot(dir,dir) * dir` 
	
		the result is not sensitive to the length of `dir`
	'''
	try:	return dot(vec,dir) / dot(dir,dir) * dir
	except ZeroDivisionError:	return vec3(nan)
	
def noproject(vec, dir) -> vec3:
	''' components of `vec` not along `dir`, equivalent to :code:`vec - project(vec,dir)` 
	
		the result is not sensitive to the length of `dir`
	'''
	return vec - project(vec,dir)

def unproject(vec, dir) -> vec3:
	''' return the vector in the given direction as if `vec` was its projection on it, equivalent to :code:`dot(vec,vec) / dot(vec,dir) * dir` 
	
		the result is not sensitive to the length of `dir`
	'''
	try:	return dot(vec,vec) / dot(vec,dir) * dir
	except ZeroDivisionError:	return vec3(nan)

def perpdot(a:vec2, b:vec2) -> float:
	return -a[1]*b[0] + a[0]*b[1]

def perp(v:vec2) -> vec2:
	''' perpendicular vector to the given vector '''
	return vec2(-v[1], v[0])
	
def dirbase(dir, align=vec3(1,0,0)):
	''' returns a base using the given direction as z axis (and the nearer vector to align as x) '''
	x = noproject(align, dir)
	if not length2(x) > NUMPREC**2:
		align = vec3(align[2],-align[0],align[1])
		x = noproject(align, dir)
	if not length2(x) > NUMPREC**2:
		align = vec3(align[1],-align[2],align[0])
		x = noproject(align, dir)
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
		*	(vec3,vec3), (vec3,mat3), (vec3,quat)   - (o,T) translation and rotation
		*	(vec3,vec3,vec3)                        - (x,y,z) base of vectors for rotation
		*	(vec3,vec3,vec3,vec3)                   - (o,x,y,z) translation and base of vectors for rotation
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
	
	raise TypeError('a transformation must be a  mat3, mat4, quat, (O,mat3), (O,quat), (0,x,y,z), not {}'.format(args))
	
def transformer(trans):
	''' return an function to apply the given transform on vectors
		
		:vec3:  translate the given position
		:mat3:  rotate the given position
		:quat:  rotate the given position
		:mat4:  affine transform (rotate then translate)
	'''
	if isinstance(trans, (dquat, fquat)):		trans = mat3_cast(trans)
	if callable(trans):							return trans
	if isinstance(trans, (dvec3, fvec3)):		return lambda v: v + trans
	if isinstance(trans, (dmat3, fmat3)):		return lambda v: trans * v
	if isinstance(trans, dmat4):				return lambda v: dvec3(trans * dvec4(v,1))
	if isinstance(trans, fmat4):				return lambda v: fvec3(trans * fvec4(v,1))
	raise TypeError('a transformer must be a  vec3, quat, mat3, mat4 or callable, not {}'.format(trans))



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

def intri_flat(pts, a,b):
	A,B,C = pts
	c = 1-a-b
	return a*A + b*B + c*C

def intri_sphere(pts, ptangents, a,b, etangents=None):
	''' cubic interpolation over a triangle (2 dimension space), edges are guaranteed to fit an interpol2 curve using the edge tangents
	
	.. note::
		if the tangents lengths are set to the edge lenghts, that version gives a result close to a sphere surface
	'''
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

def intri_smooth(pts, ptangents, a,b):
	''' cubic interpolation over a triangle, edges are guaranteed to fit an interpol2 curve using the edge tangents
	
	.. note::
		if the tangents lengths are set to the edge lenghts, that version gives a result that only blends between the curved edges, a less bulky result than `intri_sphere`
	'''
	A,B,C = pts
	ta,tb,tc = ptangents
	c = 1-a-b
	return (	0
			+	a**2 * (A + b*ta[0] + c*ta[1] + b*c*(ta[0]+ta[1]))
			+	b**2 * (B + c*tb[0] + a*tb[1] + c*a*(tb[0]+tb[1]))
			+	c**2 * (C + a*tc[0] + b*tc[1] + a*b*(tc[0]+tc[1]))
			+	2*(b*c + c*a + a*b) * (a*A + b*B + c*C)
			)

def intri_parabolic(pts, ptangents, a,b, etangents=None):
	''' quadratic interpolation over a triangle, edges are NOT fitting an interpol2 curve '''
	A,B,C = pts
	ta,tb,tc = ptangents
	c = 1-a-b
	return (	0
			+	a * (A + b*ta[0] + c*ta[1])
			+	b * (B + c*tb[0] + a*tb[1])
			+	c * (C + a*tc[0] + b*tc[1])
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
	if not l:	return 0
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

def bisect(l, index, key=lambda x:x):
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

# TODO rename it first
def find(iterator, predicate, default=None):
	for e in iterator:
		if predicate(e):	return e
	return default
			
def imax(iterable, default=None):
	''' return the index of the max of the iterable '''
	best = default
	score = -inf
	for i,o in enumerate(iterable):
		if o > score:
			score = o
			best = i
	if best is None:	raise IndexError('iterable is empty')
	return i


class Box:
	''' box always orthogonal to the base axis, used as convex for area delimitations '''
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
	
	def corners(self):
		''' create a list of the corners of the box '''
		c = self.min, self.max
		t = type(self.min)
		return [
			t(c[i&1][0], c[(i>>1)&1][1], c[(i>>2)&1][2])
			for i in range(8)
			]
	
	def isvalid(self):
		''' return True if the box defines a valid space (min coordinates <= max coordinates) '''
		return any(self.min <= self.max)
	def isempty(self):
		''' return True if the box contains a non null circumference '''
		return not any(self.min < self.max)
	
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
		''' extend the area of the box to bound the given point or box '''
		if isinstance(other, (dvec3, fvec3)):
			self.min = glm.min(self.min, other)
			self.max = glm.max(self.max, other)
		elif isinstance(other, Box):
			self.min = glm.min(self.min, other.min)
			self.max = glm.max(self.max, other.max)
		else:
			raise TypeError('unable to integrate {}'.format(type(other)))
		return self
	def intersection(self, other):
		''' intersection area between the 2 boxes '''
		if isinstance(other, Box):
			self.min = glm.max(self.min, other.min)
			self.max = glm.min(self.max, other.max)
		else:
			raise TypeError('expected a Box'.format(type(other)))
		return self
	def transform(self, trans):
		''' box bounding the current one in a transformed space '''
		if not self.isvalid():	return self
		trans = transformer(trans)
		return boundingbox((trans(p)  for p in self.corners()))
	def cast(self, vec):
		return Box(vec(self.min), vec(self.max))
	
	def __bool__(self):
		return self.isvalid()
	def __repr__(self):
		return '{}({}, {})'.format(self.__class__.__name__, repr(self.min), repr(self.max))

def boundingbox(obj, ignore=False, default=Box(width=vec3(-inf))) -> Box:
	''' return a box containing the object passed
		obj can be a vec3, Box, object with a `box()` method, or an iterable of such objects
	'''
	if isinstance(obj, Box):			return obj
	if isinstance(obj, (dvec3, fvec3)):	return Box(obj,obj)
	if hasattr(obj, 'box'):				return obj.box()
	if hasattr(obj, '__iter__'):
		obj = iter(obj)
		if ignore:
			bound = default
			for e in obj:
				try:	bound = boundingbox(e)
				except TypeError:	continue
				break
			for e in obj:
				try:	bound.union(e)
				except TypeError:	continue
		else:
			bound = boundingbox(next(obj, default))
			for e in obj:	bound.union(e)
		return bound
	if ignore:
		return default
	else:
		raise TypeError('unable to get a boundingbox from {}'.format(type(obj)))


