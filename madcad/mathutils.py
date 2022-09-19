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

from arrex import typedlist
import arrex.glm

# alias definitions
vec2 = dvec2
mat2 = dmat2
vec3 = dvec3
mat3 = dmat3
vec4 = dvec4
mat4 = dmat4
quat = dquat

# numerical precision of floats used
NUMPREC = 1e-13	# float64 here, so 14 decimals
#NUMPREC = 1e-6	# float32 here, so 7 decimals, so 1e-6 when exponent is 1
COMPREC = 1-NUMPREC



# common base definition, for end user
O = vec3(0,0,0)
X = vec3(1,0,0)
Y = vec3(0,1,0)
Z = vec3(0,0,1)


def isfinite(x):
	''' return false if x contains a `inf` or a `nan` '''
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
	except ZeroDivisionError:	
		if dot(vec,vec):		return vec3(nan)
		else:					return vec3(0)
		
	
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
	except ZeroDivisionError:	
		if dot(vec,vec):		return vec3(nan)
		else:					return vec3(0)

def perpdot(a:vec2, b:vec2) -> float:
	''' dot product of a with perpendicular vector to b, equivalent to `dot(a, prep(b))` '''
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
		dir = dir / factor
	return mat3(1) + (factor-1)*mat3(dir[0]*dir, dir[1]*dir, dir[2]*dir)
	
def rotatearound(angle, *args) -> mat4:
	''' return a transformation matrix for a rotation around an axis
		
		rotatearound(angle, axis)
		rotatearound(angle, origin, dir)
	'''
	if len(args) == 1:		origin, dir = args[0]
	elif len(args) == 2:	origin, dir = args
	else:
		raise TypeError('invalid use of rotatearound')
	
	r = mat3_cast(angleAxis(angle, dir))
	m = mat4(r)
	m[3] = vec4(origin - r*origin, 1)
	return m

def transform(*args) -> mat4:
	''' create an affine transformation matrix.
		
		supported inputs:
			:mat4:                                    obviously returns it unmodified
			:float:                                   scale using the given ratio 
			:vec3:                                    translation only
			:quat, mat3, mat4:                        rotation only
			:(vec3,vec3), (vec3,mat3), (vec3,quat):   `(o,T)` translation and rotation
			:(vec3,vec3,vec3):                        `(x,y,z)` base of vectors for rotation
			:(vec3,vec3,vec3,vec3):                   `(o,x,y,z)` translation and base of vectors for rotation
	'''
	if len(args) == 1 and isinstance(args[0], (tuple,list)):
		args = args[0]
	if len(args) == 1:
		if isinstance(args[0], mat4):	return args[0]
		elif isinstance(args[0], mat3):	return mat4(args[0])
		elif isinstance(args[0], quat):	return mat4_cast(args[0])
		elif isinstance(args[0], vec3):	return translate(mat4(1), args[0])
		elif isinstance(args[0], (int,float)):	return mat4(args[0])
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
		
		supported inputs:
			:float:	scale by the given ratio
			:vec3:  translate the given position
			:mat3:  rotate the given position
			:quat:  rotate the given position
			:mat4:  affine transform (rotate then translate)
	'''
	if isinstance(trans, (dquat, fquat)):		trans = mat3_cast(trans)
	if callable(trans):													return trans
	if isinstance(trans, (dvec3, fvec3)):								return lambda v: v + trans
	if isinstance(trans, (dmat3, fmat3, dmat4, fmat4, int, float)):		return lambda v: trans * v
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

hermite = spline = interpol2

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
	l = length2(dir)
	if not l:	return 0
	x = dot(pt-edge[0], dir)/l
	if   x < 0:	return distance(pt,edge[0])
	elif x > 1:	return distance(pt,edge[1])
	else:
		return length(noproject(pt-edge[0], dir))

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
		if o >= score:
			score = o
			best = i
	if best is None:	raise IndexError('iterable is empty')
	return best


def linrange(start, stop=None, step=None, div=0, end=True):
	''' yield successive intermediate values between start and stop 
		
		stepping:
		* if `step` is given, it will be the amount between raised value until it gets over `stop`
		* if `div` is given, it will be the number of intermediate steps between `start` and `stop` (with linear spacing)
		
		ending:
		* if `end` is True, it will stop iterate with value `stop` (or just before)
		* if `end` is False, it will stop iterating just before `stop` and never with `stop`
		
		NOTE:  
			If step is given and is not a multiple of `stop-start` then `end` has no influence
	'''
	if stop is None:	start, stop = 0, start
	if step is None:	step = (stop-start)/(div+1)
	elif step * (stop-start) < 0:	step = -step
	if not end:			stop -= step
	stop += NUMPREC*stop
	
	t = start
	while t <= stop:
		yield t
		t += step

class Box:
	''' This class describes a box always orthogonal to the base axis, used as convex for area delimitations 
	
		This class is independent from the dimension or number precision of the used vectors. You can for instance have a `Box` of `vec2` as well as a box of `vec3`. however boxes with different vector types cannot interperate.
	
		Attributes:
			
			min:	vector of minimum coordinates of the box (usually bottom left corner)
			max:	vector of maximum coordinates of the box (usually top right corner)
	
	'''
	__slots__ = ('min', 'max')
	def __init__(self, min=None, max=None, center=vec3(0), width=vec3(-inf)):
		if min and max:			self.min, self.max = min, max
		else:					self.min, self.max = center-width/2, center+width/2
	
	@property
	def center(self) -> vec3:
		''' mid coordinates of the box '''
		return (self.min + self.max) /2
	@property
	def width(self) -> vec3:
		''' diagonal vector of the box '''
		return self.max - self.min
	
	def corners(self) -> '[vec3]':
		''' create a list of the corners of the box '''
		c = self.min, self.max
		t = type(self.min)
		return [
			t(c[i&1][0], c[(i>>1)&1][1], c[(i>>2)&1][2])
			for i in range(8)
			]
			
	def volume(self) -> float:
		''' volume inside '''
		v = 1
		for edge in self.width:
			v *= edge
		return v
		
	def contain(self, point):
		''' return True if the given point is inside or on the surface of the box '''
		return all(self.min <= point) and all(point <= self.max)
		
	def inside(self, point):
		''' return True if the given point is strictly inside the box '''
		return all(self.min < point) and all(point < self.max)
	
	def isvalid(self):
		''' return True if the box defines a valid space (min coordinates <= max coordinates) '''
		return any(self.min <= self.max)
	def isempty(self):
		''' return True if the box contains a non null volume '''
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
	
	def union_update(self, other) -> 'self':
		''' extend the volume of the current box to bound the given point or box '''
		if isinstance(other, (dvec3, fvec3)):
			self.min = glm.min(self.min, other)
			self.max = glm.max(self.max, other)
		elif isinstance(other, Box):
			self.min = glm.min(self.min, other.min)
			self.max = glm.max(self.max, other.max)
		else:
			raise TypeError('unable to integrate {}'.format(type(other)))
		return self
	
	def intersection_update(self, other) -> 'self':
		''' reduce the volume of the current box to the intersection between the 2 boxes '''
		if isinstance(other, Box):
			self.min = glm.max(self.min, other.min)
			self.max = glm.min(self.max, other.max)
		else:
			raise TypeError('expected a Box'.format(type(other)))
		return self
	
	def union(self, other) -> 'Box':
		''' return a box containing the current and the given box (or point) 
		
			Example:
				
				>>> Box(vec2(1,2), vec2(2,3)) .union(vec3(1,4))
				Box(vec2(1,2), vec2(2,4))
				
				>>> Box(vec2(1,2), vec2(2,3)) .union(Box(vec3(1,-4), vec3(2,8)))
				Box(vec2(1,-4), vec3(2,8))
		
		'''
		if isinstance(other, (dvec3, fvec3)):
			return Box(	glm.min(self.min, other),
						glm.max(self.max, other))
		elif isinstance(other, Box):
			return Box(	glm.min(self.min, other.min),
						glm.max(self.max, other.max))
		else:
			raise TypeError('unable to integrate {}'.format(type(other)))
	
	def intersection(self, other) -> 'Box':
		''' return a box for the volume common to the current and the given box 
		
			Example:
			
				>>> Box(vec2(-1,2), vec2(2,3)) .intersection(Box(vec3(1,-4), vec3(2,8)))
				Box(vec2(1,2), vec3(2,3))
		
		'''
		if isinstance(other, Box):
			return Box(	glm.max(self.min, other.min),
						glm.min(self.max, other.max))
		else:
			raise TypeError('expected a Box'.format(type(other)))
	
	def transform(self, trans) -> 'Box':
		''' box bounding the current one in a transformed space '''
		if not self.isvalid():	return self
		trans = transformer(trans)
		return boundingbox((trans(p)  for p in self.corners()))
	
	def cast(self, vec) -> 'Box':
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
			for e in obj:	bound.union_update(e)
		return bound
	if ignore:
		return default
	else:
		raise TypeError('unable to get a boundingbox from {}'.format(type(obj)))


