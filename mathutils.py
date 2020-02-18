''' Regroupement des fonctions et classes math de pymadcad '''

import glm
from glm import *
del version, license
from math import pi, atan2
from copy import deepcopy

'''
vec3 = dvec3
mat3 = dmat3
vec4 = dvec4
mat4 = dmat4

NUMPREC = 1e-12
'''

# numerical precision of floats used (float32 here, so 7 decimals, so 1e-6 when exponent is 1)
NUMPREC = 1e-6
COMPREC = 1-NUMPREC


def anglebt(x,y):
	n = length(x)*length(y)
	return acos(dot(x,y) / n)	if n else 0

def project(vec, dir):
	return dot(vec, dir) * dir

def perpdot(a:vec2, b:vec2) -> float:
	return -a[1]*b[0] + a[0]*b[1]
	
def dirbase(dir, align=vec3(1,0,0)):
	x = normalize(align - project(align, dir))
	if isnan(length(x)):
		align = vec3(align[2],-align[0],align[1])
		x = normalize(align - project(align, dir))
	y = normalize(cross(dir, x))
	return x,y,dir

# donne une mat3 effectuant une mise a l'echelle selon la direction donnée
def scaledir(dir, factor):
	return mat3(1) + (factor-1)*mat3(dir[0]*dir, dir[1]*dir, dir[2]*dir)

# donne la matrice de transformation 4x4
def transform(translation=None, rotation=None):
	if rotation is not None:	transform = mat4(rotation)
	else:						transform = mat4(1)
	if translation is not None:	transform[3] = vec4(translation)
	return transform

def interpol1(a, b, x):
	''' 1st order polynomial interpolation '''
	return x*a + (1-x)*b

def interpol2(a, b, x):
	''' 2nd order polynomial interpolation 
		a and b are iterable of successive derivatives of a[0] and b[0]
	'''
	return (	2*x*(1-x)  * interpol1(a,b,x)		# linear component
			+	x**2       * (a[0] + x*a[1])		# tangent
			+	(1-x)**2   * (b[0] + (x-1)*b[1])	# tangent
			)


def dichotomy_index(l, index, key=lambda x:x):
	start,end = 0, len(l)
	while start < end:
		mid = (start+end)//2
		val = key(l[mid])
		if val < index:		start =	mid+1
		elif val > index:	end =	mid
		else:	return mid
	return start



class Box:
	__slots__ = ('min', 'max')
	def __init__(self, min=None, max=None, center=vec3(0), width=vec3(0)):
		if min and max:			self.min, self.max = min, max
		else:					self.min, self.max = center-width, center+width
	
	@property
	def center(self):
		return (self.min + self.max) /2
	@property
	def width(self):
		return self.max - self.min
	
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
