from __future__ import annotations
from copy import copy

from .mathutils import *


class Box:
    ''' This class describes a box always orthogonal to the base axis, used as convex for area delimitations 
    
        box can be created either by poviding:
        
            - min and max corners
            - center position and size vector
            - center only (box will be empty)
            - size only (box will be centered around corner)
    
        This class is independent from the dimension or number precision of the used vectors. You can for instance have a `Box` of `vec2` as well as a box of `vec3`. However boxes with different vector types cannot interperate.
    
        Attributes:
            
            min:	vector of minimum coordinates of the box (usually bottom left corner)
            max:	vector of maximum coordinates of the box (usually top right corner)
    
    '''
    __slots__ = ('min', 'max')
    
    def __init__(self, min=None, max=None, center=vec3(0), size=vec3(-inf), width=None, alignment:vec3|float=0.5):
        # DEPRECATED
        if width is not None:
            size = width
        if min and max:			self.min, self.max = min, max
        else:					self.min, self.max = center-alignment*size, center+(1-alignment)*size
        
    @staticmethod
    def from_iter(it) -> Self:
        ''' create a box bounding the positions given by the input iterable '''
        it = iter(it)
        first = next(it)
        new = Box(first, first)
        for p in it:
            new |= p
        return new
    
    @staticmethod
    def from_torch(array: tuple) -> Box[vec2]:
        ''' create from a tuple `(xmin, ymin, xmax, ymax)` which is the pytorch convention '''
        return box(vec2(array[0:2]), vec2(array[2:4]))
    
    @staticmethod
    def from_cv(array: tuple) -> Box[vec2]:
        ''' create from a tuple `(xmin, ymax, width, height)` which is the opencv convention '''
        min = vec2(array[0:2])
        return box(min, min + vec2(array[2:4]))
    
    @staticmethod
    def from_matrix(matrix:mat4, centered=False) -> Box:
        ''' reciprocal of `self.to_matrix()` '''
        m = matrix
        s = _from_affine_mat[type(m)]
        o = _from_affine_mat[type(m)]
        for i in range(len(v)):
            o[i] = m[-1][i]
            s[i] = m[i][i]
        if centered:
            return Box(o-s, o+s)
        else:
            return Box(o, o+s)
    
    def to_torch(self) -> tuple:
        ''' convert to a tuple `(xmin, ymin, xmax, ymax)` '''
        return (*self.min, *self.max)
    
    def to_cv(self) -> tuple:
        ''' convert to a tuple `(xmin, ymax, width, height)` '''
        return (*self.min, *self.size())
        
    def to_matrix(self, centered=False) -> mat4:
        ''' return a matrix interpolating between corners
            - not centered: (0,0,0) and (1,1,1) are opposite corners
            - centered:  (-1,-1,-1) and (1,1,1) are opposite corners
        '''
        if centered:
            s = self.size/2
            o = self.min + self.size/2
        else:
            s = self.size
            o = self.min
        m = _to_affine_mat[type(s)]()
        for i in range(len(s)):
            m[i][i] = s[i]
            m[len(s)][i] = o[i]
        return m
    
    @property
    def center(self) -> vec3:
        ''' Mid coordinates of the box '''
        return (self.min + self.max) /2
    @property
    def width(self) -> vec3:
        ''' Diagonal vector of the box 
            deprecated
        '''
        return self.size
    @property
    def size(self) -> vec3:
        ''' Diagonal vector of the box '''
        return self.max - self.min
        
    def volume(self) -> float:
        ''' Volume inside '''
        v = 1
        for edge in self.size:
            v *= edge
        return v
    
    
    def map(self, alignment:vec3) -> vec3:
        ''' linearly interpolate between box corners '''
        return glm.mix(self.min, self.max, alignment)
    
    def corners(self) -> Iterator[vec3]:
        ''' return an iterable of the corner points of the box '''
        vec = copy(self.min)
        d = len(self.min)
        for i in range(2**d):
            for j in range(d):
                vec[j] = self.max[j] if (i>>j)&1 else self.min[j]
            yield copy(vec)
    
    def slice(self):
        ''' create a tuple of slice, a slice each dimension of the box '''
        return tuple(slice(start,stop)  for start, stop in zip(self.min, self.max))
        
    def transform(self, transformation) -> Box:
        ''' box bounding the current one in a transformed space '''
        if not self.isvalid():	return self
        return Box.from_iter(transformation * p   for p in self.corners())
        
        
    def touch_borders(self, other:Box) -> bool:
        ''' True if the box is touchng one or more border of the other box '''
        return any((self.min <= other.min) + (self.max >= other.max))
        
    def touch_corners(self, other:Box) -> bool:
        ''' True if the box is touching one or more corner of the other box '''
        return all((self.min <= other.min) + (self.max >= other.max))
        
    def contains(self, other:Box|vec3) -> bool:
        ''' return True if the given point is inside or on the surface of the box '''
        if isinstance(other, box):
            return all((self.min <= other.min) * (other.max <= self.max))
        else:
            return all((self.min <= other) * (other <= self.max))
        
    def inside(self, other:Box|vec3) -> bool:
        ''' return True if the given point is strictly inside the box '''
        if isinstance(other, box):
            return all((self.min < other.min) * (other.max < self.max))
        else:
            return all((self.min < other) * (other < self.max))

    def isvalid(self) -> bool:
        ''' Return True if the box defines a valid space (min coordinates <= max coordinates) '''
        return any(self.min <= self.max)
    
    def isempty(self) -> bool:
        ''' Return True if the box contains a non null volume '''
        return not any(self.min < self.max)
    
    
    def __add__(self, other:Box|vec3):
        ''' component-wise addition '''
        if isinstance(other, Box):	
            return Box(self.min + other.min, self.max + other.max)
        else:		
            return Box(self.min + other, self.max + other)
    
    def __sub__(self, other:Box|vec3):
        ''' component-wise substraction '''
        if isinstance(other, Box):	
            return Box(self.min - other.min, self.max - other.max)
        else:		
            return Box(self.min - other, self.max - other)
            
    def __iadd__(self, other:Box|vec3):
        if isinstance(other, Box):	
            self.min += other.min
            self.max += other.max
        else:
            self.min += other
            self.max += other
        return self
            
    def __isub__(self, other:Box|vec3):
        if isinstance(other, Box):	
            self.min -= other.min
            self.max -= other.max
        else:
            self.min -= other
            self.max -= other
        return self
    
    def merge_update(self, other:Box|vec3) -> Box:
        ''' Extend the volume of the current box to bound the given point or box '''
        if isinstance(other, Box):
            self.min = glm.min(self.min, other.min)
            self.max = glm.max(self.max, other.max)
        else:
            self.min = glm.min(self.min, other)
            self.max = glm.max(self.max, other)
        return self
    
    def intersection_update(self, other:Box|vec3) -> Box:
        ''' Reduce the volume of the current box to the intersection between the 2 boxes '''
        if isinstance(other, Box):
            self.min = glm.max(self.min, other.min)
            self.max = glm.min(self.max, other.max)
        else:
            raise TypeError('expected a Box'.format(type(other)))
        return self
    
    __ior__ = merge_update
    __iand__ = intersection_update
    
    def merge(self, other:Box|vec3) -> Box:
        ''' Return a box containing the current and the given box (or point) 
        
            Example:
                
                >>> Box(vec2(1,2), vec2(2,3)) .merge(vec2(1,4))
                Box(vec2(1,2), vec2(2,4))
                
                >>> Box(vec2(1,2), vec2(2,3)) .merge(Box(vec2(1,-4), vec2(2,8)))
                Box(vec2(1,-4), vec2(2,8))
        
        '''
        if isinstance(other, box):
            return Box(	glm.min(self.min, other.min),
                        glm.max(self.max, other.max))
        else:
            return Box(	glm.min(self.min, other),
                        glm.max(self.max, other))
    
    def intersection(self, other) -> Box:
        ''' Return a box for the volume common to the current and the given box 
        
            Example:
            
                >>> Box(vec2(-1,2), vec2(2,3)) .intersection(Box(vec2(1,-4), vec2(2,8)))
                Box(vec2(1,2), vec2(2,3))
        
        '''
        return Box(	glm.max(self.min, other.min),
                    glm.min(self.max, other.max))
    
    __or__ = merge
    __and__ = intersection
    
    def cast(self, vec:type) -> Box:
        ''' convert to a box with an other vector type '''
        return Box(vec(self.min), vec(self.max))
    
    def __bool__(self):
        return self.isvalid()
    
    def __array__(self):
        return np.array(self.to_torch())
    
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, repr(self.min), repr(self.max))


_to_affine_mat = {
    vec1: mat2,
    vec2: mat3,
    vec3: mat4,
    fvec1: fmat2,
    fvec2: fmat3,
    fvec3: fmat4,
    }
_from_affine_mat = {v:k  for k,v in _to_affine_mat.items()}



def boundingbox(obj, ignore=False, default=Box(size=vec3(-inf))) -> Box:
	''' Return a box containing the object passed
		`obj` can be a vec3, Box, object with a `box()` method, or an iterable of such objects
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
			bound = Box(bound.min, bound.max)
			for e in obj:
				try:	bound.merge_update(e)
				except TypeError:	continue
		else:
			bound = boundingbox(next(obj, default))
			bound = Box(bound.min, bound.max)
			for e in obj:	bound.merge_update(e)
		return bound
	if ignore:
		return default
	else:
		raise TypeError('unable to get a boundingbox from {}'.format(type(obj)))
