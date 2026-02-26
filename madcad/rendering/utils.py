from __future__ import annotations

import moderngl as mgl
from numpy import ndarray
import numpy as np
from pyglm.glm import ivec2, uvec2, fvec4, mix

from .. import settings

__all__ = [
	"writeproperty",
	"forwardproperty",
	"sceneshare",
	"receiver",
	"Weak",
	"Rc",
	"CheapMap",
	"snail",
	"snailaround",
	"glsize",
	"highlight_color",
	"vec_to_qpoint",
	"qpoint_to_vec",
	"vec_to_qsize",
	"qsize_to_vec",
]

def writeproperty(func):
	''' Decorator to create a property that has only an action on variable write '''
	fieldname = '_'+func.__name__
	def getter(self):	
		return getattr(self, fieldname)
	def setter(self, value):
		setattr(self, fieldname, value)
		func(self)
	return property(getter, setter, doc=func.__doc__)
	
def forwardproperty(attribute, sub):
	''' read/write property to a attribute's attribute '''
	def getter(self):
		return getattr(getattr(self, attribute), sub)
	def setter(self, value):
		setattr(getattr(self, attribute), sub, value)
	return property(getter, setter)

def sceneshare(generator, immortal=False):
	''' decorator for shared ressources with the scene '''
	name = generator
	def share(self, scene):
		if not scene.shared.get(name):
			data = generator(self, scene)
			if immortal:
				data = Rc(data)
			else:
				data = Weak(data)
			scene.shared[name] = data
		setattr(self, generator.__name__, Rc(scene.shared[name]))
		vars(self).update(scene.shared[name]())
	share.__doc__ = generator.__doc__
	return share

def receiver(generator) -> Callable:
	''' simple helper running the generator until the first yield, and returning its send method '''
	next(generator, None)
	def callback(evt):
		try:
			generator.send(evt)
		except StopIteration:
			return False
		else:
			return True
	return callback
	
class Weak:
	''' weak reference to an object '''
	__slots__ = 'object', 'refcount'
	def __init__(self, object):
		self.object = object
		self.refcount = 0
	def __bool__(self):
		''' return True if the object is still alive and has not been dropped '''
		return self.refcount > 0
	def __call__(self):
		''' retreive the underlying object or None if deallocated '''
		return self.object

class Rc:
	''' strong counted reference to an object '''
	__slots__ = 'weak'
	def __init__(self, other):
		if isinstance(other, Weak):
			self.weak = other
		elif isinstance(other, Rc):
			self.weak = other.weak
		else:
			self.weak = Weak(other)
		self.weak.refcount += 1
	def __del__(self):
		if self.weak is not None:
			self.weak.refcount -= 1
			if self.weak.refcount <= 0:
				self.weak.object = None
		self.weak = None
	def __call__(self):
		''' retreive the underlying object '''
		return self.weak.object

class CheapMap:
	''' object retreiving portions of a framebuffer, on demand and avoiding redundant memory copies '''
	def __init__(self, framebuffer: mgl.Framebuffer, attachment:int):
		self.framebuffer = framebuffer
		self.attachment = attachment
		if attachment < 0:
			renderbuffer = framebuffer.depth_attachment
		else:
			renderbuffer = framebuffer.color_attachments[attachment]
		self.dtype = renderbuffer.dtype
		self.components = renderbuffer.components
		size = renderbuffer.size
		self.buffer = np.zeros(size[1]*size[0], self.dtype)
		
		self.clear()
	
	def clear(self):
		''' clear the last retreived region
			
			next call to `region` will trigger a transfer from the GPU 
		'''
		self.viewport = (1, 1, -1, -1)
		
	def region(self, viewport: tuple) -> ndarray:
		''' retreive a given region of the framebuffer from the GPU
		
			any next extracted region that is inside this one will no need a new transfer
		'''
		if (viewport[0] < self.viewport[0] or viewport[2] > self.viewport[2] 
		or  viewport[1] < self.viewport[1] or viewport[3] > self.viewport[3]):
			self.viewport = viewport
			self.framebuffer.read_into(self.buffer, 
				(viewport[0], self.framebuffer.height-viewport[3], viewport[2]-viewport[0], viewport[3]-viewport[1]), 
				attachment=self.attachment, components=self.components, dtype=self.dtype)
		
		view = self.buffer[:(self.viewport[3]-self.viewport[1]) * (self.viewport[2]-self.viewport[0])]
		view = view.reshape((self.viewport[3]-self.viewport[1], self.viewport[2]-self.viewport[0]))
		view = view[::-1]
		return view[
			viewport[1]-self.viewport[1]:viewport[3]-self.viewport[1],
			viewport[0]-self.viewport[0]:viewport[2]-self.viewport[0],
			]


def snail(radius):
	''' Generator of coordinates snailing around 0,0 '''
	x = 0
	y = 0
	for r in range(radius):
		for x in range(-r,r):		yield ivec2(x,-r)
		for y in range(-r,r):		yield ivec2(r, y)
		for x in reversed(range(-r,r)):	yield ivec2(x, r)
		for y in reversed(range(-r,r)):	yield ivec2(-r,y)

def snailaround(pt, box, radius):
	''' Generator of coordinates snailing around pt, coordinates that goes out of the box are skipped '''
	cx,cy = pt
	mx,my = box
	for rx,ry in snail(radius):
		x,y = cx+rx, cy+ry
		if 0 <= x and x < mx and 0 <= y and y < my:
			yield ivec2(x,y)

def glsize(size: uvec2) -> uvec2:
	# if the size is not a multiple of 4, it seems that openGL or Qt doesn't understand its strides well
	m = 4
	return size + uvec2(-ivec2(size)%m)

def highlight_color(display: Display, color: fvec3) -> fvec3:
	if display.selected:    highlight = fvec4(settings.display['selection_color'])
	elif display.hovered:   highlight = fvec4(settings.display['hover_color'])
	else:                   highlight = fvec4(0)
	return mix(color, highlight.rgb, highlight.a)
			
# qt conversion functions

try:
	from ..qt import QPoint, QSize
except ImportError:
	pass
else:
	def vec_to_qpoint(p: ivec2) -> QPoint: 	return QPoint(p.x, p.y)
	def qpoint_to_vec(p: QPoint) -> ivec2:	return ivec2(p.x(), p.y())
	def vec_to_qsize(p: uvec2) -> QSize: 	return QSize(p.x, p.y)
	def qsize_to_vec(p: QSize) -> uvec2:	return uvec2(p.size(), p.height())
