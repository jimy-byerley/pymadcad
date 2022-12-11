# This file is part of pymadcad,  distributed under license LGPL v3

'''	Display module of pymadcad
	
	This module provides a render pipeline system featuring:

	- class `Scene` to gather the data to render
	- widget `View` that actually renders the scene 
	- the display protocol, that allows any object to define its `Display` subclass to be rendered in a scene.

	The view is for window integration and user interaction. `Scene` is only to manage the objects to render . Almost all madcad data types can be rendered to scenes being converted into an appropriate subclass of `Display`. Since the conversion from madcad data types into display instance is automatically handled via the *display protocol*, you usually don't need to deal with displays directly.

	
	display protocol
	----------------
		a displayable is an object that implements the signatue of Display:
		
			class display:
				box (Box)                      # delimiting the display, can be an empty or invalid box
				world (fmat4)                  # local transformation
				
				__getitem__                    # access to subdisplays if there is
				stack(scene)                   # rendering routines (can be methods, or any callable)
				
				duplicate(src,dst)             # copy the display object for an other scene if possible
				update(scene,displayable)     # upgrade the current display to represent the given displayable
				control(...)                   # handle events
		
		For more details, see class Display below
		
	ATTENTION
	---------
		As the GPU native precision is f4 (float 32 bits), all the vector stuff regarding rendering is made using simple precision types: `fvec3, fvec4, fmat3, fmat4, ...`
	
	NOTE
	----
		There is some restrictions using the widget. This is due to some Qt limitations (and design choices), that Qt is using separated opengl contexts for each independent widgets or window.
		
		- a View should not be reparented once displayed
		- a View can't share a scene with Views from an other window
		- to share a Scene between Views, you must activate 
				QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
'''

from copy import copy, deepcopy
from operator import itemgetter
import traceback

import moderngl as mgl
import numpy.core as np

from PyQt5.QtCore import Qt, QPoint, QEvent
from PyQt5.QtWidgets import QApplication, QWidget, QOpenGLWidget
from PyQt5.QtGui import QSurfaceFormat, QMouseEvent, QInputEvent, QKeyEvent, QTouchEvent, QFocusEvent

from PIL import Image

from .mathutils import *
from .common import ressourcedir
from . import settings

from .nprint import nprint


# minimum opengl version required by the rendering pipeline
opengl_version = (3,3)
# shared open gl context, None if not yet initialized
global_context = None


def show(scene:dict, interest:Box=None, size=uvec2(400,400), projection=None, navigation=None, **options):
	''' 
		easy and convenient way to create a window containing a `View` on a created `Scene`
		
		If a Qt app is not already running, the functions returns when the window has been closed and all GUI destroyed
		
		Parameters:
			scene:     a mapping (dict or list) giving the objects to render in the scene
			interest:  the region of interest to zoom on at the window initialization
			size:      the window size (pixel)
			options:   options to set in `Scene.options`
		
		Tip:
			For integration in a Qt window or to manipulate the view, you should directly use `View`
	'''
	global global_context

	if isinstance(scene, list):	scene = dict(enumerate(scene))
	# retro-compatibility fix, shall be removed in future versions
	if 'options' in options:	options.update(options['options'])
	if not isinstance(scene, Scene):	scene = Scene(scene, options)

	app = QApplication.instance()
	created = False
	if not app:
		import sys
		QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
		app = QApplication(sys.argv)
		global_context = None
		created = True

	# use the Qt color scheme if specified
	if settings.display['system_theme']: 
		settings.use_qt_colors()

	# create the scene as a window
	view = View(scene, projection, navigation)
	view.resize(*size)
	view.show()

	# make the camera see everything
	if not interest:	
		interest = view.scene.box()
	view.center(interest.center)
	view.adjust(interest)

	if created:
		err = app.exec()
		if err != 0:	print('error: Qt exited with code', err)

def render(scene, options=None, interest:Box=None, navigation=None, projection=None, size=uvec2(400,400)):
	''' shortcut to render the given objects to an image, returns a PIL Image
	
		For repeated renderings or view manipualtion, you should directly use `Offscreen`
		
		NOTE:
			the system theme colors cannot be automatically loaded since no running QApplication is assumed in the function
	'''
	if isinstance(scene, list):	scene = dict(enumerate(scene))
	if not isinstance(scene, Scene):	scene = Scene(scene, options)

	# create the scene and an offscreen renderer
	view = Offscreen(scene, size, navigation=navigation, projection=projection)

	# load objects in the scene, so the scene's box can be computed
	with scene.ctx:
		scene.dequeue()

	# make the camera see everything
	if not navigation:
		if not interest:
			interest = view.scene.box()
		view.center(interest.center)
		view.adjust(interest)

	return view.render()





class Display:
	''' Blanket implementation for displays.
		This class signature is exactly the display protocol specification
		
		Attributes:
		
			world(fmat4):  matrix from local space to parent space
			box(Box):      boudingbox of the display in local space
			
		These attributes are variable members by default but can be overriden as properties if needed.
	'''
	
	# mendatory part of the protocol
	
	box = Box(center=0, width=fvec3(-inf))	# to inform the scene and the view of the object size
	world = fmat4(1)		# set by the display containing this one if it is belonging to a group
	
	def display(self, scene) -> 'self':
		''' displays are obviously displayable as themselves '''
		return self
	def stack(self, scene) -> '[(key, target, priority, callable)]':
		''' rendering functions to insert in the renderpipeline.
		
			the expected result can be any iterable providing tuples `(key, target, priority, callable)` such as:
			
			:key:	    a tuple with the successive keys in the displays tree, most of the time implementers set it to `()`  because it doesn't belong to a subpart of the Display.
			:target:    the name of the render target in the view that will be rendered (see View)
			:priority:  a float that is used to insert the callable at the proper place in the rendering stack
			:callable:  a function that renders, signature is  `func(view)`			
			
			The view contains the uniforms, rendering targets and the scene for common ressources
		'''
		return ()
	def duplicate(self, src, dst) -> 'display/None':
		''' duplicate the display for an other scene (other context) but keeping the same memory buffers when possible.
			
			return None if not possible or not implemented.
		'''
		return None
	def __getitem__(self, key) -> 'display':
		''' get a subdisplay by its index/key in this display (like in a scene) '''
		raise IndexError('{} has no sub displays'.format(type(self).__name__))
	def update(self, scene, displayable) -> bool:
		''' update the current displays internal datas with the given displayable .
			
			if the display cannot be upgraded, it must return False to be replaced by a fresh new display created from the displayable
		'''
		return False
	
	# optional part for usage with Qt
	
	selected = False
	
	def control(self, view, key, sub, evt: 'QEvent'):
		''' handle input events occuring on the area of this display (or of one of its subdisplay).
			for subdisplay events, the parents control functions are called first, and the sub display controls are called only if the event is not accepted by parents
			
			Parameters:
				key:    the key path for the current display
				sub:    the key path for the subdisplay
				evt:    the Qt event (see Qt doc)
		'''
		pass

def qt_2_glm(v):
	if isinstance(v, (QPoint, QPointF)):	return vec2(v.x(), v.y())
	elif isinstance(v, (QSize, QSizeF)):	return vec2(v.width(), v.height())
	else:
		raise TypeError("can't convert {} to vec2".format(type(v).__name__))

def navigation_tool(dispatcher, view):
	''' internal navigation tool '''	
	ctrl = alt = slow = False
	nav = curr = None
	moving = False
	hastouched = False
	while True:
		evt = yield
		evt.ignore()	# ignore the keys to pass shortcuts to parents
		
		if isinstance(evt, QKeyEvent):
			k = evt.key()
			press = evt.type() == QEvent.KeyPress
			if	 k == Qt.Key_Control:	ctrl = press
			elif k == Qt.Key_Alt:		alt = press
			elif k == Qt.Key_Shift:		slow = press
			if ctrl and alt:		curr = 'zoom'
			elif ctrl:				curr = 'pan'
			elif alt:				curr = 'rotate'
			else:					curr = None
			# no accept because the shortcuts need to get the keys also
		elif evt.type() == QEvent.MouseButtonPress:
			last = evt.pos()
			if evt.button() == Qt.MiddleButton:
				nav = 'rotate'
			else:
				nav = curr
			# prevent any scene interaction
			if nav:
				evt.accept()
		elif evt.type() == QEvent.MouseMove:
			if nav:
				moving = True
				gap = evt.pos() - last
				dx = gap.x()/view.height()
				dy = gap.y()/view.height()
				if nav == 'pan':		view.navigation.pan(dx, dy)
				elif nav == 'rotate':	view.navigation.rotate(dx, dy, 0)
				elif nav == 'zoom':		
					middle = QPoint(view.width(), view.height())/2
					f = (	(last-middle).manhattanLength()
						/	(evt.pos()-middle).manhattanLength()	)
					view.navigation.zoom(f)
				last = evt.pos()
				view.update()
				evt.accept()
		elif evt.type() == QEvent.MouseButtonRelease:
			if moving:
				moving = False
				evt.accept()
		elif evt.type() == QEvent.Wheel:
			view.navigation.zoom(exp(-evt.angleDelta().y()/(8*90)))	# the 8 factor is there because of the Qt documentation
			view.update()
			evt.accept()
	
				
		elif isinstance(evt, QTouchEvent):
			nav = None
			pts = evt.touchPoints()
			# view rotation
			if len(pts) == 2:
				startlength = (pts[0].lastPos()-pts[1].lastPos()).manhattanLength()
				zoom = startlength / (pts[0].pos()-pts[1].pos()).manhattanLength()
				displt = (	(pts[0].pos()+pts[1].pos()) /2 
						-	(pts[0].lastPos()+pts[1].lastPos()) /2 ) /view.height()
				dc = pts[0].pos() - pts[1].pos()
				dl = pts[0].lastPos() - pts[1].lastPos()
				rot = atan2(dc.y(), dc.x()) - atan2(dl.y(), dl.x())
				view.navigation.zoom(zoom)
				view.navigation.rotate(displt.x(), displt.y(), rot)
				hastouched = True
				view.update()
				evt.accept()
			# view translation
			elif len(pts) == 3:
				lc = (	pts[0].lastPos() 
					+	pts[1].lastPos() 
					+	pts[2].lastPos() 
					)/3
				lr = (	(pts[0].lastPos() - lc) .manhattanLength()
					+	(pts[1].lastPos() - lc) .manhattanLength()
					+	(pts[2].lastPos() - lc) .manhattanLength()
					)/3
				cc = (	pts[0].pos() 
					+	pts[1].pos() 
					+	pts[2].pos() 
					)/3
				cr = (	(pts[0].pos() - cc) .manhattanLength()
					+	(pts[1].pos() - cc) .manhattanLength()
					+	(pts[2].pos() - cc) .manhattanLength()
					)/3
				zoom = lr / cr
				displt = (cc - lc)  /view.height()
				view.navigation.zoom(zoom)
				view.navigation.pan(displt.x(), displt.y())
				hastouched = True
				view.update()
				evt.accept()
			# finish a gesture
			elif evt.type() in (QEvent.TouchEnd, QEvent.TouchUpdate):
				evt.accept()
				

class Turntable:
	''' navigation rotating on yaw and pitch around a center 
	
		object used as `View.navigation`
	'''
	def __init__(self, center:fvec3=0, distance:float=1, yaw:float=0, pitch:float=0):
		self.center = fvec3(center)
		self.yaw = yaw
		self.pitch = pitch
		self.distance = distance
		self.tool = navigation_tool
		
	def rotate(self, dx, dy, dz):
		self.yaw += dx*pi
		self.pitch += dy*pi
		if self.pitch > pi/2:	self.pitch = pi/2
		if self.pitch < -pi/2:	self.pitch = -pi/2
	def pan(self, dx, dy):
		mat = transpose(mat3_cast(inverse(fquat(fvec3(pi/2-self.pitch, 0, -self.yaw)))))
		self.center += ( mat[0] * -dx + mat[1] * dy) * self.distance/2
	def zoom(self, f):
		self.distance *= f
	
	def matrix(self) -> fmat4:
		# build rotation from view euler angles
		rot = inverse(fquat(fvec3(pi/2-self.pitch, 0, -self.yaw)))
		mat = translate(mat4_cast(rot), -self.center)
		mat[3][2] -= self.distance
		return mat

class Orbit:
	''' navigation rotating on the 3 axis around a center.
	
		object used as `View.navigation`
	'''
	def __init__(self, center:fvec3=0, distance:float=1, orient:fvec3=fvec3(1,0,0)):
		self.center = fvec3(center)
		self.distance = float(distance)
		self.orient = fquat(orient)
		self.tool = navigation_tool
		
	def rotate(self, dx, dy, dz):
		# rotate from view euler angles
		self.orient = inverse(fquat(fvec3(-dy, -dx, dz) * pi)) * self.orient
	def pan(self, dx, dy):
		x,y,z = transpose(mat3_cast(self.orient))
		self.center += (fvec3(x) * -dx + fvec3(y) * dy) * self.distance/2
	def zoom(self, f):
		self.distance *= f
	
	def matrix(self) -> fmat4:
		mat = translate(mat4_cast(self.orient), -self.center)
		mat[3][2] -= self.distance
		return mat


class Perspective:
	''' object used as `View.projection` 
	
		Attributes:
			fov (float):	field of view (rad), defaulting to `settings.display['field_of_view']`
	'''
	def __init__(self, fov=None):
		self.fov = fov or settings.display['field_of_view']
	def matrix(self, ratio, distance) -> fmat4:
		return perspective(self.fov, ratio, distance*1e-2, distance*1e4)

class Orthographic:
	''' object used as `View.projection` 
	
		Attributes:
			size (float):  
			
				factor between the distance from camera to navigation center and the zone size to display
				defaulting to `tan(settings.display['field_of_view']/2)`
	'''
	def __init__(self, size=None):
		self.size = size or tan(settings.display['field_of_view']/2)
	def matrix(self, ratio, distance) -> fmat4:
		return fmat4(1/(ratio*distance*self.size), 0, 0, 0,
		            0,       1/(distance*self.size), 0, 0,
		            0,       0,          -2/(distance*(1e3-1e-2)), 0,
		            0,       0,          -(1e3+1e-2)/(1e3-1e-2), 1)


class Scene:
	''' rendeing pipeline for madcad displayable objects 
		
		This class is gui-agnostic, it only relys on opengl, and the context has to be created by te user.
		
		When an object is added to the scene, a Display is not immediately created for it, the object is put into the queue and the Display is created at the next render.
		If the object is removed from the scene before the next render, it is dequeued.
		
		Attributes:
		
			ctx:           moderngl Context (must be the same for all views using this scene)
			ressources (dict):    dictionnary of scene ressources (like textures, shaders, etc) index by name
			options (dict):       dictionnary of options for rendering, initialized with a copy of `settings.scene`
			
			displays (dict):      dictionnary of items in the scheme `{'name': Display}`
			stacks (list):        lists of callables to render each target `{'target': [(key, priority, callable(view))]}`
			setup (dict):         setup of each rendering target `{'target': callable}`
			
			touched (bool):       flag set to True if the stack must be recomputed at the next render time (there is a change in a Display or in one of its children)
	'''
	
	def __init__(self, objs=(), options=None, ctx=None, setup=None):
		# context variables
		self.ctx = ctx
		self.ressources = {}	# context-related ressources, shared across displays, but not across contexts (shaders, vertexarrays, ...)
		
		# rendering options
		self.options = deepcopy(settings.scene)
		if options:	self.options.update(options)
		
		# render elements
		self.queue = {}	# list of objects to display, not yet loaded on the GPU
		self.displays = {} # displays created from the inserted objects, associated to their insertion key
		self.stacks = {}	# dict of list of callables, that constitute the render pipeline:  (key,  priority, callable)
		self.setup = setup or {}	# callable for each target
		
		self.touched = False
		self.update(objs)
	
	# methods to manage the rendering pipeline
	
	def add(self, displayable, key=None) -> 'key':
		''' add a displayable object to the scene, if key is not specified, an unused integer key is used 
			the object is not added to the the renderpipeline yet, but queued for next rendering.
		'''
		if key is None:
			for i in range(len(self.displays)+len(self.queue)+1):
				if i not in self.displays and i not in self.queue:	key = i
		self.queue[key] = displayable
		return key

	def __setitem__(self, key, value):
		''' equivalent with self.add with a key '''
		self.queue[key] = value
	def __getitem__(self, key) -> 'display':
		''' get the displayable for the given key, raise when there is no object or when the object is still in queue. '''
		return self.displays[key]
	def __delitem__(self, key):
		''' remove an item from the scene, at the root level '''
		if key in self.displays:
			del self.displays[key]
		if key in self.queue:
			del self.queue[key]
		for stack in self.stacks.values():
			for i in reversed(range(len(stack))):
				if stack[i][0][0] == key:
					stack.pop(i)
					
	def item(self, key):
		''' get the Display associated with the given key, descending the parenting tree 
		
			The parents must all make their children accessible via `__getitem__`
		'''
		disp = self.displays
		for i in range(1,len(key)):
			disp = disp[key[i-1]]
		return disp
	
	def update(self, objs:dict):
		''' rebuild the scene from a dictionnary of displayables 
			update former displays if possible instead of replacing it
		'''
		self.queue.update(objs)
		self.touch()
	
	def sync(self, objs:dict):
		''' update the scene from a dictionnary of displayables, the former values that cannot be updated are discarded '''
		for key in list(self.displays):
			if key not in objs:
				del self.displays[key]
		self.update(objs)
	
	def touch(self):
		''' shorthand for `self.touched = True` '''
		self.touched = True
		
	def dequeue(self):
		''' load all pending objects to insert into the scene.
			this is called automatically by the next `render()` if `touch()` has been called
		'''
		if self.queue:
			with self.ctx:
				self.ctx.finish()
				# update displays
				for key,displayable in self.queue.items():
					try:	
						self.displays[key] = self.display(displayable, self.displays.get(key))
					except:
						print('\ntried to display', object.__repr__(displayable))
						traceback.print_exc()
				self.touched = True
				self.queue.clear()
		
		if self.touched:
			self.restack()
			
	def restack(self):
		''' update the rendering calls stack from the current scene's displays.
			this is called automatically on `dequeue()`
		'''
		# recreate stacks
		for stack in self.stacks.values():
			stack.clear()
		for key,display in self.displays.items():
			for frame in display.stack(self):
				if len(frame) != 4:
					raise ValueError('wrong frame format in the stack from {}\n\t got {}'.format(display, frame))
				sub,target,priority,func = frame
				if target not in self.stacks:	self.stacks[target] = []
				stack = self.stacks[target]
				stack.append(((key,*sub), priority, func))
		# sort the stack using the specified priorities
		for stack in self.stacks.values():
			stack.sort(key=itemgetter(1))
		self.touched = False
	
	def render(self, view):
		''' render to the view targets. 
			
			This must be called by the view widget, once the the opengl context is set.
		'''
		empty = ()
		with self.ctx:
			# apply changes that need opengl runtime
			self.dequeue()
			# render everything
			for target, frame, setup in view.targets:
				view.target = frame
				frame.use()
				setup()
				for key, priority, func in self.stacks.get(target,empty):
					func(view)
	
	def box(self):
		''' computes the boundingbox of the scene, with the current object poses '''
		box = Box(center=fvec3(0), width=fvec3(-inf))
		for display in self.displays.values():
			box.union_update(display.box.transform(display.world))
		return box
	
	def ressource(self, name, func=None):
		''' get a ressource loaded or load it using the function func.
			If func is not provided, an error is raised
		'''
		if name in self.ressources:	
			return self.ressources[name]
		elif callable(func):
			with self.ctx as ctx:  # set the scene context as current opengl context
				res = func(self)
				self.ressources[name] = res
				return res
		else:
			raise KeyError("ressource {} doesn't exist or is not loaded".format(repr(name)))
					
	def display(self, obj, former=None):
		''' create a display for the given object for the current scene.
		
			this is the actual function converting objects into displays.
			you don't need to call this method if you just want to add an object to the scene, use add() instead
		'''
		if former and former.update(self, obj):
			return former
		if type(obj) in overrides:
			disp = overrides[type(obj)](self, obj)
		elif hasattr(obj, 'display'):
			if isinstance(obj.display, type):
				disp = obj.display(self, obj)
			elif callable(obj.display):
				disp = obj.display(self)
			else:
				raise TypeError("member 'display' must be a method or a type, on {}".format(type(obj).__name__))
		else:
			raise TypeError('type {} is not displayable'.format(type(obj).__name__))
		
		if not isinstance(disp, Display):
			raise TypeError('the display for {} is not a subclass of Display: {}'.format(type(obj).__name__, type(disp)))
		return disp
	

def displayable(obj):
	''' return True if the given object has the matching signature to be added to a Scene '''
	return type(obj) in overrides or hasattr(obj, 'display') and callable(obj.display) and not isinstance(obj, type)


class Step(Display):
	''' simple display holding a rendering stack step 
	
		`Step(target, priority, callable)`
	'''
	__slots__ = 'step',
	def __init__(self, target, priority, callable):	self.step = ((), target, priority, callable)
	def __repr__(self):		return '{}({}, {}, {})'.format(type(self).__name__, self.step[1], self.step[2], self.step[3])
	def stack(self, scene):		return self.step,

class Displayable:
	''' simple displayable initializeing the given Display class with arguments 
		
		at the display creation time, it will simply execute `build(*args, **kwargs)`
	'''
	__slots__ = 'build', 'args', 'kwargs'
	def __init__(self, build, *args, **kwargs):
		self.args, self.kwargs = args, kwargs
		self.build = build
	def __repr__(self):
		return '{}({}, {}, {})'.format(type(self).__name__, repr(self.args[1:-1]), repr(self.kwargs)[1:-1])
	def display(self, scene):
		return self.build(scene, *self.args, **self.kwargs)


def writeproperty(func):
	''' decorator to create a property that has only an action on variable write '''
	fieldname = '_'+func.__name__
	def getter(self):	return getattr(self, fieldname)
	def setter(self, value):
		setattr(self, fieldname, value)
		func(self, value)
	return property(getter, setter, doc=func.__doc__)

class Group(Display):
	''' a group is like a subscene '''
	def __init__(self, scene, objs:'dict/list'=None, pose=1):
		self._world = fmat4(1)
		self._pose = fmat4(pose)
		self.displays = {}
		if objs:	self.dequeue(scene, objs)
	
	def __getitem__(self, key):
		return self.displays[key]
	def __iter__(self):
		return iter(self.displays.values())
		
	def update(self, scene, objs):
		if isinstance(objs, dict):		objs = objs
		elif hasattr(objs, 'keys'):		objs = dict(objs)
		elif hasattr(objs, '__iter__'):	objs = dict(enumerate(objs))
		else:
			return False
		
		# update displays
		sub = self._world * self._pose
		with scene.ctx:
			scene.ctx.finish()
			for key, obj in objs.items():
				if not displayable(obj):	continue
				try:
					self.displays[key] = disp = scene.display(obj, self.displays.get(key))
					disp.world = sub
				except:
					print('tried to display', object.__repr__(obj))
					traceback.print_exc()
			for key in self.displays.keys() - objs.keys():
				del self.displays[key]
		scene.touch()
		return True
	dequeue = update
	
	def stack(self, scene):
		for key,display in self.displays.items():
			for sub,target,priority,func in display.stack(scene):
				yield ((key, *sub), target, priority, func)
	
	@writeproperty
	def pose(self, pose):
		''' pose of the group relatively to its parents '''
		sub = self._world * self._pose
		for display in self.displays.values():
			display.world = sub
			
	@writeproperty
	def world(self, world):
		''' update children's world matrix applying the current pose in addition to world '''
		sub = self._world * self._pose
		for display in self.displays.values():
			display.world = sub
			
	@property
	def box(self):
		''' computes the boundingbox of the scene, with the current object poses '''
		box = Box(center=fvec3(0), width=fvec3(-inf))
		for display in self.displays.values():
			box.union_update(display.box)
		return box.transform(self._pose)


# dictionnary to store procedures to override default object displays
overrides = {
	list: Group,
	dict: Group,
	}


class ViewCommon:
	''' Common base for Qt's View rendering and Offscreen rendering. It provides common methods to render and interact with a view.
		
		You should always use one of its subclass.
	'''
	def __init__(self, scene, projection=None, navigation=None):
		# interaction methods
		self.projection = projection or globals()[settings.scene['projection']]()
		self.navigation = navigation or globals()[settings.controls['navigation']]()

		# render parameters
		self.scene = scene if isinstance(scene, Scene) else Scene(scene)
		self.uniforms = {'proj':fmat4(1), 'view':fmat4(1), 'projview':fmat4(1)}	# last frame rendering constants
		self.targets = []
		self.steps = []
		self.step = 0
		self.stepi = 0

		# dump targets
		self.map_depth = None
		self.map_idents = None
		self.fresh = set()	# set of refreshed internal variables since the last render

	# -- internal frame system --

	def refreshmaps(self):
		''' load the rendered frames from the GPU to the CPU

			- When a picture is used to GPU rendering it's called 'frame'
			- When it is dumped to the RAM we call it 'map' in this library
		'''
		if 'fb_ident' not in self.fresh:
			self.makeCurrent()	# set the scene context as current opengl context
			with self.scene.ctx as ctx:
				#ctx.finish()
				self.fb_ident.read_into(self.map_ident, viewport=self.fb_ident.viewport, components=2)
				self.fb_ident.read_into(self.map_depth, viewport=self.fb_ident.viewport, components=1, attachment=-1, dtype='f4')
			self.fresh.add('fb_ident')
			#from PIL import Image
			#Image.fromarray(self.map_ident*16, 'I;16').show()

	def render(self):
		# prepare the view uniforms
		w, h = self.fb_screen.size
		self.uniforms['view'] = view = self.navigation.matrix()
		self.uniforms['proj'] = proj = self.projection.matrix(w/h, self.navigation.distance)
		self.uniforms['projview'] = proj * view
		self.fresh.clear()

		# call the render stack
		self.scene.render(self)

	def identstep(self, nidents):
		''' updates the amount of rendered idents and return the start ident for the calling rendering pass 
			method to call during a renderstep
		'''
		s = self.step
		self.step += nidents
		self.steps[self.stepi] = self.step-1
		self.stepi += 1
		return s

	def setup_ident(self):
		# steps for fast fast search of displays with the idents
		self.stepi = 0
		self.step = 1
		if 'ident' in self.scene.stacks and len(self.scene.stacks['ident']) != len(self.steps):
			self.steps = [0] * len(self.scene.stacks['ident'])
		# ident rendering setup
		ctx = self.scene.ctx
		ctx.multisample = False
		ctx.enable_only(mgl.DEPTH_TEST)
		ctx.blend_func = mgl.ONE, mgl.ZERO
		ctx.blend_equation = mgl.FUNC_ADD
		self.target.clear(0)

	def setup_screen(self):
		# screen rendering setup
		ctx = self.scene.ctx
		ctx.multisample = True
		ctx.enable_only(mgl.BLEND | mgl.DEPTH_TEST)
		ctx.blend_func = mgl.SRC_ALPHA, mgl.ONE_MINUS_SRC_ALPHA
		ctx.blend_equation = mgl.FUNC_ADD
		
		background = settings.display['background_color']
		if len(background) == 3:
			self.target.clear(*background, alpha=1)
		elif len(background) == 4:
			self.target.clear(*background)
		else:
			raise ValueError(f"background_color must be a RGB or RGBA tuple, currently {background}")

	def preload(self):
		''' internal method to load common ressources '''
		ctx, ressources = self.scene.ctx, self.scene.ressources
		ressources['shader_ident'] = ctx.program(
					vertex_shader=open(ressourcedir+'/shaders/object-ident.vert').read(),
					fragment_shader=open(ressourcedir+'/shaders/ident.frag').read(),
					)

		ressources['shader_subident'] = ctx.program(
					vertex_shader=open(ressourcedir+'/shaders/object-item-ident.vert').read(),
					fragment_shader=open(ressourcedir+'/shaders/ident.frag').read(),
					)

	# -- methods to deal with the view --

	def somenear(self, point: ivec2, radius=None) -> ivec2:
		''' return the closest coordinate to coords, (within the given radius) for which there is an object at
			So if objnear is returing something, objat and ptat will return something at the returned point
		'''
		if radius is None:
			radius = settings.controls['snap_dist']
		self.refreshmaps()
		for x,y in snailaround(point, (self.map_ident.shape[1], self.map_ident.shape[0]), radius):
			ident = int(self.map_ident[-y, x])
			if ident:
				return uvec2(x,y)

	def ptat(self, point: ivec2) -> fvec3:
		''' return the point of the rendered surfaces that match the given window coordinates '''
		self.refreshmaps()
		viewport = self.fb_ident.viewport
		depthred = float(self.map_depth[-point.y,point.x])
		x =  (point.x/viewport[2] *2 -1)
		y = -(point.y/viewport[3] *2 -1)

		if depthred == 1.0:
			return None
		else:
			view = self.uniforms['view']
			proj = self.uniforms['proj']
			a,b = proj[2][2], proj[3][2]
			depth = b/(depthred + a) * 0.5	# TODO get the true depth  (can't get why there is a strange factor ... opengl trick)
			#near, far = self.projection.limits  or settings.display['view_limits']
			#depth = 2 * near / (far + near - depthred * (far - near))
			#print('depth', depth, depthred)
			return vec3(fvec3(affineInverse(view) * fvec4(
						depth * x /proj[0][0],
						depth * y /proj[1][1],
						-depth,
						1)))

	def ptfrom(self, point: ivec2, center: fvec3) -> fvec3:
		''' 3D point below the cursor in the plane orthogonal to the sight, with center as origin '''
		view = self.uniforms['view']
		proj = self.uniforms['proj']
		viewport = self.fb_ident.viewport
		x =  (point.x/viewport[2] *2 -1)
		y = -(point.y/viewport[3] *2 -1)
		depth = (view * fvec4(fvec3(center),1))[2]
		return vec3(fvec3(affineInverse(view) * fvec4(
					-depth * x /proj[0][0],
					-depth * y /proj[1][1],
					depth,
					1)))

	def itemat(self, point: ivec2) -> 'key':
		''' return the key path of the object at the given screen position (widget relative).
			If no object is at this exact location, None is returned
		'''
		self.refreshmaps()
		point = uvec2(point)
		ident = int(self.map_ident[-point.y, point.x])
		if ident and 'ident' in self.scene.stacks:
			rdri = bisect(self.steps, ident)
			if rdri == len(self.steps):
				print('internal error: object ident points out of idents list')
			while rdri > 0 and self.steps[rdri-1] == ident:	rdri -= 1
			if rdri > 0:	subi = ident - self.steps[rdri-1] - 1
			else:			subi = ident - 1
			
			if rdri >= len(self.scene.stacks['ident']):
				print('wrong identification index', ident, self.scene.stacks['ident'][-1])
				nprint(self.scene.stacks['ident'])
				return
			
			return (*self.scene.stacks['ident'][rdri][0], subi)

	# -- view stuff --

	def look(self, position: fvec3=None):
		''' Make the scene navigation look at the position.
			This is changing the camera direction, center and distance.
		'''
		if not position:	position = self.scene.box().center
		dir = position - fvec3(affineInverse(self.navigation.matrix())[3])
		if not dot(dir,dir) > 1e-6 or not isfinite(position):	return

		if isinstance(self.navigation, Turntable):
			self.navigation.yaw = atan2(dir.x, dir.y)
			self.navigation.pitch = -atan2(dir.z, length(dir.xy))
			self.navigation.center = position
			self.navigation.distance = length(dir)
		elif isinstance(self.navigation, Orbit):
			focal = self.orient * fvec3(0,0,1)
			self.navigation.orient = quat(dir, focal) * self.navigation.orient
			self.navigation.center = position
			self.navigation.distance = length(dir)
		else:
			raise TypeError("navigation {} is not supported by 'look'".format(type(self.navigation)))

	def adjust(self, box:Box=None):
		''' Make the navigation camera large enough to get the given box in .
			This is changing the zoom level
		'''
		if not box:	box = self.scene.box()
		if box.isempty():	return

		# get the most distant point to the focal axis
		invview = affineInverse(self.navigation.matrix())
		camera, look = fvec3(invview[3]), fvec3(invview[2])
		dist = length(noproject(box.center-camera, look)) + max(glm.abs(box.width))/2 * 1.1
		if not dist > 1e-6:	return

		# adjust navigation distance
		if isinstance(self.projection, Perspective):
			self.navigation.distance = dist / tan(self.projection.fov/2)
		elif isinstance(self.projection, Orthographic):
			self.navigation.distance = dist / self.projection.size
		else:
			raise TypeError('projection {} not supported'.format(type(self.projection)))

	def center(self, center: fvec3=None):
		''' Relocate the navigation to the given position .
			This is translating the camera.
		'''
		if not center:	center = self.scene.box().center
		if not isfinite(center):	return

		self.navigation.center = center


class Offscreen(ViewCommon):
	''' object allowing to perform offscreen rendering, navigate and get informations from screen as for a normal window 
	'''
	def __init__(self, scene, size=uvec2(400,400), projection=None, navigation=None):
		global global_context
		
		super().__init__(scene, projection=projection, navigation=navigation)
		
		if global_context:
			self.scene.ctx = global_context
		else:
			self.scene.ctx = global_context = mgl.create_standalone_context(requires=opengl_version)
		self.scene.ctx.line_width = settings.display["line_width"]

		self.init(size)
		self.preload()

	def init(self, size):
		w, h = size

		ctx = self.scene.ctx
		assert ctx, 'context is not initialized'

		# self.fb_frame is already created and sized by Qt
		self.fb_screen = ctx.simple_framebuffer(size)
		self.fb_ident = ctx.simple_framebuffer(size, components=3, dtype='f1')
		self.targets = [ ('screen', self.fb_screen, self.setup_screen),
						 ('ident', self.fb_ident, self.setup_ident)]
		self.map_ident = np.empty((h,w), dtype='u2')
		self.map_depth = np.empty((h,w), dtype='f4')
		
	@property
	def size(self):		
		return self.fb_screen.size
		
	def width(self):
		return self.fb_screen.size[0]
		
	def height(self):
		return self.fb_screen.size[1]
	
	def resize(self, size):
		if size != self.fb_screen.size:
			self.ctx.finish()
			self.init(size)

	def render(self):
		super().render()
		return Image.frombytes('RGBA', tuple(self.size), self.fb_screen.read(components=4), 'raw', 'RGBA', 0, -1)


class View(ViewCommon, QOpenGLWidget):
	''' Qt widget to render and interact with displayable objects
		it holds a scene as renderpipeline

		Attributes:

			scene:        the `Scene` object displayed
			projection:   `Perspective` or `Orthographic`
			navigation:   `Orbit` or `Turntable`
			tool:         list of callables in priority order to receive events

			targets:     render targets matching those requested in `scene.stacks`
			uniforms:    parameters for rendering, used in shaders
	'''
	def __init__(self, scene, projection=None, navigation=None, parent=None):
		# super init
		super(QOpenGLWidget, self).__init__(parent)
		fmt = QSurfaceFormat()
		fmt.setVersion(*opengl_version)
		fmt.setProfile(QSurfaceFormat.CoreProfile)
		fmt.setSamples(4)
		self.setFormat(fmt)
		
		# ugly trick to receive interaction events in a different function than QOpenGLWidget.event (that one is locking the GIL during the whole rendering, killing any possibility of having a computing thread aside)
		# that event reception should be in the current widget ...
		self.handler = GhostWidget(self)
		self.handler.setFocusPolicy(Qt.StrongFocus)
		self.handler.setAttribute(Qt.WA_AcceptTouchEvents, True)
		self.setFocusProxy(self.handler)

		ViewCommon.__init__(self, scene, projection=projection, navigation=navigation)
		self.tool = [Tool(self.navigation.tool, self)] # tool stack, the last tool is used for input events, until it is removed 

	def init(self):
		w, h = self.width(), self.height()

		ctx = self.scene.ctx
		assert ctx, 'context is not initialized'

		# self.fb_screen is already created and sized by Qt
		self.fb_screen = ctx.detect_framebuffer(self.defaultFramebufferObject())
		self.fb_ident = ctx.simple_framebuffer((w, h), components=3, dtype='f1')
		self.targets = [ ('screen', self.fb_screen, self.setup_screen),
						 ('ident', self.fb_ident, self.setup_ident)]
		self.map_ident = np.empty((h,w), dtype='u2')
		self.map_depth = np.empty((h,w), dtype='f4')

	def render(self):
		# set the opengl current context from Qt (doing it only from moderngl interferes with Qt)
		self.makeCurrent()
		ViewCommon.render(self)

	
	# -- view stuff --

	def look(self, position: fvec3=None):
		ViewCommon.look(self, position)
		self.update()

	def adjust(self, box:Box=None):
		ViewCommon.adjust(self, box)
		self.update()

	def center(self, center: fvec3=None):
		ViewCommon.center(self, center)
		self.update()
	
	def somenear(self, point: QPoint, radius=None) -> QPoint:
		some = ViewCommon.somenear(self, qt_2_glm(point), radius)
		if some:
			return glm_to_qt(some)

	def ptat(self, point: QPoint) -> fvec3:
		return ViewCommon.ptat(self, qt_2_glm(point))

	def ptfrom(self, point: QPoint, center: fvec3) -> fvec3:
		return ViewCommon.ptfrom(self, qt_2_glm(point), center)

	def itemat(self, point: QPoint) -> 'key':
		return ViewCommon.itemat(self, qt_2_glm(point))
	

	# -- event system --

	def inputEvent(self, evt):
		''' Default handler for every input event (mouse move, press, release, keyboard, ...)
			When the event is not accepted, the usual matching Qt handlers are used (mousePressEvent, KeyPressEvent, etc).

			This function can be overwritten to change the view widget behavior.
		'''
		# send the event to the current tools using the view
		if self.tool:
			for tool in reversed(self.tool):
				tool(evt)
				if evt.isAccepted():	return

		# send the event to the scene objects, descending the item tree
		if isinstance(evt, QMouseEvent) and evt.type() in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease, QEvent.MouseButtonDblClick, QEvent.MouseMove):
			pos = self.somenear(evt.pos())
			if pos:
				key = self.itemat(pos)
				if key:
					self.control(key, evt)
					if evt.isAccepted():	return

			# if clicks are not accepted, then some following keyboard events may not come to the widget
			# NOTE this also discarding the ability to move the window from empty areas
			if evt.type() == QEvent.MouseButtonPress:
				evt.accept()

	def control(self, key, evt):
		''' transmit a control event successively to all the displays matching the key path stages.
			At each level, if the event is not accepted, it transmits to sub items

			This function can be overwritten to change the interaction with the scene objects.
		'''
		disp = self.scene.displays
		stack = []
		for i in range(1,len(key)):
			disp = disp[key[i-1]]
			disp.control(self, key[:i], key[i:], evt)
			if evt.isAccepted(): return
			stack.append(disp)

		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
			disp = stack[-1]
			# select what is under cursor
			if type(disp).__name__ in ('SolidDisplay', 'WebDisplay'):
				disp.vertices.selectsub(key[-1])
				disp.selected = any(disp.vertices.flags & 0x1)
			else:
				disp.selected = not disp.selected
			# make sure that a display is selected if one of its sub displays is
			for disp in reversed(stack):
				if hasattr(disp, '__iter__'):
					disp.selected = any(sub.selected	for sub in disp)
			self.update()

	# -- Qt things --

	def initializeGL(self):
		# retrieve global shared context if available
		global global_context
		
		if QApplication.testAttribute(Qt.AA_ShareOpenGLContexts):
			if not global_context:
				global_context = mgl.create_context()
			self.scene.ctx = global_context
		# or create a context
		else:
			self.scene.ctx = mgl.create_context()
		self.init()
		self.preload()

	def paintGL(self):
		self.makeCurrent()
		self.render()

	def resizeEvent(self, evt):
		QOpenGLWidget.resizeEvent(self, evt)
		self.handler.resize(self.size())
		self.init()
		self.update()

	def changeEvent(self, evt):
		# detect theme change
		if evt.type() == QEvent.PaletteChange and settings.display['system_theme']:
			settings.use_qt_colors()
		return QOpenGLWidget.changeEvent(self, evt)

class GhostWidget(QWidget):
	def __init__(self, parent):
		super().__init__(parent)
		
	def event(self, evt):
		if isinstance(evt, QInputEvent):
			# set the opengl current context from Qt (doing it only from moderngl interferes with Qt)
			#self.makeCurrent()
			evt.ignore()
			self.parent().inputEvent(evt)
			if evt.isAccepted():	return True
		elif isinstance(evt, QFocusEvent):
			self.parent().event(evt)
		return super().event(evt)


def snail(radius):
	''' generator of coordinates snailing around 0,0 '''
	x = 0
	y = 0
	for r in range(radius):
		for x in range(-r,r):		yield ivec2(x,-r)
		for y in range(-r,r):		yield ivec2(r, y)
		for x in reversed(range(-r,r)):	yield ivec2(x, r)
		for y in reversed(range(-r,r)):	yield ivec2(-r,y)

def snailaround(pt, box, radius):
	''' generator of coordinates snailing around pt, coordinates that goes out of the box are skipped '''
	cx,cy = pt
	mx,my = box
	for rx,ry in snail(radius):
		x,y = cx+rx, cy+ry
		if 0 <= x and x < mx and 0 <= y and y < my:
			yield ivec2(x,y)

def glm_to_qt(p): 	return QPoint(p.x, p.y)
def qt_2_glm(p):	return ivec2(p.x(), p.y())

'''
		-- generators helpers --
'''

class Generated(object):
	''' generator that has a returned value '''
	__slots__ = 'generator', 'value'
	def __init__(self, generator):	self.generator = generator
	def __iter__(self):				self.value = yield from self.generator

class Dispatcher(object):
	''' iterable object that holds a generator built by passing self as first argument
		it allows the generator code to dispatch references to self.
		NOTE:  at contrary to current generators, the code before the first yield is called at initialization
	'''
	__slots__ = 'generator', 'value'
	def __init__(self, func=None, *args, **kwargs):
		self.func = func
		self.generator = self._run(func, *args, **kwargs)
		# run the generator until the first yield
		next(self.generator, None)
	def _run(self, func, *args, **kwargs):
		self.value = yield from func(self, *args, **kwargs)
		
	def __repr__(self):
		return '<{} on {}>'.format(type(self).__name__, self.func)
		
	def send(self, value):	return self.generator.send(value)
	def __iter__(self):		return self.generator
	def __next__(self):		return next(self.generator)

class Tool(Dispatcher):
	''' generator wrapping an yielding function, that unregisters from view.tool once the generator is over '''
	def _run(self, func, *args, **kwargs):
		try:	
			self.value = yield from func(self, *args, **kwargs)
		except StopTool:
			pass
		try:	
			args[0].tool.remove(self)
		except ValueError:	
			pass
	
	def __call__(self, evt):
		try:	return self.send(evt)
		except StopIteration:	pass
		
	def stop(self):
		if self.generator:
			try:	self.generator.throw(StopTool())
			except StopTool:	pass
			except StopIteration:	pass
			self.generator = None
	def __del__(self):
		self.stop()
	
class StopTool(Exception):
	''' used to stop a tool execution '''
	pass


# temporary examples
if False:

	def tutu(self, main):
		evt = yield
		gnagna
		scene.tool = self.send
		budu.iterator = self

	Tool(tutu, main)
	
