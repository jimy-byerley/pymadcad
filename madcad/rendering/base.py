from __future__ import annotations
import traceback
from weakref import WeakValueDictionary
from dataclasses import dataclass
from copy import deepcopy
from typing import Self, Callable, Iterable
from operator import attrgetter

import moderngl as mgl

from .. import settings
from ..mathutils import boundingbox
from .utils import writeproperty


# minimum opengl version required by the rendering pipeline
opengl_version = (4,3)
# shared open gl context, None if not yet initialized
global_context = None
# helper constant to avoid reinstantiating empty tuples
empty = ()



class Scene:
	''' rendering pipeline for madcad displayable objects 
	
		This class is gui-agnostic, it only relies on OpenGL, and the context has to be created by the user.
		
		When an object is added to the scene, a Display is not immediately created for it, the object is put into the queue and the Display is created at the next render.
		If the object is removed from the scene before the next render, it is dequeued.
	'''
	root: Display
	''' the root display of the scene, usually a `Group` '''
	stacks: dict[str, list[Frame]]
	''' a list of the display callbacks for each target frame buffers '''
	overrides: dict[type, Callable] = {}
	''' attribute allowing to alter how the displayable are transformed into displays
	
		- the class attribute stores default overrides
		- the instance attribute stores the effective overrides for the scene
	'''
	options: dict
	''' scene rendering parameters, deriving from `settings.scene` '''
	shared: dict
	''' strong references to the resources we don't want to free when no display is using them '''
	context: mgl.Context
	''' opengl context used in this scene's displays. It is assumed to have `gc_mode = auto` '''
	selection: set[Display]
	''' current set of selected displays (readonly, use the scene methods to alter the selection) '''
	hover: Display = None
	''' current display being hovered (readwrite) '''
	
	def __init__(self, src:object, options:dict=None, context:mgl.Context=None, overrides:dict=None):
		'''
		Args:
			src:        the root object of the scene, it must be a displayable and is usually a `dict`
			options:    scene rendering parameters, overriding `settings.scene`
		'''
		self.root = None
		self.overrides = deepcopy(Scene.overrides)
		if overrides:
			self.overrides.update(overrides)
		self.stacks = {}
		self.options = deepcopy(settings.scene)
		self.shared = {}
		self.selection = set()
		self._hover = None
		self._need_restack = False
		if options:   self.options.update(options)
		
		global global_context
		if context:
			self.context = context
		elif global_context:
			self.context = global_context
		else:
			self.context = global_context = mgl.create_standalone_context(requires=opengl_version)
			self.context.gc_mode = 'auto'
			self.context.line_width = settings.display["line_width"]
		
		self.update(src)
	
	def update(self, src):
		''' update the root display of the scene '''
		self.root = self.display(src, self.root)
		self.root.key = ()
		self.touch()
		
	def touch(self):
		''' require a restack before next rendering '''
		self._need_restack = True
		
	def display(self, obj, former:Display=None) -> Display:
		''' Create a display for the given object for the current scene.
		
			This is the actual function converting objects into displays.
			You don't need to call this method if you just want to add an object to the scene, use add() instead
		'''
		if former and former.update(self, obj):
			return former
		if type(obj) in self.overrides:
			disp = self.overrides[type(obj)](self, obj)
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
		
	def displayable(self, obj) -> bool:
		''' return whether the given object could be displayed in the scene '''
		if type(obj) in self.overrides:
			return True
		elif hasattr(obj, 'display'):
			if isinstance(obj.display, type):
				return True
			elif callable(obj.display):
				return True
		return False
	
	def share(self, key, generator=None, hook=False):
		''' Get a shared resource loaded or load it using the function func.
			If func is not provided, an error is raised
		'''
		if key in self.shared:	
			return self.shared[key]
		elif callable(generator):
			res = generator(self)
			self.shared[key] = res
			if hook:
				self.hooks[key] = res
			return res
		else:
			raise KeyError("resource {} doesn't exist or is not loaded".format(repr(name)))
	
	def prepare(self):
		''' convert all displayable to displays and bufferize everything for comming renderings '''
		if self._need_restack:
			with self.context:
				for stack in self.stacks.values():
					stack.clear()
				for step in self.root.stack(self):
					if isinstance(step, tuple):
						step = Step(*step)
					elif not isinstance(step, Step):
						raise TypeError('stack step must be of type Step, not {}'.format(step))
					if step.target not in self.stacks:	
						self.stacks[step.target] = []
					stack = self.stacks[step.target]
					stack.append(step)
				# sort the stack using the specified priorities
				for stack in self.stacks.values():
					stack.sort(key=attrgetter('priority'))
				self._need_restack = False
	
	def render(self, view):
		''' Render to the view targets. 
			
			This must be called by the view widget, once the OpenGL context is set.
		'''
		with self.context:
			self.prepare()
			# render everything
			for target, frame, setup in view.targets:
				view.target = frame
				frame.use()
				setup()
				for step in self.stacks.get(target,empty):
					step.render(view)
	
	def selection_add(self, display:Display):
		''' select the given display '''
		display.selected = True
		self.selection.add(display)
	
	def selection_discard(self, display:Display):
		''' deselect the given display '''
		display.selected = False
		self.selection.discard(display)
		
	def selection_clear(self):
		''' deselect all previously selected displays '''
		for display in self.selection:
			display.selected = False
	
	def _get_hover(self):
		return self._hover
	def _set_hover(self, hover):
		if hover is self.hover:
			return
		if self.hover:
			self.hover.hovered = False
		self.hover = hover
		if self.hover:
			self.hover.hovered = True
	hover = property(_get_hover, _set_hover)

@dataclass
class Step:
	''' describes a rendering step for a display '''
	display: Display
	''' the display instance responsible of this callback '''
	target: str
	''' the name of the render target in the view that will be rendered (see View) '''
	priority: float
	''' a float that is used to insert the callable at the proper place in the rendering stack '''
	render: Callable
	''' a function that renders, signature is  `func(view)` '''


class Display:
	''' base class for objects displayed in a scene '''
	key: str = None
	''' display path in the scene tree, set when Scene.restack is called '''
	world: fmat3|fmat4
	''' (optional) matrix from local space to scene space '''
	box: Box
	''' (optional) boudingbox of the display in local space '''
	selected: bool|set
	''' (optional) whether the display has been selected by the user '''
	hovered: bool|set
	''' (optional) whether the display is hovered by a pointing device and might be selected '''
	
	def display(scene) -> Self:
		''' Displays are obviously displayable as themselves, this should be overriden '''
		return self
	
	def update(scene, src) -> bool:
		''' Update the current displays internal datas with the given displayable .
			
			If the display cannot be upgraded, it must return False to be replaced by a fresh new display created from the displayable
		'''
		return False
	
	def stack() -> Iterable[Step]:
		''' Rendering functions to insert in the renderpipeline.
		
			The expected result can be any iterable providing tuples `(key, target, priority, callable)` such as:
			
			The view contains the uniforms, rendering targets and the scene for common resources
		'''
		return empty
	
	def control(self, view, key, sub, evt: 'QEvent'):
		''' Handle input events occuring on the area of this display (or of one of its subdisplay).
			For subdisplay events, the parents control functions are called first, and the sub display controls are called only if the event is not accepted by parents
			
			Parameters:
				key:    the key path for the current display
				sub:    the key path for the subdisplay
				evt:    the Qt event (see Qt doc)
		'''
		pass
	
	def __iter__(self) -> iter:
		''' yield child displays '''
		return iter(empty)
	
	def __getitem__(self, key) -> 'display':
		''' Get a child display by its key in this display (like in a scene) '''
		raise IndexError('{} has no sub displays'.format(type(self).__name__))

	def share(self, scene, name=None):
		if not name:
			name = type(self)
		self._shared = scene.shared(self.shared)
		vars(self).update(self._shared.vars)


class Group(Display):
	''' display holding multiple displays associated with a key, just like a dictionnary '''
	displays: dict
	''' dictionnary of displays currently displayed in the group '''
	
	def __init__(self, scene:Scene, src:dict|list):
		self.displays = {}
		self._pending = None
		self._world = 1
		self.update(src)
		
	@property
	def box(self):
		return boundingbox(display.box   for display in self.displays.values())
	
	@writeproperty
	def world(self):
		''' matrix from local space to scene space
		
			a group propagates this value to its children
		'''
		for display in self.displays.values():
			display.world = self.world
	
	def update(self, src:dict):
		''' update all child displays with the displayable present in the given dictionnary
			
			former children that do not match keys in the given dictionnary are dropped
			
			the new content will not be immediately available in `self.displays` because they will only be buffered at next rendering
		'''
		if isinstance(src, list):
			src = dict(enumerate(src))
		if not isinstance(src, dict):
			return False
		self._pending = src
	
	def stack(self, scene):
		# update with the pending data
		if self._pending is not None:
			removed = [key  for key in self.displays if key not in self.pending]
			for key in removed:
				del self.displays[key]
			for key, displayable in self._pending.items():
				try:
					self.displays[key] = scene.display(displayable, self.displays.get(key))
				except:
					print('\ntried to display', object.__repr__(displayable))
					traceback.print_exc()
			self._pending = None
			scene.touch()
		# stack children
		for key, display in self.displays.items():
			display.world = self.world
			display.key = (*self.key, key)
			yield from display.stack(scene)
	
	def __iter__(self):
		return iter(self.displays.values())
	
	def __getitem__(self, key):
		return self.displays[key]


Scene.overrides[dict] = Group
Scene.overrides[list] = Group


class Displayable:
	''' Simple displayable initializing the given Display class with arguments 
		
		At the display creation time, it will simply execute `build(*args, **kwargs)`
	'''
	__slots__ = 'build', 'args', 'kwargs'
	def __init__(self, build, *args, **kwargs):
		self.args, self.kwargs = args, kwargs
		self.build = build
	def __repr__(self):
		return '{}({}, {})'.format(
			type(self).__name__, 
			','.join(repr(arg) for arg in self.args), 
			','.join(key+'='+repr(arg)  for key, arg in self.kwargs.items())
			)
	def display(self, scene):
		return self.build(scene, *self.args, **self.kwargs)
