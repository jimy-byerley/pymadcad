# This file is part of pymadcad,  distributed under license LGPL v3
from __future__ import annotations
import traceback
from weakref import WeakValueDictionary
from dataclasses import dataclass
from copy import deepcopy, copy
from operator import attrgetter

import moderngl as mgl

from .. import settings
from ..mathutils import fvec3, inf
from ..box import Box, boundingbox
from .utils import writeproperty

__all__ = ['Scene', 'Step', 'Display', 'Group', 'Displayable']


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
	touched: bool
	''' trigger for restacking render steps '''
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
	hover: Display|None = None
	''' current display being hovered (readwrite) '''
	
	def __init__(self, src:object=None, options:dict=None, context:mgl.Context=None, overrides:dict=None):
		'''
		Args:
			src:        the root object of the scene, it must be a displayable and is usually a `dict`
			options:    scene rendering parameters, overriding `settings.scene`
			context:    the openGL context to use to send rendering instructions to the GPU
		'''
		self.root = None
		self.overrides = deepcopy(Scene.overrides)
		if overrides:
			self.overrides.update(overrides)
		self.stacks = {}
		self.options = deepcopy(settings.scene)
		self.shared = {}
		self.selection = set()
		self.touched = False
		self._hover = None
		self._memo = set()
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
	
	def update(self, src:object):
		''' update the root display of the scene '''
		if src is None:
			src = {}
		self.root = self.display(src, self.root)
		self.root.key = ()
		self.touch()
		
	def touch(self):
		''' inform the scene than its composition changed and that it needs to recompute its call stack '''
		self.touched = True
		
	def display(self, obj, former:Display=None) -> Display:
		''' Create a display for the given object for the current scene.
		
			This is the actual function converting objects into displays.
			You don't need to call this method if you just want to add an object to the scene, use add() instead
		'''
		# prevent reference-loop in groups (groups are taken from the execution env, so the user may not want to display it however we are trying to)
		ido = id(obj)
		assert ido not in self._memo, 'there should not be recursion loops in cascading displays'
		self._memo.add(ido)
		
		try:
			# no refresh if object has not changed
			if former and hasattr(former, 'source') and (former.source is obj or former.source == obj):
				disp = former
			# refresh cleverly if a previous displays proposes it
			elif former and former.update(self, obj):
				disp = former
			# create it with overrides
			elif type(obj) in self.overrides:
				disp = self.overrides[type(obj)](self, obj)
			# create it with the displayable protocol
			elif hasattr(obj, 'display'):
				if isinstance(obj, type):
					raise TypeError('types are not displayable')
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
				
		finally:
			self._memo.remove(ido)
		
		# keeping a reference of the source object may increas the RAM used but avoid to refresh displays when updating the scene with the same constant value
		if self.options['track_source']:
			disp.source = obj
		return disp
		
	def displayable(self, obj) -> bool:
		''' return whether the given object could be displayed in the scene '''
		if type(obj) in self.overrides:
			return True
		elif hasattr(obj, 'display'):
			if isinstance(obj, type):
				return False
			if isinstance(obj.display, type):
				return True
			elif callable(obj.display):
				return True
		return False
	
	def share(self, key, generator=None):
		''' Get a shared resource loaded or load it using the function func.
			If func is not provided, an error is raised
		'''
		if key in self.shared:	
			return self.shared[key]
		elif callable(generator):
			res = generator(self)
			self.shared[key] = res
			return res
		else:
			raise KeyError("resource {} doesn't exist or is not loaded".format(repr(key)))
	
	def prepare(self):
		''' convert all displayable to displays and bufferize everything for comming renderings '''
		if self.touched:
			self.touched = False
			self.active = None
			self.hover = None
			with self.context:
				for stack in self.stacks.values():
					stack.clear()
				for step in self.root.stack(self):
					if isinstance(step, tuple):
						step = Step(*step)
					elif not isinstance(step, Step):
						raise TypeError('stack step must be of type Step, not {}'.format(type(step)))
					if not isinstance(step.display, Display):
						raise TypeError('step.display must be a Display, not {}'.format(type(step.display)))
					if step.target not in self.stacks:	
						self.stacks[step.target] = []
					stack = self.stacks[step.target]
					stack.append(step)
				# sort the stack using the specified priorities
				for stack in self.stacks.values():
					stack.sort(key=attrgetter('priority'))
	
	def render(self, view):
		''' Render to the view targets. 
			
			This must be called by the view widget, once the OpenGL context is set.
		'''
		with self.context:
			# restack and bufferize
			try:
				self.prepare()
			except Exception:
				traceback.print_exc()
			# render everything
			for target, frame, setup in view.targets:
				try:
					view.target = frame
					frame.use()
					setup()
					for step in self.stacks.get(target,empty):
						step.render(view)
				except Exception:
					traceback.print_exc()
	
	def selection_add(self, display:Display, sub:int=None):
		''' select the given display '''
		if not hasattr(display, 'selected'):
			raise TypeError('{} is not selectable'.format(type(display).__name__))
		if isinstance(display.selected, set):
			display.selected.add(sub)
			display.selected = display.selected
			self.selection.add(display)
		else:
			display.selected = True
			self.selection.add(display)
		self.touch()
	
	def selection_remove(self, display:Display, sub:int=None):
		''' deselect the given display '''
		if not hasattr(display, 'selected'):
			raise TypeError('{} is not selectable'.format(type(display).__name__))
		if isinstance(display.selected, set):
			if sub is None:
				display.selected.clear()
			else:
				display.selected.discard(sub)
			display.selected = display.selected
			if not display.selected:
				self.selection.discard(display)
		else:
			display.selected = False
			self.selection.discard(display)
		self.touch()
		
	def selection_toggle(self, display:Display, sub:int=None):
		if not hasattr(display, 'selected'):
			raise TypeError('{} is not selectable'.format(type(display).__name__))
		if isinstance(display.selected, set):
			selected = sub in display.selected
		else:
			selected = display.selected
		if selected:
			self.selection_remove(display, sub)
		else:
			self.selection_add(display, sub)
	
	def selection_clear(self):
		''' deselect all previously selected displays '''
		for display in self.selection:
			if isinstance(display.selected, set):
				display.selected.clear()
				display.selected = display.selected
			else:
				display.selected = False
		if self.selection:
			self.touch()
			self.selection.clear()
	
	def _get_hover(self):
		return self._hover
	def _set_hover(self, hover):
		# hover can be set to a tuple to hover a subitem
		if isinstance(hover, tuple):
			hover, sub = hover
		else:
			sub = None
		# prevent unnecessary hover change to avoid triggering rebuffering of hover settings
		if hover is self._hover:
			if hover and isinstance(hover.hovered, set):
				if sub in hover.hovered:
					return
			else:
				return
		# clear previous hover in displays
		if self._hover:
			if isinstance(self._hover.hovered, set):
				self._hover.hovered.clear()
				self._hover.hovered = self._hover.hovered
			else:
				self._hover.hovered = False
		self._hover = hover
		# set new hover in displays
		if self._hover:
			if isinstance(self._hover.hovered, set):
				self._hover.hovered.add(sub)
				self._hover.hovered = self._hover.hovered
			else:
				self._hover.hovered = True
	hover = property(_get_hover, _set_hover)

	def __getitem__(self, key):
		return self.root[key]
	
	def __setitem__(self, key, value):
		self.root[key] = value
	
	def item(self, key:tuple):
		node = self.root
		for sub in key:
			node = node[sub]
	
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
	source: object
	''' source object of that display, set by `Scene.display` if this option is enabled '''
	world: fmat3|fmat4
	''' (optional) matrix from local space to scene space '''
	box: Box
	''' (optional) boudingbox of the display in local space '''
	selected: bool|set
	''' (optional) whether the display has been selected by the user 
	
		when display sets this value as a set, it designate selection of internal parts of the display designed by integers (like groups in a mesh). when the whole display is selected and not a subpart of it, `None` is present in the set.
		This doesn't prevent the scene from trying to assign booleans to it so it must be protected by a setter adding `None` to the set instead of assigning to `True`
	'''
	hovered: bool|set
	''' (optional) whether the display is hovered by a pointing device and might be selected 
	
		the same semantics of `selected` applies here
	'''
	
	def __init__(self, *args): 
		pass
	
	def display(self, scene:Scene) -> Self:
		''' Displays are obviously displayable as themselves, this should be overriden '''
		return self
	
	def update(self, scene:Scene, src:object) -> bool:
		''' Update the current displays internal datas with the given displayable .
			
			If the display cannot be upgraded, it must return False to be replaced by a fresh new display created from the displayable
		'''
		return False
	
	def stack(self, scene:Scene) -> Iterable[Step]:
		''' Rendering functions to insert in the renderpipeline.
		
			The expected result can be any iterable providing tuples `(key, target, priority, callable)` such as:
			
			The view contains the uniforms, rendering targets and the scene for common resources
		'''
		return empty
	
	def control(self, view, key:tuple, sub:tuple, evt:QEvent):
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
	
	def __getitem__(self, key) -> Display:
		''' Get a child display by its key in this display (like in a scene) '''
		raise IndexError('{} has no sub displays'.format(type(self).__name__))


class Group(Display):
	''' display holding multiple displays associated with a key, just like a dictionnary 
	
		`world`, `selected`, `hovered` properties are propagated to its children
	'''
	displays: dict
	''' dictionnary of displays currently displayed in the group '''
	
	def __init__(self, scene:Scene, src:dict|list=None, world=1):
		self.displays = {}
		self._pending = None
		self._world = world
		self._selected = False
		self._hovered = False
		if src:
			self.update(None, src)
		
	@property
	def box(self):
		return boundingbox(
			(getattr(display, 'box', None)   for display in self.displays.values()), 
			ignore=True, default=Box(fvec3(inf), fvec3(-inf)))
	
	@writeproperty
	def world(self):
		for display in self.displays.values():
			display.world = self.world
			
	@writeproperty
	def selected(self):
		for display in self.displays.values():
			if not hasattr(display, 'selected'):
				continue
			display.selected = self.selected
	
	@writeproperty
	def hovered(self):
		for display in self.displays.values():
			if not hasattr(display, 'hovered'):
				continue
			display.hovered = self.hovered
	
	def update(self, *args):
		''' 
			update(scene, src)
			update(src)
		
			update all child displays with the displayable present in the given dictionnary
			
			former children that do not match keys in the given dictionnary are dropped
			
		Note:
			the new content will not be immediately available in `self.displays` because they will only be buffered at next rendering. it makes this function thread safe and able to run without a reference to the scene
		'''
		if len(args) == 1:
			src, = args
		elif len(args) == 2:
			scene, src = args
		else:
			raise TypeError("expected 1 or 2 arguments, got {}".format(len(args)))
		if isinstance(src, list):
			src = dict(enumerate(src))
		if not isinstance(src, dict):
			return False
		self._pending = src
		return True
	
	def prepare(self, scene):
		# update with the pending data
		if self._pending is not None:
			removed = []
			for key in self.displays:
				if key not in self._pending or not scene.displayable(self._pending[key]):
					removed.append(key)
			for key in removed:
				del self.displays[key]
			for key, displayable in self._pending.items():
				if not scene.displayable(displayable):
					continue
				try:
					self.displays[key] = scene.display(displayable, self.displays.get(key))
				except:
					print('\ntried to display', object.__repr__(displayable))
					traceback.print_exc()
					continue
			self._pending = None
	
	def stack(self, scene):
		self.prepare(scene)
		for key, display in self.displays.items():
			display.world = self.world
			display.key = (*self.key, key)
			yield from display.stack(scene)
	
	def __iter__(self):
		return iter(self.displays.values())
	
	def __getitem__(self, key):
		return self.displays[key]

	def __setitem__(self, key, value):
		if self._pending is None:
			self._pending = copy(self.displays)
		self._pending[key] = value

Scene.overrides[dict] = Group
Scene.overrides[list] = Group
Scene.overrides[type(None)] = Display


class Displayable:
	''' Simple displayable initializing the given Display class with arguments 
		
		At the display creation time, it will simply execute `build(*args, **kwargs)`
	'''
	__slots__ = 'build', 'args', 'kwargs'
	def __init__(self, build, *args, **kwargs):
		self.args, self.kwargs = args, kwargs
		self.build = build
	def __eq__(self, other):
		return self is other or isinstance(other, Displayable) and (
			self.build == other.build
			and self.args == other.args
			and self.kwargs == other.kwargs
			)
	def __repr__(self):
		return '{}({}, {})'.format(
			type(self).__name__, 
			','.join(repr(arg) for arg in self.args), 
			','.join(key+'='+repr(arg)  for key, arg in self.kwargs.items())
			)
	def display(self, scene):
		return self.build(scene, *self.args, **self.kwargs)
