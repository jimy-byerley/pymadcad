# This file is part of pymadcad,  distributed under license LGPL v3
from __future__ import annotations
import random
from time import time
from pyglm.glm import smoothstep, fmat4, fvec3, slerp, mix, fquat, translate, fmat3, transpose, inverse, mat4
from math import inf

from ..box import boundingbox, Box
from ..mathutils import vec3
from ..rendering import Scene, Group, writeproperty, receiver
from . import Solid, explode_offsets, SolidBox
try:
	from ..qt import Qt, QEvent, QTimer
except ImportError:
	# it not found, assume the event-handlers will not be called
	pass

__all__ = ["ExplodableGroup", "SolidDisplay", "Animator", "Animation"]




class ExplodableGroup(Group):
	''' just like a group but allowing exploded views 
	
		expoloded view is enabled by right-clicks on content when `solid_freemove` is enabled in the scene
	'''
	def __init__(self, scene, src=None, world=1):
		super().__init__(scene, src, world)
		self._exploded = False
	
	def stack(self, scene):
		if not scene.options['solid_freemove']:
			self.unexplode()
		return super().stack(scene)
	
	def control(self, view, key, sub, event):
		if not view.scene.options['solid_freemove']:
			return
		if event.button() == Qt.MouseButton.RightButton and event.type() == QEvent.MouseButtonRelease:
			stack = []
			disp = self
			for i,key in enumerate(sub[:-1]):
				if isinstance(disp, ExplodableGroup):
					stack.append(disp)
				disp = disp[key]
			# explode if interacted display not already exploded
			explode = not stack[-1].exploded
			
			boxes = {}
			for disp in reversed(stack):
				if explode:
					disp.explode(view, boxes)
				else:
					disp.unexplode(view)
			
			event.accept()
			view.scene.touch()
			view.update()
			
	@property
	def exploded(self):
		''' true if this container is currently shown exploded '''
		return self._exploded

	def unexplode(self, view=None):
		''' move children back into nominal position 
		
			if view is given, the result is not instantaneous but animated
		'''
		if self._exploded:
			self._exploded = False
			for child in self.displays.values():
				if isinstance(child, SolidDisplay):
					child.reset(view)
					# child.displays.pop('box', None)
			
	def explode(self, view=None, exploded:dict[int,Box]=None) -> Box[fvec3]:
		''' move `SolidDisplay` children away from each other
		
			if view is given, the result is not instantaneous but animated
		'''
		if self._exploded:
			return
		self._exploded = True
		
		if exploded is None:
			exploded = {}
		
		# collect solids and non-movable displays at this level
		solids = [None]
		nonsolid = Box(fvec3(+inf), fvec3(-inf))
		for child in self.displays.values():
			if isinstance(child, SolidDisplay):
				child._free = fmat4()
				# child.displays.pop('box', None)
				solids.append(child)
			elif hasattr(child, 'box'):
				nonsolid |= child.box
		if nonsolid.isempty():
			solids.pop(0)
		
		# for solid in solids:
		# 	if solid:
		# 		solid['box'] = _box_local(solid)
		
		# retreive boundingboxes
		shapes = []
		for solid in solids:
			if solid is None:
				# special case for nonsolid
				shapes.append(SolidBox(
					box = nonsolid.cast(vec3),
					exploded = nonsolid.cast(vec3),
					place = mat4(),
					))
			else:
				# solid could be exploded inside
				if id(solid) in exploded:
					ex = exploded[id(solid)].cast(vec3)
				else:
					ex = boundingbox(child.box  for child in solid.displays.values()  if hasattr(child, 'box')) .cast(vec3)
				shapes.append(SolidBox(
					box = _box_local(solid) .cast(vec3),
					exploded = ex,
					place = mat4(solid.local),
					))
		
		# compute offsets (relative to first item in case there is non-movable items)
		offsets = explode_offsets(shapes, spacing=0.5)
		offsets = [offset - offsets[0]  for offset in offsets]

		# move solids
		for solid, offset in zip(solids, offsets):
			if solid:
				if view:
					inv = transpose(fmat3(solid.local))
					def animation(x, solid=solid, offset=offset, inv=inv):
						solid.free = translate(inv * smoothstep(0, 1, x) * fvec3(offset))
						view.update()
					solid._animate(0.15, animation)
				else:
					solid.free = translate(inverse(fmat3(solid.local)) * fvec3(offset))
		
		exploded[id(self)] = boundingbox(shape.box.transform(translate(offset) * shape.place) for shape, offset in zip(shapes, offsets)) .cast(fvec3)

def _box_local(display):
	''' get the local boundingbox of a soliddisplay, or the normal box for other displays '''
	if isinstance(display, SolidDisplay):
		return boundingbox(
			(_box_local(sub)   for sub in display.displays.values()), 
			ignore=True)
	else:
		return getattr(display, 'box', None)

Scene.overrides[list] = ExplodableGroup
Scene.overrides[dict] = ExplodableGroup


			

class SolidDisplay(ExplodableGroup):
	''' Movable and explodable `Group` for the rendering pipeline '''
	def __init__(self, scene, solid:Solid, world=fmat4()):
		self._local = fmat4()
		self._free = fmat4()
		self._animation = None
		super().__init__(scene, world=world)
		self._animate = scene.share(Animator, Animator)
		self.update(self, solid)
	
	@property
	def box(self):
		return boundingbox(
			(getattr(display, 'box', None)   for display in self.displays.values()), 
			ignore=True) .transform(self._local * self._free)

	@writeproperty
	def local(self):  self._place_displays()
			
	@writeproperty
	def world(self):  self._place_displays()
			
	@writeproperty
	def free(self):   self._place_displays()
	
	def _place_displays(self):
		sub = self._world * self._local * self._free
		for display in self.displays.values():
			display.world = sub
	
	def update(self, *args):
		''' 
			update(scene, src)
			update(src)
		
			update to the solid pose and its content dictionnary
			
			Note:
				the new content will not be immediately available in `self.displays` because they will only be buffered at next rendering. it makes this function thread safe and able to run without a reference to the scene
		'''
		if len(args) == 1:
			src, = args
		elif len(args) == 2:
			scene, src = args
		else:
			raise TypeError("expected 1 or 2 arguments, got {}".format(len(args)))
		if not isinstance(src, Solid):
			return False
		content = dict(src)
		pose = content.pop('pose')
		if not super().update(content):
			return False
		self._local = fmat4(pose)
		return True
	
	def stack(self, scene):
		self.prepare(scene)
		if not scene.options['solid_freemove']:
			self.reset()

		sub = self._world * self._local * self._free
		for key, display in self.displays.items():
			display.world = sub
			display.key = (*self.key, key)
			if key == 'annotations' and not scene.options['display_annotations'] and not self.selected:
				continue
			yield from display.stack(scene)

	def control(self, view, key, sub, evt):
		super().control(view, key, sub, evt)
		if not view.scene.options['solid_freemove']:
			return
		if self.selected:
			# accept clicks to listen for dragging
			if evt.type() == QEvent.Type.MouseButtonPress and (
					evt.button() == Qt.MouseButton.LeftButton
					or evt.button() == Qt.MouseButton.RightButton):
				evt.accept()
			
			elif evt.type() == QEvent.Type.MouseMove:
				# drag with left click moves the solid
				if evt.buttons() & Qt.MouseButton.LeftButton:
					evt.accept()
					# put the tool into the view, to handle events
					view.callbacks.append(receiver(self._move(view, evt)))
					self.animated = False
			
				# drag with right click resets its free pose
				elif evt.buttons() & Qt.MouseButton.RightButton:
					evt.accept()
					if self._free != fmat4():
						self.reset(view)
				
	def _move(self, view, evt):
		''' moves the selected solids in the view plane following the mouse movements '''
		moving = [(solid, solid.free)   
			for solid in view.scene.selection if isinstance(solid, SolidDisplay)]
		
		anchor = view.ptat(view.somenear(evt.pos()))
		while True:
			evt = yield
			if not evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				continue
			evt.accept()
			if not (evt.buttons() & Qt.MouseButton.LeftButton):
				break
			view.update()
			
			click = view.ptfrom(evt.pos(), anchor)
			
			for solid, former in moving:
				move = transpose(fmat3(solid._world * solid._local * solid._free)) * (click - anchor)
				solid.free = former * translate(move)

	def reset(self, view=None):
		''' reset solid free position, with animation is view is provided '''
		if view:
			if (self._animation and not self._animation.complete()):
				return
			rinit = fquat(fmat3(self.free))
			tinit = fvec3(self.free[3])
			rfinal = fquat()
			tfinal = fvec3(0)
			def animation(x):
				x = smoothstep(0, 1, x)
				self.free = translate(mix(tinit, tfinal, x)) * fmat4(slerp(rinit, rfinal, x))
				view.update()
			self._animation = self._animate(0.15, animation)
		else:
			self.free = fmat4()

			

class Animator:
	''' shared object for managing simple animations in the madcad scene '''
	def __init__(self, scene):
		self.timer = None
		self.animations = {}
		self.removal = set()
	
	def __call__(self, duration:float, callback:callable) -> Animation:
		''' schedule an animation callback
			it receives values from 0 to 1, 1 is passed the last time the callback is ran 
		'''
		# JIT initialization, so no ressource is loaded when no animation ever run
		if not self.timer:
			self.timer = QTimer()
			self.timer.timeout.connect(self._step)
			self.timer.setInterval(30)
		# add an entry
		id = 0
		while id in self.animations:
			id = random.randrange(len(self.animations)*2)
		entry = [time(), duration, callback]
		self.animations[id] = entry
		self.timer.start()
		return Animation(self, id)
	
	def _step(self):
		# run all callbacks
		now = time()
		for id, (start, duration, callback) in self.animations.items():
			if not callback:
				continue
			progress = (now-start)/duration
			if 0 <= progress <= 1:
				callback((time()-start)/duration)
			else:
				callback(1.)
				self.animations[id][2] = None
		# stop when no more animations
		if not self.animations:
			self.timer.stop()

class Animation:
	''' animation handle '''
	def __init__(self, animator, id):
		self.animator = animator
		self.id = id
	def __del__(self):
		if self.id is not None:
			self.animator.animations[self.id]
			self.id = None
	
	def complete(self):
		''' return True if the animation is over '''
		if self.id in self.animator.animations:
			start, duration, _ = self.animator.animations[self.id]
			return time() - start >= duration
		return True
	
	def stop(self):
		''' stop the animation immediately, callback(1) is called a last time '''
		if self.id in self.animator.animations:
			_, _, callback = self.animator.animations.pop(self.id)
			callback(1)
			self.id = None
