# This file is part of pymadcad,  distributed under license LGPL v3
'''
	module providing different kind of 3d views over a Scene, and few helpers with them
'''
from __future__ import annotations

import numpy as np
import moderngl as mgl
from pnprint import nprint

from ... import settings
from ...mathutils import *
from ..base import Scene, empty
from ..utils import *

__all__ = ['GLView3D', 'Offscreen3D', 'QView3D', 'Orbit', 'Turntable', 'Perspective', 'Orthographic']


class GLView3D:
	''' pure openGL 3D view over a madcad scene '''
	scene: Scene
	''' madcad scene to render, it must contain 3D displays (at least a the root) '''
	screen: mgl.Framebuffer
	''' color map (and depth) openGL buffer '''
	ident: mgl.Framebuffer|None
	''' identification map (and depth) openGL buffer '''
	targets: list
	''' exposed for `Scene` '''
	uniforms: dict
	''' exposed for `Scene` '''
	view: fmat4
	''' current view matrix, this will be the default for next rendering '''
	proj: fmat4
	''' current projection matrix, this will be the default for next rendering '''
	enable_ident: bool
	''' enable `self.ident` '''
	
	def __init__(self, scene, size:uvec2=None, view:fmat4=None, proj:fmat4=None, 
			enable_ident=True, 
			**uniforms):
		self.scene = scene if isinstance(scene, Scene) else Scene(scene)
		self.view = view
		self.proj = proj
		self.uniforms = uniforms
		self.screen = None
		self.ident = None
		self.targets = []
		self.enable_ident = enable_ident
		# indentifiers management for the ident map
		self._steps = []  # identifiers before each ident stack step
		self._stepi = 0
		self._step = 0
		
		self.scene.root.world = fmat4()
		if size:
			self._reallocate(size)
	
	def render(self, size:uvec2=None, view:fmat4=None, proj:fmat4=None) -> Self:
		''' trigger the rendering of a frame, do not wait for the result 
		
			- the `view` and `proj` instance attributes can be changed on the fly without extra cost.
			- a `size` change will trigger reallocation of the buffers
		'''
		size = glsize(size)
		self._reallocate(size)
		# setup uniforms
		if view:    self.view = view
		if proj:    self.proj = proj
		self.uniforms['size'] = uvec2(self.screen.size)
		self.uniforms['view'] = self.view
		self.uniforms['proj'] = self.proj
		self.uniforms['projview'] = self.proj * self.view
		# normal rendering of the scene
		self.scene.render(self)
		# downsample the screen buffer to 1 sample per pixel
		self.scene.context.copy_framebuffer(self.screen, self._screen_multisample)
		
		# self.screen.use()
		# self.screen.clear(1,0,0)
		self.scene.context.finish()
		return self
		
	def _reallocate(self, size):
		''' ensure the framebuffer fit the given size, reallocate if needed '''
		if self.screen and size == self.screen.size:
			return
		ctx = self.scene.context
		self.targets.clear()
		self.screen = ctx.simple_framebuffer(size, components=3, dtype='f1')
		self._screen_multisample = ctx.simple_framebuffer(size, components=3, samples=4, dtype='f1')
		self.targets.append(('screen', self._screen_multisample, self._setup_screen))
		if self.enable_ident:
			self.ident = ctx.simple_framebuffer(size, components=1, dtype='u2')
			self.targets.append(('ident', self.ident, self._setup_ident))
		else:
			self.ident = None

	def _setup_ident(self):
		''' ident rendering setup '''
		self._stepi = 0
		self._step = 1
		if len(self.scene.stacks.get('ident', empty)) != len(self._steps):
			self._steps = [0] * len(self.scene.stacks['ident'])
		# ident rendering setup
		ctx = self.scene.context
		ctx.multisample = False
		ctx.enable_only(mgl.DEPTH_TEST)
		ctx.blend_func = mgl.ONE, mgl.ZERO
		ctx.blend_equation = mgl.FUNC_ADD
		self.target.clear()

	def _setup_screen(self):
		''' screen rendering setup '''
		ctx = self.scene.context
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

	def identstep(self, nidents):
		''' Updates the amount of rendered idents and return the start ident for the calling rendering pass?
			Method to call during a renderstep
		'''
		s = self._step
		self._step += nidents
		self._steps[self._stepi] = self._step-1
		self._stepi += 1
		return s

	
class Offscreen3D:
	''' 3D view giving images accessible to numpy buffers '''
	gl: View
	''' opengl renderer '''
	color: ndarray[np.uint8]
	''' result image from the previous rendering, showing the  '''
	depth: ndarray[np.float32]|None
	''' result image from the previous rendering '''
	ident: ndarray[np.uint16]|None
	''' result image from the previous rendering '''
	enable_alpha: bool
	''' if enabled, `self.color` is RGBA else it is 'RGB' '''
	enable_depth: bool
	''' enable `self.depth` '''
	enable_ident: bool
	''' enable `self.ident` '''
		
	scene = forwardproperty('gl', 'scene')
	view = forwardproperty('gl', 'view')
	proj = forwardproperty('gl', 'proj')
	uniforms = forwardproperty('gl', 'uniforms')
	enable_ident = forwardproperty('gl', 'enable_ident')
	
	def __init__(self, scene, size:uvec2=None, view:fmat4=None, proj:fmat4=None, 
			enable_depth=False, 
			enable_ident=False, 
			enable_alpha=False,
			**uniforms):
		self.gl = GLView3D(scene, size, view, proj, enable_ident, **uniforms)
		self.color = self.depth = self.ident = None
		self.enable_depth = enable_depth
		self.enable_alpha = enable_alpha
		if size:
			self._reallocate(size)
	
	def render(self, size:uvec2=None, view:fmat4=None, proj:fmat4=None) -> Self:
		'''
			render the scene and retreive the result in the `color`, `depth` and `ident` attributes
			
			- the `view` and `proj` instance attributes can be changed on the fly without extra cost.
			- a `size` change will trigger reallocation of the buffers
		'''
		if size:
			size = glsize(size)
			self.gl.render(size, view, proj)
			self._reallocate(size)
		# retreive everything from the GPU to the CPU
		if self.enable_alpha:
			self.gl.screen.read_into(self.color, components=4)
		else:
			self.gl.screen.read_into(self.color, components=3)
		# switch vertical axis to convert from opengl image convention to usual (and Qt) image convention
		self.color[:] = self.color[::-1]
		if self.enable_ident:
			self.gl.ident.read_into(self.ident, attachment=0)
			self.ident[:] = self.ident[::-1]
		if self.enable_depth:
			self.gl.screen.read_into(self.depth, attachment=-1)
			self.depth[:] = self.depth[::-1]
		return self
		
	def _reallocate(self, size):
		if self.color is not None and size == self.color.shape[::-1]:
			return
		w,h = size
		self.color = np.empty((h,w,3), dtype='u1')
		self.ident = np.empty((h,w), dtype='u2')  if self.enable_ident else None
		self.depth = np.empty((h,w), dtype='f4')  if self.enable_depth else None


try:
	from ...qt import (Qt, QApplication, QWidget, QImage, QPainter, 
					QEvent, QInputEvent, QMouseEvent, QKeyEvent, QTouchEvent)
except ImportError:
	pass
else:

	class QView3D(QWidget):
		gl: View
		''' underlying opengl rendering system '''
		navigation: Orbit|TurnTable|None
		''' turns basic screen actions into camera movements '''
		projection: Perspective|Orthographic
		''' generates the projection matrix '''
		color: ndarray[np.uint8]
		''' last rendering '''
		depth: CheapMap[np.float32]
		''' access to the depthmap from last rendering '''
		ident: CheapMap[np.uint16]
		''' access to the identification map from last rendering '''
		callbacks: list
		''' list of callbacks subscribing to input events
		
			they will be called in revered order, and as any Qt event handler, this first accepting prevents the other to be called
		'''
		
		scene = forwardproperty('gl', 'scene')
		view = forwardproperty('gl', 'view')
		proj = forwardproperty('gl', 'proj')
		uniforms = forwardproperty('gl', 'uniforms')
		
		def __init__(self, scene, projection=None, navigation=True, parent=None):
			QWidget.__init__(self, parent)
			self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
			self.setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents, True)
			self.setMouseTracking(True)
			self.gl = GLView3D(scene)
			if navigation is True:
				navigation = globals()[settings.controls['navigation']]()
			self.navigation = navigation
			self.projection = projection or globals()[settings.scene['projection']]()
			self.color = self.depth = self.ident = None
			self.callbacks = []
			
			self._move_mode = receiver(self._move_mode())
			self._keyboard_move = receiver(self._keyboard_move())
			self._mouse_move = receiver(self._mouse_move())
			self._touch_move = receiver(self._touch_move())
			
		def _reallocate(self, size):
			w, h = size
			if self.color is not None and w == self.color.shape[1] and h == self.color.shape[0]:
				return
			self.ident = CheapMap(self.gl.ident, attachment=0)
			self.depth = CheapMap(self.gl.ident, attachment=-1)
			self.color = np.empty((h,w,3), dtype='u1')
		
		def paintEvent(self, painter):
			w = self.width()
			h = self.height()
			size = glsize(uvec2(w, h))
			
			self.gl.render(
				size,
				view = self.navigation.matrix(),
				proj = self.projection.matrix(w/h if h > 0 else 0, self.navigation.distance),
			)
			self._reallocate(size)
			# retreive buffer
			self.gl.screen.read_into(self.color, components=3)
			# switch vertical axis to convert from opengl image convention to usual (and Qt) image convention
			self.color[:] = self.color[::-1]
			# copy to Qt
			image = QImage(
				self.color,
				self.color.shape[1], 
				self.color.shape[0],
				QImage.Format.Format_RGB888)
			painter = QPainter(self).drawImage(0, 0, image)
		
		def event(self, evt):
			if isinstance(evt, QInputEvent):
				evt.ignore()
				self.inputEvent(evt)
				if evt.isAccepted():	
					return True
			return super().event(evt)
		
		def resizeEvent(self, evt):
			QWidget.resizeEvent(self, evt)
			self.update()
		
		def changeEvent(self, evt):
			# detect theme change
			if evt.type() == QEvent.Type.PaletteChange and settings.display['system_theme']:
				settings.use_qt_colors()
			return QWidget.changeEvent(self, evt)
		
		def leaveEvent(self, evt):
			self.scene.hover = None
			self.update()
		
		def inputEvent(self, evt):
			''' Default handler for every input event (mouse move, press, release, keyboard, ...)
				When the event is not accepted, the usual matching Qt handlers are used (mousePressEvent, KeyPressEvent, etc).

				This function can be overwritten to change the view widget behavior.
			'''
			evt.ignore()
			
			for i in reversed(range(len(self.callbacks))):
				if not self.callbacks[i](evt):
					self.callbacks.pop(i)
				if evt.isAccepted():
					return
			
			if self.navigation:
				self._move_mode(evt)
				self._keyboard_move(evt)
				self._mouse_move(evt)
				self._touch_move(evt)
				if evt.isAccepted():
					return
			
			# send the event to the scene objects, descending the item tree
			pos = None
			if isinstance(evt, QMouseEvent):
				pos = self.somenear(evt.pos())
				if pos:
					key = self.displayat(pos)
					if key:
						disp, sub = key
						key = (*disp.key, sub)
						
						self.control(key, evt)
						if evt.isAccepted():
							return
						self._selection(disp, sub, evt)
						if evt.isAccepted():
							return
			if not pos:
				self._deselection(evt)
			
			# if clicks are not accepted, then some following keyboard events may not come to the widget
			# NOTE this also discarding the ability to move the window from empty areas
			if evt.type() == QEvent.Type.MouseButtonPress:
				evt.accept()
		
		def control(self, key, evt):
			''' Transmit a control event successively to all the displays matching the key path stages.
				At each level, if the event is not accepted, it transmits to sub items

				This function can be overwritten to change the interaction with the scene objects.
			'''
			disp = self.scene.root.displays
			for i in range(1,len(key)):
				disp = disp[key[i-1]]
				disp.control(self, key[:i], key[i:], evt)
				if evt.isAccepted(): 
					self.update()
					break
				
		def _selection(self, disp, sub, evt):
			''' handle the selection events '''
			if not self.scene.options['selection_sub']:
				sub = None
			
			# selection on click above
			if evt.type() == QEvent.Type.MouseButtonRelease and evt.button() == Qt.MouseButton.LeftButton and hasattr(disp, 'selected'):
				shift = bool(QApplication.queryKeyboardModifiers() & Qt.KeyboardModifier.ShiftModifier)
				if not self.scene.options['selection_multiple'] ^ shift:
					self.scene.selection_clear()
				self.scene.selection_toggle(disp, sub)
				self.update()
				evt.accept()
				return
			
			# hover on move above
			if evt.type() == QEvent.Type.MouseMove and hasattr(disp, 'hovered'):
				# precise condition on hover change, to avoid triggering unnecessary renderings
				if disp is not self.scene.hover or (isinstance(disp.hovered, set) and sub not in disp.hovered):
					self.scene.hover = (disp, sub)
					self.update()
					
		def _deselection(self, evt):
			''' handle the deselection events '''
			if evt.type() == QEvent.Type.MouseButtonPress and evt.button() == Qt.MouseButton.LeftButton:
				self.scene.selection_clear()
				self.update()
				evt.accept()
			elif evt.type() == QEvent.Type.MouseMove:
				if self.scene.hover:
					self.scene.hover = None
					self.update()
		
		# move the view following events
		# navigation modes
		_ZOOM = 1
		_PAN = 2
		_ROTATE = 3
		
		def _move_mode(self):
			''' change the navigation mode according to keyboard or mouse events '''
			ctrl = alt = False
			self._slow = False
			self._mode = None
			while True:
				evt = yield
				
				if isinstance(evt, QKeyEvent):
					k = evt.key()
					press = evt.type() == QEvent.Type.KeyPress
					if	 k == Qt.Key.Key_Control:	ctrl = press
					elif k == Qt.Key.Key_Alt:		alt = press
					elif k == Qt.Key.Key_Shift:		self._slow = press
				elif evt.type() == QEvent.Type.MouseButtonPress:
					modifiers = QApplication.queryKeyboardModifiers()
					ctrl = bool(modifiers & Qt.KeyboardModifier.ControlModifier)
					alt = bool(modifiers & Qt.KeyboardModifier.AltModifier)
					shift = bool(modifiers & Qt.KeyboardModifier.ShiftModifier)
				else:
					continue
				
				if evt.type() == QEvent.Type.MouseButtonPress and evt.button() == Qt.MouseButton.MiddleButton:
					self._mode = self._ROTATE
				else:
					if ctrl and alt:		self._mode = self._ZOOM
					elif ctrl:				self._mode = self._PAN
					elif alt:				self._mode = self._ROTATE
					else:					self._mode = None
					
		def _keyboard_move(self):
			''' navigate the view with the keyboard '''
			while True:
				evt = yield
				
				if self._mode and evt.type() == QEvent.Type.KeyPress:
					dx = dy = 0
					speed = pi/72 if self._slow else pi/24
					if evt.key() == Qt.Key.Key_Left:     dx = -speed
					elif evt.key() == Qt.Key.Key_Right:  dx = +speed
					elif evt.key() == Qt.Key.Key_Down:   dy = -speed
					elif evt.key() == Qt.Key.Key_Up:     dy = +speed
					else:
						continue
					
					if self._mode == self._PAN:
						self.navigation.pan(fvec2(dx, dy))
					elif self._mode == self._ROTATE:
						self.navigation.rotate(fvec3(dx, dy, 0))
					elif self._mode == self._ZOOM:
						self.navigation.rotate(dy)
						
					evt.accept()
					self.update()
				
		def _mouse_move(self):
			''' navigate the view using the mouse '''
			while True:
				evt = yield
				
				if evt.type() == QEvent.Type.Wheel:
					self.navigation.zoom(exp(-evt.angleDelta().y()/(8*90)))	# the 8 factor is there because of the Qt documentation
					evt.accept()
					self.update()
				
				if self._mode and evt.type() == QEvent.Type.MouseButtonPress:
					last = evt.pos()
					evt.accept()
					while True:
						evt = yield
						
						if not self._mode or evt.type() == QEvent.Type.MouseButtonRelease or isinstance(evt, QMouseEvent) and evt.buttons() == Qt.MouseButton.NoButton:
							break
						if evt.type() != QEvent.Type.MouseMove:
							continue
						
						gap = evt.pos() - last
						dx = gap.x()/self.height()
						dy = gap.y()/self.height()
						if self._mode == self._PAN:		
							self.navigation.pan(fvec2(dx, dy))
						elif self._mode == self._ROTATE:
							self.navigation.rotate(fvec3(dx, dy, 0))
						elif self._mode == self._ZOOM:
							middle = QPoint(self.width(), self.height())/2
							f = (	(last-middle).manhattanLength()
								/	(evt.pos()-middle).manhattanLength()	)
							self.navigation.zoom(f)
						last = evt.pos()
						
						evt.accept()
						self.update()
		
		def _touch_move(self):
			''' navigate the view using the touchscreen '''
			while True:
				evt = yield
				if isinstance(evt, QTouchEvent):
					self._mode = None
					pts = evt.touchPoints()
					# view rotation
					if len(pts) == 2:
						startlength = (pts[0].lastPos()-pts[1].lastPos()).manhattanLength()
						zoom = startlength / (pts[0].pos()-pts[1].pos()).manhattanLength()
						displt = (	(pts[0].pos()+pts[1].pos()) /2 
								-	(pts[0].lastPos()+pts[1].lastPos()) /2 ) /self.height()
						dc = pts[0].pos() - pts[1].pos()
						dl = pts[0].lastPos() - pts[1].lastPos()
						rot = atan2(dc.y(), dc.x()) - atan2(dl.y(), dl.x())
						self.navigation.zoom(zoom)
						self.navigation.rotate(fvec3(displt.x(), displt.y(), rot))
						self.update()
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
						displt = (cc - lc)  /self.height()
						self.navigation.zoom(zoom)
						self.navigation.pan(fvec2(displt.x(), displt.y()))
						self.update()
						evt.accept()
					# finish a gesture
					elif evt.type() in (QEvent.Type.TouchEnd, QEvent.Type.TouchUpdate):
						evt.accept()
		
		# forward access to items
				
		def __getitem__(self, key):
			self.root[key]
		def __setitem__(self, key, value):
			self.root[key] = value
		def __iter__(self):
			return iter(self.root)
		
		# extract informations from the ident map
		
		def somenear(self, point: QPoint, radius=None) -> QPoint|None:
			''' Return the closest coordinate to coords, (within the given radius) for which there is an object at
				So if objnear is returning something, objat and ptat will return something at the returned point
			'''
			point = qpoint_to_vec(point)
			if radius is None:
				radius = settings.controls['snap_dist']
			region = self.ident.region((*(point-radius), *(point+radius+1)))
			for x,y in snailaround(uvec2(radius), region.shape, radius):
				ident = int(region[y, x])
				if ident:
					return vec_to_qpoint(point-radius + ivec2(x,y))

		def ptat(self, point: QPoint) -> fvec3|None:
			''' Return the point of the rendered surfaces that match the given window coordinates '''
			point = qpoint_to_vec(point)
			region = self.depth.region((*point, *(point+1)))
			size = self.uniforms['size']
			depthred = float(region[0,0])
			x =  (point.x/size.x *2 -1)
			y = -(point.y/size.y *2 -1)

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
				return fvec3(affineInverse(view) * fvec4(
							depth * x /proj[0][0],
							depth * y /proj[1][1],
							-depth,
							1))

		def ptfrom(self, point: QPoint, center: fvec3|vec3) -> fvec3:
			''' 3D point below the cursor in the plane orthogonal to the sight, with center as origin '''
			point = qpoint_to_vec(point)
			view = self.uniforms['view']
			proj = self.uniforms['proj']
			size = self.uniforms['size']
			x =  (point.x/size.x *2 -1)
			y = -(point.y/size.y *2 -1)
			depth = (view * fvec4(fvec3(center),1))[2]
			return fvec3(affineInverse(view) * fvec4(
						-depth * x /proj[0][0],
						-depth * y /proj[1][1],
						depth,
						1))

		def displayat(self, point: QPoint) -> (Display, int)|None:
			''' Return the display at the given screen position (widget relative).
			
				Every display can have a range of ident values in the ident map, they may correspond to subelements whose meaning only depends on the display. the subindex is returned with the display.
				
				If no object is at this exact location, None is returned.
			'''
			point = qpoint_to_vec(point)
			ident = self.ident.region((*point, *(point+1)))[0,0]
			if ident and 'ident' in self.scene.stacks:
				rdri = bisect(self.gl._steps, ident)
				if rdri == len(self.gl._steps):
					print('internal error: object ident points out of idents list')
					nprint(self.gl._steps)
				while rdri > 0 and self.gl._steps[rdri-1] == ident:	rdri -= 1
				if rdri > 0:	subi = ident - self.gl._steps[rdri-1] - 1
				else:			subi = ident - 1
				
				if rdri >= len(self.scene.stacks['ident']):
					print('wrong identification index', ident, self.scene.stacks['ident'][-1])
					nprint(self.gl._steps)
					return
				
				return (self.scene.stacks['ident'][rdri].display, int(subi))
				
		# move the view (in addition to what already exists in navigation)
		
		def look(self, position: fvec3=None):
			''' Make the scene navigation look at the position.
				This is changing the camera direction, center and distance.
			'''
			if not position:	position = self.scene.box().center
			self.navigation.look(position)
			self.update()
		
		def adjust(self, box:Box=None):
			''' Make the navigation camera large enough to get the given box in .
				This is changing the zoom level
			'''
			if not box:	box = self.scene.root.box
			if box.isempty():	return

			# get the most distant point to the focal axis
			invview = affineInverse(self.navigation.matrix())
			camera, look = fvec3(invview[3]), fvec3(invview[2])
			margin = 0.3  # margins around the zomed box, to make sure the displays fits in the view, sqrt(2)-1 should be necessary, but lower is generally better
			dist = length(noproject(box.center-camera, look)) + max(glm.abs(box.width))/2 * (1+margin)
			if not dist > 1e-6:	return

			# adjust navigation distance
			self.navigation.adjust(self.projection.adjust(dist))
			self.update()

		def center(self, center: fvec3=None):
			''' Relocate the navigation to the given position .
				This is translating the camera.
			'''
			if not center:	center = self.scene.box().center
			if not isfinite(center):	return

			self.navigation.center(center)
			self.update()


# view matrix providers

class Orbit:
	''' Navigation rotating on the 3 axis around a center.

		Object used as `View.navigation`
	'''
	position: fvec3
	distance: float
	orientation: fquat

	def __init__(self, position:fvec3=0, distance:float=1, orient:fvec3=fvec3(1,0,0)):
		self.center = fvec3(center)
		self.distance = float(distance)
		self.orient = fquat(orient)
		self.tool = navigation_tool

	def matrix(self) -> fmat4:
		mat = translate(fmat4(self.orient), -self.center)
		mat[3][2] -= self.distance
		return mat

	def adjust(self, distance: float):
		self.distance = distance

	def look(self, position: fvec3):
		dir = position - fvec3(affineInverse(self.matrix())[3])
		self.distance = length(dir)
		self.position = position
		self.sight(dir)
		
	def sight(self, direction: fvec3):
		if length2(direction, direction) <= 1e-6:
			return
		focal = self.orient * fvec3(0,0,1)
		self.orient = quat(direction, focal) * self.orient

	def center(self, position: fvec3):
		self.position = position

	def zoom(self, ratio: float):
		self.distance *= f

	def pan(self, offset: vec2):
		x,y,z = transpose(fmat3(self.orient))
		self.center += (fvec3(x) * -offset.x + fvec3(y) * offset.y) * self.distance/2

	def rotate(self, offset: vec3):
		# rotate from view euler angles
		self.orient = inverse(fquat(fvec3(-offset.y, -offset.dx, offset.dz) * pi)) * self.orient

class Turntable:
	''' Navigation rotating on yaw and pitch around a center 
	
		Object used as `View.navigation`
	'''
	position: fvec3
	distance: float
	pitch: float
	yaw: float
	
	def __init__(self, position:fvec3=0, distance:float=1, yaw:float=0, pitch:float=0):
		self.position = fvec3(position)
		self.yaw = yaw
		self.pitch = pitch
		self.distance = distance
	
	def matrix(self) -> fmat4:
		# build rotation from view euler angles
		rot = inverse(fquat(fvec3(pi/2-self.pitch, 0, -self.yaw)))
		mat = translate(fmat4(rot), -self.position)
		mat[3][2] -= self.distance
		return mat
	
	def adjust(self, distance: float):
		self.distance = distance
	
	def look(self, position: fvec3):
		dir = position - fvec3(affineInverse(self.matrix())[3])
		self.distance = length(dir)
		self.position = position
		self.sight(dir)
	
	def sight(self, direction: fvec3):
		if length2(direction) <= 1e-6:
			return
		self.yaw = atan2(direction.x, direction.y)
		self.pitch = -atan2(direction.z, length(direction.xy))
	
	def center(self, position: fvec3):
		self.position = position
	
	def zoom(self, ratio: float):
		self.distance *= ratio
	
	def pan(self, offset: fvec2):
		mat = transpose(fmat3(inverse(fquat(fvec3(pi/2-self.pitch, 0, -self.yaw)))))
		self.position += ( mat[0] * -offset.x + mat[1] * offset.y) * self.distance/2
	
	def rotate(self, offset: fvec3):
		if abs(self.pitch) > 3.2:	offset.x = -offset.x
		self.yaw += offset.x*pi
		self.pitch += offset.y*pi
		if self.pitch > pi:	self.pitch -= 2*pi
		if self.pitch < -pi: self.pitch += 2*pi

# projection matrix providers

class Perspective:
	''' Object used as `View.projection` 
	
		Attributes:
			fov (float):	field of view (rad), defaulting to `settings.display['field_of_view']`
	'''
	def __init__(self, fov=None):
		self.fov = fov or settings.display['field_of_view']
	
	def matrix(self, ratio, distance) -> fmat4:
		return perspective(self.fov/min(1, ratio), ratio, distance*1e-2, distance*1e4)
	
	def adjust(self, size):
		return size / tan(self.fov/2)

class Orthographic:
	''' Object used as `View.projection` 
	
		Attributes:
			size (float):  
			
				factor between the distance from camera to navigation center and the zone size to display
				defaulting to `tan(settings.display['field_of_view']/2)`
	'''
	def __init__(self, size=None):
		self.size = size or tan(settings.display['field_of_view']/2)

	def matrix(self, ratio, distance) -> fmat4:
		rf = 1e3 # relative far
		rn = 1e-2 # relative near
		return fmat4(min(1, 1/ratio)/(distance*self.size), 0, 0, 0,
					0,       min(1, ratio)/(distance*self.size), 0, 0,
					0,       0,          -2/(distance*(rf-rn)), 0,
					0,       0,          -(rf+rn)/(rf-rn), 1)
					
	def adjust(self, size):
		return size / self.size
