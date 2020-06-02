'''	Display module of pymadcad
	
	This module provide a Qt widget 'Scene' to handle user events and implements a graphic pipeline for the displayed objects
	The objects displayed can be of any type but must implement the display protocol
	
	display protocol
	----------------
		a displayable is an object that have a method `display() -> iterator` returning an iterable of display objects
		a display object is a part of the rendering pipeline.
		these are defined as follow:
			- a class/object member  `renderindex`
				used to determine the place of this display in the pipeline (the bigger will be rendered the latter)
			- a method `render(scene)`
			- a method `identify(scene, startindex) -> (num of indices)`
				used to render a map of ids for selectable regions
				if there is no selectable regions, just return 0
		
		displays can implement a selection method
			- a method `select(idents, state=None)`	
				setting the selection state of the ident(s) passed, or returning them if state is None
			NOTE: to be useful, the idents used here must be the same as those baked by `identify`
		
		displays can defines a custom user control callback (user actions triggered when the display is being clicked):
			when an object is clicked on, its method `control(scene, ident, event) -> callable` is called
			the returned object is called for each further action on the view, until the called procedure assign `view.tool = None`
	
	NOTE
	----
		Unfortunately it's not possible to get a multi-view scene. This is due to some Qt limitations (and design choices), that Qt is using separated opengl contexts for each independent widgets. (actually there is ways to deal with data sharing across contexts but it would be very complex to implement it with the objects exposed here)
'''


import numpy.core as np
from PIL import Image
import moderngl as mgl
from copy import copy, deepcopy
from math import tan, sin, cos, pi, exp

from .mathutils import vec3, fvec3, fvec4, fmat3, fmat4, row, column, length, perspective, translate, project, inverse, dichotomy_index, Box
from . import settings

from PyQt5.QtCore import Qt
from PyQt5.QtOpenGL import QGLWidget, QGLFormat
from PyQt5.QtWidgets import QWidget
# new open gl widget (bug)
from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtGui import QSurfaceFormat


opengl_version = (3,3)	# min (3,3)


class Perspective:
	''' class to provide the perspective matrix '''
	__slots__ = ('fov', 'limits')
	def __init__(self, fov=None, limits=None):
		self.limits, self.fov = limits, fov
	def matrix(self, ratio):
		#return perspective(ratio, self.fov, self.limits, dtype='f4')
		fov = self.fov or settings.display['field_of_view']
		limits = self.limits or settings.display['view_limits']
		return perspective(fov, ratio, *limits)
		
class Turntable:
	''' class to rotate the view using the mouse events '''
	def __init__(self):
		self.center = fvec3(0,0,0)
		self.distance = 10
		self.orientation = [0,-pi/2]
		self.speed = 1
		self.update()
	
	def slow(self, enable):
		self.speed = 0.1 if enable else 1
	
	def rotatestart(self):
		self.old_orientation = copy(self.orientation)
	def rotating(self, x,y):
		f = settings.controls['orbit_sensitivity'] * self.speed
		self.orientation[0] = self.old_orientation[0] + f*x
		self.orientation[1] = self.old_orientation[1] - f*y
		if self.orientation[1] > 0:		self.orientation[1] = 0
		elif self.orientation[1] < -pi:	self.orientation[1] = -pi
		self.update()
	
	def panstart(self):
		self.old_center = deepcopy(self.center)
	def paning(self, x,y):
		f = - settings.controls['pan_sensitivity'] * self.speed * self.distance
		vx = fvec3(row(self.mat, 0))
		vy = fvec3(row(self.mat, 1))
		self.center = self.old_center + vx * f*x + vy * f*y
		self.update()
	
	def zoomstart(self):
		self.old_distance = self.distance
	def zooming(self, x,y):
		self.distance = self.old_distance * exp((x+y) * 2 * settings.controls['zoom_sensitivity'] * self.speed)
		self.update()
	
	def zoom(self, amount):
		self.distance *= exp(amount * settings.controls['zoom_sensitivity'])
		self.update()
	
	def update(self):
		f = settings.controls['orbit_sensitivity']
		rotx,roty = self.orientation
		rotation = (
						  fmat3(1.,         0.,          0.,
								0.,  cos(roty),   sin(roty),
								0., -sin(roty),   cos(roty))
						* fmat3( cos(rotx),   sin(rotx),  0.,
								-sin(rotx),   cos(rotx),  0.,
								 0.,          0.,         1.)
					)
		self.mat = mat = translate(fmat4(rotation), -self.center)
		mat[3,2] -= self.distance
		
	def matrix(self):	return self.mat



IDENT_TYPE = 'u2'
IDENT_SIZE = int(IDENT_TYPE[1:]) * 8

class Scene(QOpenGLWidget):
	''' Scene widget to display CAD objects 
		Attributes defined here:
			
		* groups		list of the displays for rendering, grouped by the insertion index
		* projection	an object with a method `matrix(aspectration)` that provide the perspective projection matrix
		* manipulator	an object with a method `matrix()` that provide the world->camera matrix
		* options		a dict of rendering options used by default by displays, this is overloaded by assining the member 'options' of a display
		* ctx			the mgl context used for renders
	'''
	
	def __init__(self, objects=(), projection=None, manipulator=None, parent=None):
		# vieille version: QGLWidget, qui plante quand on écrit du texte
		#fmt = QGLFormat()
		#fmt.setVersion(*opengl_version)
		#fmt.setProfile(QGLFormat.CoreProfile)
		#fmt.setSampleBuffers(True)
		#super().__init__(fmt, parent)
		# nouvelle version: QOpenGLWidget, qui fait des bugs d'affichage
		super().__init__(parent)
		fmt = QSurfaceFormat()
		fmt.setVersion(*opengl_version)
		fmt.setProfile(QSurfaceFormat.CoreProfile)
		fmt.setSamples(4)
		self.setFormat(fmt)
		self.setFocusPolicy(Qt.StrongFocus)
        
        # user members
		self.projection = projection or Perspective()
		self.manipulator = manipulator or Turntable()
		self.ressources = {}
		# display rendering options
		self.options = deepcopy(settings.scene)
		
		self.groups = []
		self.layers = []
		self.stack = []
		self.ident_steps = []
		self.queue = []
		
		self.ctx = None		# opengl context, that is None when no yet initialized
		
		# mouse modes
		self.modes = {
			0b00:	(None, None),
			0b10:	(self.manipulator.rotatestart, self.manipulator.rotating),
			0b01:	(self.manipulator.panstart, self.manipulator.paning),
			0b11:	(self.manipulator.zoomstart, self.manipulator.zooming),
			}
		self.speckeys = 0b00
		self.mode = self.modes[self.speckeys]
		self.tool = None
		
		for obj in objects:	self.add(obj)
	
	
	def initializeGL(self):	pass

	def paintGL(self):
		self.ctx = mgl.create_context()
		#self.screen = self.ctx.detect_framebuffer()	# old glwidget
		self.screen = self.ctx.detect_framebuffer(self.defaultFramebufferObject()) # new glwidget
		self.ident_frame = self.ctx.simple_framebuffer((self.size().width(), self.size().height()), components=3, dtype='f1')
		self.ident_shader = self.ctx.program(
						vertex_shader=open('shaders/object-ident.vert').read(),
						fragment_shader=open('shaders/ident.frag').read(),
						)
		self.subident_shader = self.ctx.program(
						vertex_shader=open('shaders/object-item-ident.vert').read(),
						fragment_shader=open('shaders/ident.frag').read(),
						)
		self.init()
		self.render()
		self.paintGL = self.render

	def init(self):
		if not self.ctx:	return False
		
		w, h = self.size().width(), self.size().height()
		self.aspectratio = w/h
		# self.sceeen has already been resized by Qt
		#self.ident_frame.viewport = (0, 0, w, h)	# reallocate the framebuffer
		self.ident_frame = self.ctx.simple_framebuffer((self.size().width(), self.size().height()), components=3, dtype='f1')
		self.screen = self.ctx.detect_framebuffer(self.defaultFramebufferObject())
		self.ident_map = np.empty((h,w), dtype='u2')
		self.depth_map = np.empty((h,w), dtype='f4')
		
		return True

	def render(self):
		''' render the scene to the graphic buffer. need to be called from the opengl thread (often the main thread) '''
		self.dequeue()
		
		# setup the render pass
		self.view_matrix = self.manipulator.matrix()	# column major matrix for opengl
		self.proj_matrix = self.projection.matrix(self.aspectratio)		# column major matrix for opengl
		
		# set the default flags for the scene
		self.ctx.multisample = True
		self.ctx.enable_only(mgl.BLEND | mgl.DEPTH_TEST)
		self.ctx.blend_func = mgl.SRC_ALPHA, mgl.ONE_MINUS_SRC_ALPHA
		self.ctx.blend_equation = mgl.FUNC_ADD
		# render objects
		self.screen.use()
		self.screen.clear()
		for grp,rdr in self.stack:
			rdr.render(self)
		# filter TODO
		
		# set the flags for the identification map
		self.ctx.enable_only(mgl.DEPTH_TEST)
		self.ctx.blend_func = mgl.ONE, mgl.ZERO
		self.ctx.blend_equation = mgl.FUNC_ADD
		# identify objects
		self.ident_frame.use()
		self.ident_frame.clear()
		ident = 1
		for i,(grp,rdr) in enumerate(self.stack):
			ident += rdr.identify(self, ident) or 0
			self.ident_steps[i] = ident-1
		self.refreshed = False
	
	def dequeue(self):
		if self.queue:
			# insert renderers for each object in queue
			for grp,obj in self.queue:
				for rdr in obj.display(self):
					for _ in range(len(self.layers), rdr.renderindex+1):	
						self.layers.append([])
					self.layers[rdr.renderindex].append((grp,rdr))
			self.restack()
			# empty the queue
			self.queue = []
	
	def restack(self):
		# regenerate the display stack
		self.stack = []
		for layer in self.layers:	self.stack.extend(layer)
		self.ident_steps = [0] * len(self.stack)
	
	def add(self, obj):
		''' add an object to the scene
			returns the group id created for the object's renderers 
		'''
		# find group id
		grp = 0
		while grp < len(self.groups) and grp == self.groups[grp]:		grp += 1
		# register object
		self.groups.insert(grp,grp)
		self.queue.append((grp,obj))
		return grp
	
	def remove(self, grp):
		''' remove an object from the scene and return its renderers if already created
			grp must be an integer returned by self.stack()
		'''
		# search for it in queue
		for i in range(len(self.queue)):
			if self.queue[i][0] == grp:	
				self.queue.pop(i)
				return ()
		# else remove already inserted renderers
		poped = []
		for layer in self.layers:
			i = 0
			while i < len(layer):
				if layer[i][0] == grp:	poped.append(layer.pop(i))
				else:					i += 1
		self.groups.remove(grp)
		self.restack()
		return poped
		
	
	def ressource(self, name, func=None):
		''' get a ressource loaded or load it using the function func.
			If func is not provided, an error is raised
		'''
		if name in self.ressources:	
			return self.ressources[name]
		elif callable(func):
			self.makeCurrent()	# set the scene context as current opengl context
			res = func(self)
			self.ressources[name] = res
			return res
		else:
			raise KeyError(f"ressource {repr(name)} doesn't exist or is not loaded")
	
	def resizeEvent(self, evt):
		super().resizeEvent(evt)
		self.init()
		self.update()
	
	def sizeref(self):
		''' size of the window to use for navigation purposes '''
		return self.size().width()
	
	def keyPressEvent(self, evt):
		k = evt.key()
		if	 k == Qt.Key_Control:	self.speckeys |= 0b01
		elif k == Qt.Key_Alt:		self.speckeys |= 0b10
		elif k == Qt.Key_Shift:		self.manipulator.slow(True)
	
	def keyReleaseEvent(self, evt):
		k = evt.key()
		if	 k == Qt.Key_Control:	self.speckeys &= ~0b01
		elif k == Qt.Key_Alt:		self.speckeys &= ~0b10
		elif k == Qt.Key_Shift:		self.manipulator.slow(False)
		
	def mousePressEvent(self, evt):
		self.update()
		if self.runtool(evt):		return
	
		x,y = evt.x(), evt.y()
		b = evt.button()
		# find the navigation current mode
		if b == Qt.LeftButton:
			self.mode = self.modes[self.speckeys]
		elif b == Qt.MiddleButton:
			self.mode = (self.manipulator.rotatestart, self.manipulator.rotating)
		# navigate if a mode is on
		if self.mode[0]:
			self.mouse_clicked = (x,y)	# movement origin
			self.mode[0]()
		else:
			# search for object interaction
			h,w = self.ident_frame.viewport[2:]
			clicked = self.objat((x,y), 10)
			if clicked: 
				grp,rdr = self.stack[clicked[0]]
				# right-click is the selection button
				if b == Qt.RightButton and hasattr(rdr, 'select'):
					#rdr.select(grp, not rdr.select(grp))
					rdr.select(clicked[1], not rdr.select(clicked[1]))
				# other clicks are for custom controls
				elif hasattr(rdr, 'control'):
					self.tool = rdr.control(self, grp, clicked[1], (x, y))

	def mouseMoveEvent(self, evt):
		self.update()
		if self.runtool(evt):		return
		if self.mode[1]:
			s = self.sizeref()
			ox, oy = self.mouse_clicked
			self.mode[1]((evt.x()-ox)/s, -(evt.y()-oy)/s)	# call the mode function with the coordinates relative to the movement start

	def mouseReleaseEvent(self, evt):
		if self.runtool(evt):		return
		self.mouse_clicked = (evt.x(), evt.y())
	
	def wheelEvent(self, evt):
		self.update()
		if self.runtool(evt):		return
		self.manipulator.zoom(-evt.angleDelta().y()/8 * pi/180)	# the 8 factor is there because of the Qt documentation
	
	# apply the given Qt event to the active tool
	def runtool(self, evt):
		if self.tool:
			return self.tool(self, evt)
	
	
	def refreshmaps(self):
		''' read self.ident_map and self.depth_map  from the GPU buffer 
			if already done since last render, returns immediately
		'''
		if not self.refreshed:
			self.makeCurrent()	# set the scene context as current opengl context
			self.ident_frame.read_into(self.ident_map, viewport=self.ident_frame.viewport, components=2)
			self.ident_frame.read_into(self.depth_map, viewport=self.ident_frame.viewport, components=1, attachment=-1, dtype='f4')
			self.refreshed = True
			#from PIL import Image
			#Image.fromarray(self.ident_map*10).show()
	
	def objat(self, coords, radius=0):
		''' return a tuple (obji, groupi) of the idents of the object and its group at the given screen coordinates '''
		self.refreshmaps()
		for x,y in snailaround(coords, (self.ident_map.shape[1], self.ident_map.shape[0]), radius):
			ident = int(self.ident_map[-y, x])
			if ident > 0:
				obji = dichotomy_index(self.ident_steps, ident)
				if obji == len(self.ident_steps):
					print('problem: object ident points out of idents list')
				while obji > 0 and self.ident_steps[obji-1] == ident:	obji -= 1
				if obji > 0:	groupi = ident - self.ident_steps[obji-1] - 1
				else:			groupi = ident - 1
				return obji, groupi
	
	def sight(self, coords):
		''' sight axis from the camera center to the point matching the given pixel coordinates '''
		x =  (coords[0]/viewport[2] *2 -1) /self.proj_matrix[0][0]
		y = -(coords[1]/viewport[3] *2 -1) /self.proj_matrix[1][1]
		m = affineInverse(self.view_matrix)
		return (vec3(m[3]), vec3(m[2]))

	
	def ptat(self, coords):
		''' return the point of the rendered surfaces that match the given window coordinates '''
		self.refreshmaps()
		viewport = self.ident_frame.viewport
		depthred = float(self.depth_map[-coords[1],coords[0]])
		x =  (coords[0]/viewport[2] *2 -1)
		y = -(coords[1]/viewport[3] *2 -1)
		
		if depthred == 1.0:
			return None
		else:
			a,b = self.proj_matrix[2][2], self.proj_matrix[3][2]
			depth = b/(depthred + a) * 0.53	# get the true depth  (can't get why there is a strange factor ... opengl trick)
			#near, far = self.projection.limits  or settings.display['view_limits']
			#depth = 2 * near / (far + near - depthred * (far - near))
			#print('depth', depth, depthred)
			return vec3(affineInverse(self.view_matrix) * fvec4(
						depth * x /self.proj_matrix[0][0],
						depth * y /self.proj_matrix[1][1],
						-depth,
						1))
	
	def ptfrom(self, coords, center):
		''' 3D point below the cursor in the plane orthogonal to the sight, with center as origin '''
		viewport = self.ident_frame.viewport
		x =  (coords[0]/viewport[2] *2 -1)
		y = -(coords[1]/viewport[3] *2 -1)
		depth = (self.view_matrix * fvec4(center,1))[2]
		return vec3(affineInverse(self.view_matrix) * fvec4(
					-depth * x /self.proj_matrix[0][0],
					-depth * y /self.proj_matrix[1][1],
					depth,
					1))
	
	def look(self, box):
		fov = self.projection.fov or settings.display['field_of_view']
		self.manipulator.center = fvec3(box.center)
		self.manipulator.distance = length(box.width) / (2*tan(fov/2))
		self.manipulator.update()

from .mathutils import dot, transpose, affineInverse

def snail(radius):
	''' generator of coordinates snailing around 0,0 '''
	x = 0
	y = 0
	for r in range(radius):
		for x in range(-r,r):		yield (x,-r)
		for y in range(-r,r):		yield (r, y)
		for x in reversed(range(-r,r)):	yield (x, r)
		for y in reversed(range(-r,r)):	yield (-r,y)

def snailaround(pt, box, radius):
	''' generator of coordinates snailing around pt, coordinates that goes out of the box are skipped '''
	cx,cy = pt
	mx,my = box
	for rx,ry in snail(radius):
		x,y = cx+rx, cy+ry
		if 0 <= x and x < mx and 0 <= y and y < my:
			yield x,y


def quickdisplay(objs):
	''' shortcut to create a QApplication showing only one view with the given objects inside.
		the functions returns when the window has been closed and all GUI destroyed
	'''
	import sys
	from PyQt5.QtCore import Qt, QCoreApplication
	from PyQt5.QtWidgets import QApplication
	
	QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
	app = QApplication(sys.argv)
	scn = Scene(objs)
	scn.show()
	err = app.exec()
	if err != 0:	print('Qt exited with code', err)


