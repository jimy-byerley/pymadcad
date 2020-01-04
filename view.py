import settings
import numpy as np
from numpy import float32
import moderngl
from math import tan, sin, cos, pi, exp
from PyQt5.QtCore import Qt
from PyQt5.QtOpenGL import QGLWidget, QGLFormat
from PyQt5.QtWidgets import QWidget
from PIL import Image
from copy import copy, deepcopy

from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtGui import QSurfaceFormat


opengl_version = (3,3)	# min (3,3)


def perspective(aspectratio, fov=None, limits=None, dtype=np.float32):
	'''
		aspectratio:	the ratio height/width of the view
		fov:	field of view (rad)
		limits:	min and max distance to display
	'''
	if not limits:	limits = settings.display['view_limits']
	if not fov:		fov = settings.display['field_of_view']
	f = 1 / tan(fov / 2)
	znear, zfar = limits
	return  np.array([
				[f/aspectratio,		0.,						0.,			0.],
				[			0.,		f,						0.,			0.],
				[			0.,		0.,		(zfar+znear)/(znear-zfar), (2*zfar*znear)/(znear-zfar)],
				[			0.,		0.,	                   -1,			0.],
				], dtype=dtype)

def orthographic(aspectratio, size, limits=None, dtype=np.float32):
	''' 
		aspectratio:	the ratio height/width of the view
		size:	the size of the view (distance) 
	'''
	if not limits:	limits = settings.display['view_limits']
	znear, zfar = limits
	return np.array([
				[1/size/aspectratio,	0.,		0.,					0.],
				[			0.,			1/size,	0.,					0.],
				[			0.,			0.,		2/(znear-zfar),		(zfar+znear)/(znear-zfar)],
				[			0.,			0.,		0.,					1.],
				], dtype=dtype)

class Perspective:
	''' class to provide the perspective matrix '''
	__slots__ = ('fov', 'limits')
	def __init__(self, fov=None, limits=None):
		self.limits, self.fov = limits, fov
	def matrix(self, ratio):
		return perspective(ratio, self.fov, self.limits, dtype=np.float32)
		
class Turntable:
	''' class to rotate the view using the mouse events '''
	def __init__(self):
		self.center = [0,0,0]
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
		f = settings.controls['pan_sensitivity'] * self.speed
		self.center = self.old_center + self.mat[0,:3] * f*x + self.mat[1,:3] * f*y
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
						  np.array([[1.,         0.,          0.],
									[0.,  cos(roty),  -sin(roty)],
									[0.,  sin(roty),   cos(roty)]], dtype=float32)
						@ np.array([[cos(rotx),  -sin(rotx),  0.],
									[sin(rotx),   cos(rotx),  0.],
									[0.,          0.,         1.]], dtype=float32)
					)
		self.mat = mat = np.identity(4, dtype=float32)
		mat[:3,:3] = rotation
		mat[:3,3] = rotation @ self.center
		mat[2,3] -= self.distance
	
	def matrix(self):	return self.mat



IDENT_TYPE = 'u2'
IDENT_SIZE = int(IDENT_TYPE[1:]) * 8

class Scene(QOpenGLWidget):
	''' Scene widget to display CAD objects 
		Attributes defined here:
			
		* objs			list of the objects to render, in the render order. The user can hack into it
		* projection
		* manipulator
		* ctx			the moderngl context used for renders
	'''
	
	def __init__(self, parent=None, objects=(), projection=None, manipulator=None):
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
        
		self.projection = projection or Perspective()
		self.manipulator = manipulator or Turntable()
		self.objs = list(objects)	# objects to render, in the render order, the user can hack into it
		self.ressources = {}
		
		self.drag = False
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
		self.modelock = False
	
	
	def initializeGL(self):	pass

	def paintGL(self):
		self.ctx = moderngl.create_context()
		self.screen = self.ctx.detect_framebuffer()
		self.ident_shader = self.ctx.program(
						vertex_shader=open('shaders/identification.vert').read(),
						fragment_shader=open('shaders/identification.frag').read(),
						)
		self.init()
		self.render()
		self.paintGL = self.render

	def init(self):
		if not self.ctx:	return False
		
		w, h = self.size().width(), self.size().height()
		self.aspectratio = w/h
		self.ctx.viewport = (0, 0, w, h)
		self.ident_frame = self.ctx.simple_framebuffer((w,h), 1, samples=0, dtype=IDENT_TYPE)
		self.ident_map = bytearray(w*h*IDENT_SIZE)
		
		return True

	def render(self):
		''' render the scene to the graphic buffer. need to be called from the opengl thread (often the main thread) '''
		
		# set the default flags for the scene
		self.ctx.multisample = True
		self.ctx.enable(moderngl.BLEND | moderngl.DEPTH_TEST)
		self.ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA
		self.ctx.blend_equation = moderngl.FUNC_ADD
		
		# configure the additional objects
		i = 0
		while i < len(self.objs):
			if not hasattr(self.objs[i], 'render'):	
				self.objs[i] = self.objs[i].display(self)
			i += 1
		
		# setup the render pass
		self.view_matrix = self.manipulator.matrix().T.tobytes()	# transpose to get a column major matrix (for opengl)
		self.proj_matrix = self.projection.matrix(self.aspectratio).T.tobytes()	# transpose to get a column major matrix (for opengl)
		
		# identification map
		self.ident_frame.use()
		self.ident_frame.clear()
		for ident,rdr in enumerate(self.objs):
			rdr.identify(self, ident)
		
		# render objects
		self.screen.use()
		self.screen.clear()
		for rdr in self.objs:
			rdr.render(self)
		
		# filter TODO
	
	def ressource(self, name, func=None):
		''' get a ressource loaded or load it using the function func.
			If func is not provided, an error is raised
		'''
		if name in self.ressources:	
			return self.ressources[name]
		elif callable(func):
			res = func(self)
			self.ressources[name] = res
			return res
		else:
			raise KeyError(f"ressource {repr(name)} doesn't exist or is not loaded")
	
	def resizeEvent(self, evt):
		self.init()
	
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
		b = evt.button()
		if b == Qt.LeftButton:
			self.mode = self.modes[self.speckeys]
		elif b == Qt.MiddleButton:
			self.mode = (self.manipulator.rotatestart, self.manipulator.rotating)
		
		if self.mode[0]:
			self.mouse_clicked = (evt.x(), evt.y())	# movement origin
			self.mode[0]()
		#else:
			#s = 5
			#self.ident_frame.read_into(self.ident_map, components=1, dtype=IDENT_TYPE)
			#img = np.frombuffer(self.ident_map, dtype=IDENT_TYPE)
			#x,y = evt.x(), evt.y()
			#print(img.reshape((self.size().height(), self.size().width()))[y-s:y+s,x-s:x+s])
		self.update()

	def mouseMoveEvent(self, evt):
		if self.mode[1]:
			s = self.sizeref()
			ox, oy = self.mouse_clicked
			self.mode[1]((evt.x()-ox)/s, -(evt.y()-oy)/s)	# call the mode function with the coordinates relative to the movement start
			self.update()

	def mouseReleaseEvent(self, evt):
		self.mode = self.modes[self.speckeys]
		if self.mode[0]:
			s = self.sizeref()
			self.mouse_clicked = (evt.x(), evt.y())
			self.mode[0]()
			self.update()
	
	def wheelEvent(self, evt):
		self.manipulator.zoom(-evt.angleDelta().y()/8 * pi/180)	# the 8 factor is there because of the Qt documentation
		self.update()

class View(QWidget):
	# add some buttons and menus to customize the view
	
	def __init__(self, parent=None, scene=None):
		super().__init__(self, parent)
		self.scene = scene
		scene.setParent(self)



class SolidDisplay:
	def __init__(self, scene, points, vertexnormals, faces=None, lines=None, color=None):
		ctx = scene.ctx
		self.color = color or settings.display['solid_color']
		self.linecolor = settings.display['line_color']
		
		def load(scene):
			img = Image.open('textures/skybox.png')
			return scene.ctx.texture(img.size, 4, img.tobytes())
		self.reflectmap = scene.ressource('skybox', load)
		
		# load the shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open('shaders/solid.vert').read(),
						fragment_shader=open('shaders/solid.frag').read(),
						)
			# setup some uniforms
			shader['reflectmap'].value = 0
			return shader
		self.shader = scene.ressource('shader_solid', load)
		
		# load the line shader
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open('shaders/wire.vert').read(),
						fragment_shader=open('shaders/uniformcolor.frag').read(),
						)
		self.lineshader = scene.ressource('shader_uniformcolor', load)
		
		self.ident_shader = scene.ident_shader
		
		# allocate buffers
		if points.shape[0] != vertexnormals.shape[0]:		
			raise ValueError('points and normals must share the same length')
		self.vb_points = ctx.buffer(np.array(points, dtype=np.float32, copy=False))
		self.vb_normals = ctx.buffer(np.array(vertexnormals, dtype=np.float32, copy=False))
		
		if faces is not None:
			if np.max(faces) > points.shape[0]:
				raise IndexError('all indices in face must be a valid index for points')
			self.vb_faces = ctx.buffer(np.array(faces, dtype=np.uint32, copy=False))
			self.va_faces = ctx.vertex_array(
					self.shader, 
					[	(self.vb_points, '3f', 'v_position'), 
						(self.vb_normals, '3f', 'v_normal')], 
					self.vb_faces,
					)
			self.va_ident = ctx.vertex_array(
					scene.ident_shader, 
					[(self.vb_points, '3f', 'v_position')], 
					self.vb_faces,
					)
		else:
			self.va_faces = None
			self.va_ident = None
		
		if lines is not None:
			self.vb_lines = ctx.buffer(np.array(lines, dtype=np.uint32, copy=False))
			self.va_lines = ctx.vertex_array(
					self.lineshader,
					[(self.vb_points, '3f', 'v_position')],
					self.vb_lines,
					)
		else:
			self.va_lines = None
		
		
	def render(self, scene):
		if self.va_faces:
			# setup uniforms
			c = self.color
			f = 0.2
			self.shader['min_color'].value = (c[0]*f, c[1]*f, c[2]*f)
			self.shader['max_color'].value = c
			self.shader['refl_color'].value = c
			self.shader['view'].write(scene.view_matrix)
			self.shader['proj'].write(scene.proj_matrix)
			# render on self.context
			self.reflectmap.use(0)
			self.va_faces.render(moderngl.TRIANGLES)
		
		if self.va_lines:
			self.lineshader['color'].value = self.linecolor
			self.lineshader['view'].write(scene.view_matrix)
			self.lineshader['proj'].write(scene.proj_matrix)
			self.va_lines.render(moderngl.LINES)
			scene.ctx.point_size = 3
			self.va_lines.render(moderngl.POINTS)
		
	def identify(self, scene, ident):
		if self.va_ident:
			scene.ident_shader['ident'].value = ident
			scene.ident_shader['view'].write(scene.view_matrix)
			scene.ident_shader['proj'].write(scene.proj_matrix)
			self.va_ident.render(moderngl.TRIANGLES)


if __name__ == '__main__':
	import sys
	from PyQt5.QtWidgets import QApplication
	
	class Mesh:
		def __init__(self, points, faces, facenormals):
			self.points, self.faces, self.facenormals = points, faces, facenormals
		def display(self, scene):
			return SolidDisplay(scene,
				self.points[self.faces].reshape((self.faces.shape[0]*3,3)),
				np.hstack((self.facenormals, self.facenormals, self.facenormals)).reshape((self.faces.shape[0]*3,3)),
				np.array(range(3*self.faces.shape[0]), dtype=np.uint32).reshape(self.faces.shape),
				)
	
	m = Mesh(
		np.array([
			1.0, -1.0, -1.0,
            1.0, -1.0, 1.0,
            -1.0, -1.0, 1.0,
            -1.0, -1.0, -1.0,
            1.0, 1.0, -1.0,
            1.0, 1.0, 1.0,
            -1.0, 1.0, 1.0,
            -1.0, 1.0, -1.0]).reshape((8,3)),
		np.array([
            0, 1, 2,
            0, 2, 3,
            4, 7, 6,
            4, 6, 5,
            0, 4, 5,
            0, 5, 1,
            1, 5, 6,
            1, 6, 2,
            2, 6, 7,
            2, 7, 3,
            4, 0, 3,
            4, 3, 7], dtype='u4').reshape((12,3)),
		np.array([
			 0, -1,  0,
			 0, -1,  0,
			 0,  1,  0,
			 0,  1,  0,
			 1,  0,  0,
			 1,  0,  0,
			-1,  0,  0,
			-1,  0,  0,
			 0,  0,  1,
			 0,  0,  1,
			 0,  0, -1,
			 0,  0, -1,
			 ]).reshape((12,3)),
		)

	app = QApplication(sys.argv)
	scn = Scene()
	scn.objs.append(m)
	
	scn.show()
	sys.exit(app.exec())
	
