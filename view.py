import settings
import numpy.core as np
import moderngl as mgl
from math import tan, sin, cos, pi, exp
from mathutils import fvec3, fvec4, fmat3, fmat4, row, column, length, perspective, translate, dichotomy_index
from PyQt5.QtCore import Qt
from PyQt5.QtOpenGL import QGLWidget, QGLFormat
from PyQt5.QtWidgets import QWidget
from PIL import Image
from copy import copy, deepcopy

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

class Scene(QGLWidget):
	''' Scene widget to display CAD objects 
		Attributes defined here:
			
		* objs			list of the objects to render, in the render order. The user can hack into it
		* projection
		* manipulator
		* ctx			the mgl context used for renders
	'''
	
	def __init__(self, parent=None, objects=(), projection=None, manipulator=None):
		# vieille version: QGLWidget, qui plante quand on écrit du texte
		fmt = QGLFormat()
		fmt.setVersion(*opengl_version)
		fmt.setProfile(QGLFormat.CoreProfile)
		fmt.setSampleBuffers(True)
		super().__init__(fmt, parent)
		# nouvelle version: QOpenGLWidget, qui fait des bugs d'affichage
		#super().__init__(parent)
		#fmt = QSurfaceFormat()
		#fmt.setVersion(*opengl_version)
		#fmt.setProfile(QSurfaceFormat.CoreProfile)
		#fmt.setSamples(4)
		#self.setFormat(fmt)
        
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
		self.tool = None
	
	
	def initializeGL(self):	pass

	def paintGL(self):
		self.ctx = mgl.create_context()
		self.screen = self.ctx.detect_framebuffer()	# old glwidget
		#self.screen = self.ctx.detect_framebuffer(self.defaultFramebufferObject()) # new glwidget
		self.ident_frame = self.ctx.simple_framebuffer((self.size().width(), self.size().height()), components=3, dtype='f1')
		self.ident_shader = self.ctx.program(
						vertex_shader=open('shaders/identification.vert').read(),
						fragment_shader=open('shaders/identification2.frag').read(),
						)
		self.init()
		self.render()
		self.paintGL = self.render

	def init(self):
		if not self.ctx:	return False
		
		w, h = self.size().width(), self.size().height()
		self.aspectratio = w/h
		self.ident_frame.viewport = (0, 0, w, h)
		self.ctx.viewport = (0, 0, w, h)
		#self.screen = self.ctx.detect_framebuffer(self.defaultFramebufferObject())
		#self.ident_map = bytearray(w*h*IDENT_SIZE)
		self.ident_map = np.empty((h,w), dtype='u2')
		
		return True

	def render(self):
		''' render the scene to the graphic buffer. need to be called from the opengl thread (often the main thread) '''
		
		# configure the additional objects
		i = 0
		while i < len(self.objs):
			if not hasattr(self.objs[i], 'render'):	
				self.objs.pop(i).display(self)
			else:	
				i += 1
		
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
		for rdr in self.objs:
			rdr.render(self)
		# filter TODO
		
		# set the flags for the identification map
		self.ctx.enable_only(mgl.DEPTH_TEST)
		self.ctx.blend_func = mgl.ONE, mgl.ZERO
		self.ctx.blend_equation = mgl.FUNC_ADD
		# identify objects
		self.ident_frame.use()
		self.ident_frame.clear()
		self.ident_steps = steps = [0]*len(self.objs)
		ident = 1
		for i,rdr in enumerate(self.objs):
			steps[i] = ident = ident + (rdr.identify(self, ident) or 0)
		self.ident_refreshed = True
		
		
	
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
		if self.tool:
			if not self.tool(self.transformevent(evt)):		return
	
		x,y = evt.x(), evt.y()
		b = evt.button()
		if b == Qt.LeftButton:
			self.mode = self.modes[self.speckeys]
		elif b == Qt.MiddleButton:
			self.mode = (self.manipulator.rotatestart, self.manipulator.rotating)
		
		if self.mode[0]:
			self.mouse_clicked = (x,y)	# movement origin
			self.mode[0]()
		else:
			h,w = self.ident_frame.viewport[2:]			
			clicked = self.objat((x,y))
			if clicked: 
				self.tool = self.objs[clicked[0]].control(self, clicked[1], (x/w, y/h))
		self.update()

	def mouseMoveEvent(self, evt):
		if self.tool:
			if not self.tool(self.transformevent(evt)):		return
		if self.mode[1]:
			s = self.sizeref()
			ox, oy = self.mouse_clicked
			self.mode[1]((evt.x()-ox)/s, -(evt.y()-oy)/s)	# call the mode function with the coordinates relative to the movement start
			self.update()

	def mouseReleaseEvent(self, evt):
		if self.tool:
			if not self.tool(self.transformevent(evt)):		return
		self.mouse_clicked = (evt.x(), evt.y())
	
	def wheelEvent(self, evt):
		if self.tool:
			if not self.tool(self.transformevent(evt)):		return
		self.manipulator.zoom(-evt.angleDelta().y()/8 * pi/180)	# the 8 factor is there because of the Qt documentation
		self.update()
	
	def objat(self, coords):
		if self.ident_refreshed:
			self.ident_frame.read_into(self.ident_map, viewport=self.ident_frame.viewport, components=2)
			self.ident_refreshed = False
		
		ident = self.ident_map[-coords[1],coords[0]]
		if ident:
			ident -= 1
			obji = dichotomy_index(self.ident_steps, ident)
			if obji > 0:	groupi = ident - self.ident_steps[obji-1]
			else:			groupi = ident
			return obji, groupi
		else:
			return None
	
	def look(self, box):
		fov = self.projection.fov or settings.display['field_of_view']
		self.manipulator.center = box.center
		self.manipulator.distance = length(box.width) / (2*tan(fov/2))
		self.manipulator.update()


class View(QWidget):
	# add some buttons and menus to customize the view
	
	def __init__(self, parent=None, scene=None):
		super().__init__(self, parent)
		self.scene = scene
		scene.setParent(self)



class SolidDisplay:
	''' renderer for solids
		positions is required and is used for the other arrays:
			- vertexnormals     associates a normal for each position, used when faces is defined
			- faces             is an array of indices for faces (nx3)
			- lines             is an array of indices for lines (nx2)
			- points            is an array of indices for points (n)
		color define the main color of the solid
	'''
	def __init__(self, scene, positions, 
			vertexnormals=None, 
			faces=None, 
			lines=None, 
			points=None, 
			idents=None, 
			selection=None,
			color=None):
		
		ctx = scene.ctx
		self.color = fvec3(color or settings.display['solid_color'])
		self.linecolor = settings.display['line_color']
		
		if selection:
			self.flags = np.array(selection, dtype='u1', copy=False)
		else:
			self.flags = np.zeros(len(idents), dtype='u1')
		self.flags_updated = True
		self.idents = idents
		self.displays = set()
		
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
			shader['reflectmap'] = 0
			shader['select_color'] = settings.display['select_color_face']
			return shader
		self.shader = scene.ressource('shader_solid', load)
		
		# load the line shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open('shaders/wire.vert').read(),
						fragment_shader=open('shaders/wire.frag').read(),
						)
			shader['select_color'] = settings.display['select_color_line']
			return shader
		self.lineshader = scene.ressource('shader_wire', load)
		
		self.ident_shader = scene.ident_shader
		
		### allocate buffers ##
		self.vb_positions = ctx.buffer(np.array(positions, dtype='f4', copy=False))
		self.vb_flags = ctx.buffer(self.flags)
		
		if faces is not None:
			if positions.shape[0] != vertexnormals.shape[0]:		
				raise ValueError('positions and normals must share the same length')
			if np.max(faces) > positions.shape[0]:
				raise IndexError('all indices in face must be a valid index for positions')
				
			self.vb_normals = ctx.buffer(np.array(vertexnormals, dtype='f4', copy=False))
			self.vb_faces = ctx.buffer(np.array(faces, dtype='u4', copy=False))
			self.va_faces = ctx.vertex_array(
					self.shader, 
					[	(self.vb_positions, '3f', 'v_position'), 
						(self.vb_normals, '3f', 'v_normal'),
						(self.vb_flags, 'u1', 'v_flags')],
					self.vb_faces,
					)
		else:
			self.va_faces = None
		
		if faces is not None and idents is not None:
			self.vb_idents = ctx.buffer(np.array(idents, dtype=IDENT_TYPE, copy=False))
			self.va_idents = ctx.vertex_array(
					scene.ident_shader, 
					[	(self.vb_positions, '3f', 'v_position'),
						(self.vb_idents, IDENT_TYPE, 'item_ident')], 
					self.vb_faces,
					)
			self.nidents = max(idents)+1
		else:
			self.va_idents = None
			
		
		if lines is not None:
			self.vb_lines = ctx.buffer(np.array(lines, dtype='u4', copy=False))
			self.va_lines = ctx.vertex_array(
					self.lineshader,
					[	(self.vb_positions, '3f', 'v_position'),
						(self.vb_flags, 'u1', 'v_flags')],
					self.vb_lines,
					)
		else:
			self.va_lines = None
		
		if points is not None:
			self.vb_points = ctx.buffer(np.array(points, dtype='u4', copy=False))
			self.va_points = ctx.vertex_array(
					self.lineshader,
					[(self.vb_positions, '3f', 'v_position')],
					self.vb_points,
					)
		else:
			self.va_points = None
		
		
	def render(self, scene):
		self.update(scene)
		
		if self.va_faces:
			# setup uniforms
			self.shader['min_color'].write(self.color * settings.display['solid_color_side'])
			self.shader['max_color'].write(self.color * settings.display['solid_color_front'])
			self.shader['refl_color'].write(self.color)
			self.shader['view'].write(scene.view_matrix)
			self.shader['proj'].write(scene.proj_matrix)
			# render on self.context
			self.reflectmap.use(0)
			self.va_faces.render(mgl.TRIANGLES)
		
		if self.va_lines:
			self.lineshader['color'] = self.linecolor
			self.lineshader['view'].write(scene.view_matrix)
			self.lineshader['proj'].write(scene.proj_matrix)
			self.va_lines.render(mgl.LINES)
		
		if self.va_points:
			scene.ctx.point_size = 3
			self.va_points.render(mgl.POINTS)
		
	def identify(self, scene, startident):
		if self.va_idents:
			scene.ident_shader['start_ident'] = startident
			scene.ident_shader['view'].write(scene.view_matrix)
			scene.ident_shader['proj'].write(scene.proj_matrix)
			self.va_idents.render(mgl.TRIANGLES)
			return self.nidents
	
	def control(self, scene, ident, evt):
		self.select(ident, not self.select(ident))
		return None
	
	def select(self, idents, state=None):
		mask = 0b1
		if state is None:	return self.flags[idents] & mask
		if state:	self.flags[idents] |= mask
		else:		self.flags[idents] &= ~mask
		self.flags_updated = True
	
	def update(self, scene):
		if self.flags_updated:
			self.vb_flags.write(self.flags[self.idents])
			self.flags_updated = False


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
	
