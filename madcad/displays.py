# This file is part of pymadcad,  distributed under license LGPL v3

from math import log, exp, floor
from .mathutils import (vec3, fvec3, fvec4, fmat4, 
						mix, normalize, length, distance, dot, noproject, dirbase, transform,
						Box, isnan, isinf,
						)
from .rendering import Display, overrides, writeproperty
from .common import ressourcedir
from . import settings
from . import primitives
from PIL import Image
import numpy.core as np
import moderngl as mgl

from PyQt5.QtCore import Qt, QEvent


def shader_wire(scene):
	return scene.ctx.program(
				vertex_shader=open(ressourcedir+'/shaders/wire.vert').read(),
				fragment_shader=open(ressourcedir+'/shaders/wire.frag').read(),
				)
	
def shader_uniformcolor(scene):
	return scene.ctx.program(
				vertex_shader=open(ressourcedir+'/shaders/uniformcolor.vert').read(),
				fragment_shader=open(ressourcedir+'/shaders/uniformcolor.frag').read(),
				)


class PointDisplay(Display):
	def __init__(self, scene, position, size=10, color=None):
		self.position = fvec3(position)
		self.size = size
		self.selected = False
		self.color = fvec3(color or settings.display['line_color'])
		
		def load(scene):
			img = Image.open(ressourcedir+'/textures/point.png')
			texture = scene.ctx.texture(img.size, 1, img.convert('L').tobytes())
			#self.texture = scene.ressource('pointtex', load)
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/pointhalo.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/pointhalo.frag').read(),
						)
			shader['halotex'].value = 0
			ident_shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/pointhalo-ident.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/ident.frag').read(),
						)
			#self.shader = scene.ressource('pointhalo', load)
			vb = scene.ctx.buffer(np.array([(0,0), (0,1), (1,1), (0,0), (1,1), (1,0)], 'f4'))
			va = scene.ctx.vertex_array(shader, [(vb, '2f', 'v_uv')])
			va_ident = scene.ctx.vertex_array(ident_shader, [(vb, '2f', 'v_uv')])
			return texture, shader, va, ident_shader, va_ident
		
		(	self.texture, 
			self.shader, 
			self.va, 
			self.ident_shader, 
			self.va_ident	) = scene.ressource('pointhalo', load)
			
	@property
	def box(self):
		return Box(center=self.position, width=fvec3(0))
	
	def render(self, view):
		self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
		self.shader['position'].write(fvec3(self.world * fvec4(self.position,1)))
		self.shader['view'].write(view.uniforms['view'])
		self.shader['proj'].write(view.uniforms['proj'])
		self.shader['ratio'] = (
				self.size / view.width(),
				self.size / view.height(),
				)
		self.texture.use(0)
		self.va.render(mgl.TRIANGLES)

	def identify(self, view):
		self.ident_shader['ident'] = view.identstep(1)
		self.ident_shader['position'].write(fvec3(self.world * fvec4(self.position,1)))
		self.ident_shader['view'].write(view.uniforms['view'])
		self.ident_shader['proj'].write(view.uniforms['proj'])
		self.ident_shader['ratio'] = (
				1.5 * self.size / view.width(),
				1.5 * self.size / view.height(),
				)
		self.va_ident.render(mgl.TRIANGLES)
	
	def stack(self, scene):
		return ( ((), 'ident', 2, self.identify),
				 ((), 'screen', 2, self.render))


class AxisDisplay(Display):
	pattern = [0, 0.25, 0.45, 0.55, 0.75, 1]
	repetitions = 3
	def __init__(self, scene, axis, interval=None, color=None, pose=fmat4(1)):
		self.origin = fvec3(axis[0])
		self.direction = fvec3(axis[1])
		self.interval = interval
		self.color = fvec3(color or settings.display['line_color'])
		self.selected = False
		self.box = Box(center=self.origin, width=fvec3(0))
		
		self.shader, self.va, self.ident_shader, self.va_ident = scene.ressource('axis', self.load)
	
	def load(self, scene):
		shader = scene.ctx.program(
					vertex_shader=open(ressourcedir+'/shaders/axis.vert').read(),
					fragment_shader=open(ressourcedir+'/shaders/axis.frag').read(),
					)
		ident_shader = scene.ctx.program(
					vertex_shader=open(ressourcedir+'/shaders/axis-ident.vert').read(),
					fragment_shader=open(ressourcedir+'/shaders/ident.frag').read(),
					)
		pts = []
		for i in range(-1, self.repetitions+1):
			for pt in self.pattern:
				if i == -1:					alpha = pt
				elif i == self.repetitions:	alpha = 1-pt
				else:						alpha = 1
				pts.append(((pt+i)/self.repetitions, alpha))
		vb = scene.ctx.buffer(np.array(pts, 'f4'))
		va = scene.ctx.vertex_array(shader, [(vb, 'f f', 'v_absciss', 'v_alpha')])
		va_ident = scene.ctx.vertex_array(ident_shader, [(vb, 'f 12x', 'v_absciss')])
		return shader, va, ident_shader, va_ident
	
	def _disp_interval(self, view):
		if self.interval:	return self.interval
		else:
			size = -view.uniforms['view'][3][2]/6
			return (-0.5*size, size)
	
	def render(self, view):
		self.shader['projview'].write(view.uniforms['projview'])
		self.shader['world'].write(self.world)
		self.shader['origin'].write(self.origin)
		self.shader['direction'].write(self.direction)
		self.shader['interval'] = self._disp_interval(view)
		self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
		self.va.render(mgl.LINES)
	
	def identify(self, view):
		self.ident_shader['projview'].write(view.uniforms['projview'])
		self.ident_shader['world'].write(self.world)
		self.ident_shader['origin'].write(self.origin)
		self.ident_shader['direction'].write(self.direction)
		self.ident_shader['interval'] = self._disp_interval(view)
		self.ident_shader['ident'] = view.identstep(1)
		self.va_ident.render(mgl.LINES)
	
	def stack(self, scene):
		return ( ((), 'ident', 2, self.identify),
				 ((), 'screen', 2, self.render))

def npboundingbox(points):
	''' boundingbox for numpy arrays of points on the 3 first components '''
	return Box(
				fvec3([float(np.min(points[:,i])) for i in range(3)]),
				fvec3([float(np.max(points[:,i])) for i in range(3)]),
				)

class AnnotationDisplay(Display):
	def __init__(self, scene, points, color):
		self.color = fvec3(color or settings.display['annotation_color'])
		self.selected = False
		self.box = npboundingbox(points)
		# load shader
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/annotation.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/annotation.frag').read(),
						)
		self.shader = scene.ressource('shader_annotation', load)
		# allocate buffers
		self.vb_pts = scene.ctx.buffer(points)
		self.va = scene.ctx.vertex_array(self.shader, [(self.vb_pts, '3f f', 'v_position', 'v_alpha')])
		self.va_ident = scene.ctx.vertex_array(scene.ressource('shader_ident'), [(self.vb_pts, '3f 4x', 'v_position')])
		
	def __del__(self):
		self.va.release()
		self.va_ident.release()
		self.vb_pts.release()
	
	@staticmethod
	def buff_ptsalpha(points, alpha):
		return np.hstack((
				np.array([tuple(p) for p in points], 'f4'), 
				np.array(alpha, 'f4', ndmin=2).transpose(),
				))

	def render(self, view):
		self.shader['proj'].write(view.uniforms['proj'])
		self.shader['view'].write(view.uniforms['view'] * self.world)
		self.shader['color'].write(self.color if not self.selected else fvec3(settings.display['select_color_line']))
		self.va.render(mgl.LINES)
	
	def identify(self, view):
		shader = view.scene.ressource('shader_ident')
		shader['proj'].write(view.uniforms['proj'])
		shader['view'].write(view.uniforms['view'] * self.world)
		shader['ident'] = view.identstep(1)
		self.va_ident.render(mgl.LINES)
		
	def stack(self, scene):
		return ( ((), 'ident', 2, self.identify),
				 ((), 'screen', 2, self.render)) 


class BoxDisplay(AnnotationDisplay):
	def __init__(self, scene, box, color=None):
		# place points
		x,y,z = box.width
		c = 0.1*min(x,y,z)	# corner
		o = 0.4 # wire alpha
		pts = np.array([
			# corner
			(0,0,0,1),(c,0,0,1),
			(0,0,0,1),(0,c,0,1),
			(0,0,0,1),(0,0,2*c,1),
			
			# box wire
			(0,0,0,o),(x,0,0,o),
			(0,0,0,o),(0,y,0,o),
			(0,0,0,o),(0,0,z,o),
			
			(x,0,0,o),(x,y,0,o),
			(x,0,0,o),(x,0,z,o),
			(0,y,0,o),(x,y,0,o),
			(0,y,0,o),(0,y,z,o),
			(0,0,z,o),(x,0,z,o),
			(0,0,z,o),(0,y,z,o),
			
			(x,y,0,o),(x,y,z,o),
			(x,0,z,o),(x,y,z,o),
			(0,y,z,o),(x,y,z,o),
			], dtype='f4')
		pts += (*box.min, 0)
		self.textplace = box.min

		super().__init__(scene, pts, color)

from .mathutils import norminf, length2

class SolidDisplay(Display):
	''' Display render Meshes '''
	def __init__(self, scene, positions, normals, faces, lines, idents, color=None):
		self.box = npboundingbox(positions)
		self.options = scene.options
		
		s = settings.display
		color = fvec3(color or s['solid_color'])
		line = (	(length(s['line_color']) + 2*length(color-s['solid_color'])) 
					/ length2(color + 0.5*s['background_color']) 
					* normalize(color + 1e-6)  )
		#if length(s['line_color']) > length(color)
		reflect = normalize(color + 1e-6) * s['solid_reflectivity']
		
		self.vertices = Vertices(scene.ctx, positions, idents)
		self.disp_faces = FacesDisplay(scene, self.vertices, normals, faces, color=color, reflect=reflect, layer=0)
		self.disp_ghost = GhostDisplay(scene, self.vertices, normals, faces, color=line, layer=0)
		self.disp_groups = LinesDisplay(scene, self.vertices, lines, color=line, alpha=1, layer=-2e-6)
		self.disp_points = PointsDisplay(scene, self.vertices, range(len(positions)), layer=-3e-6)
		wire = []
		for f in faces:
			wire.append((f[0], f[1]))
			wire.append((f[1], f[2]))
			wire.append((f[2], f[0]))
		self.disp_wire = LinesDisplay(scene, self.vertices, wire, color=line, alpha=0.3, layer=-1e-6)
		
	def stack(self, scene):
		yield ((), 'screen', -1, self.vertices.prerender)
		if self.options['display_faces']:	
			yield ((), 'screen', 0, self.disp_faces.render)
			yield ((), 'ident', 0, self.disp_faces.identify)
		else:								
			yield ((), 'screen', 1, self.disp_ghost.render)
			yield ((), 'ident', 0, self.disp_ghost.identify)
		if self.options['display_groups']:	yield ((), 'screen', 1, self.disp_groups.render)
		if self.options['display_points']:	yield ((), 'screen', 2, self.disp_points.render)
		if self.options['display_wire']:	yield ((), 'screen', 2, self.disp_wire.render)
	
	@property
	def world(self):	return self.vertices.world
	@world.setter
	def world(self, value):	self.vertices.world = value

	#def control(self, view, key, sub, evt):
		#if evt.type() == QEvent.MouseButtonRelease and evt.button() == Qt.LeftButton:
			#sub = sub[0]
			#flags, idents = self.vertices.flags, self.vertices.idents
			#for i in range(len(idents)):
				#flags[i] ^= idents[i] == sub
			#self.vertices.flags_updated = True
			#view.update()
			#evt.accept()
	

class WebDisplay(Display):
	''' Display to render Webs '''
	def __init__(self, scene, positions, lines, points, idents, color=None):
		self.box = npboundingbox(positions)
		self.options = scene.options
		color = color or settings.display['line_color']
		self.vertices = Vertices(scene.ctx, positions, idents)
		self.disp_edges = LinesDisplay(scene, self.vertices, lines, color=color, alpha=1, layer=-2e-6)
		self.disp_groups = PointsDisplay(scene, self.vertices, points, layer=-3e-6)
		self.disp_points = PointsDisplay(scene, self.vertices, range(len(positions)), layer=-1e-6)
		

	def stack(self, scene):
		yield ((), 'screen', -1, self.vertices.prerender)
		if self.options['display_groups']:		yield ((), 'screen', 2, self.disp_groups.render)
		if self.options['display_points']:		yield ((), 'screen', 2, self.disp_points.render)
		yield ((), 'screen', 1, self.disp_edges.render)
		yield ((), 'ident', 1, self.disp_edges.identify)
	
	@property
	def world(self):	return self.vertices.world
	@world.setter
	def world(self, value):	self.vertices.world = value

	#def control(self, view, key, sub, evt):
		#if evt.type() == QEvent.MouseButtonRelease and evt.button() == Qt.LeftButton:
			#sub = sub[0]
			#flags, idents = self.vertices.flags, self.vertices.idents
			#for i in range(len(idents)):
				#flags[i] ^= idents[i] == sub
			#self.vertices.flags_updated = True
			#view.update()
			#evt.accept()


class Vertices(object):
	''' convenient class to share vertices between SolidDisplay, WebDisplay, PointsDisplay '''
	def __init__(self, ctx, positions, idents):
		self.idents = idents
		self.nident = int(max(idents))+1
		self.flags = np.zeros(len(positions), dtype='u1')
		self.flags_updated = False
		assert len(idents) == len(positions)
		self.vb_positions = ctx.buffer(np.array(positions, dtype='f4', copy=False))
		self.vb_idents = ctx.buffer(np.array(idents, dtype='u2', copy=False))
		self.vb_flags = self.vb_flags = ctx.buffer(self.flags, dynamic=True)
		self.world = fmat4(1)
		
	def __del__(self):
		self.vb_positions.release()
		self.vb_idents.release()
		self.vb_flags.release()
		
	def prerender(self, view):
		if self.flags_updated:
			self.vb_flags.write(self.flags)
			self.flags_updated = False
	
	def selectsub(self, sub):
		for i,id in enumerate(self.idents):
			self.flags[i] ^= id == sub
		self.flags_updated = True
			


class FacesDisplay:
	def __init__(self, scene, vertices, normals, faces, color, reflect, layer=0):
		self.color = color
		self.layer = layer
		self.reflect = reflect
		self.vertices = vertices
	
		# load the skybox texture
		def load(scene):
			img = Image.open(ressourcedir+'/textures/'+settings.display['solid_reflect'])
			return scene.ctx.texture(img.size, 3, img.tobytes())
		self.reflectmap = scene.ressource('skybox', load)
		
		# load the shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/solid.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/solid.frag').read(),
						)
			# setup some uniforms
			shader['reflectmap'] = 0
			return shader
		self.shader = scene.ressource('shader_solid', load)
		self.ident_shader = scene.ressource('shader_subident')
		# allocate buffers
		if faces is not None and len(faces) and vertices.vb_positions:
			self.vb_faces = scene.ctx.buffer(np.array(faces, 'u4', copy=False))
			self.vb_normals = scene.ctx.buffer(np.array(normals, 'f4', copy=False))
			self.va = scene.ctx.vertex_array(
					self.shader, 
					[	(vertices.vb_positions, '3f', 'v_position'), 
						(self.vb_normals, '3f', 'v_normal'),
						(vertices.vb_flags, 'u1', 'v_flags')],
					self.vb_faces,
					)
			
			self.va_ident = scene.ctx.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_faces,
					)
		else:
			self.va = None
			
	def __del__(self):
		if self.va:
			self.va.release()
			self.va_ident.release()
			self.vb_faces.release()
			self.vb_normals.release()
	
	def render(self, view):
		if self.va:
			# setup uniforms
			self.shader['select_color'].write(settings.display['select_color_face'])
			self.shader['min_color'].write(self.color * settings.display['solid_color_side'])
			self.shader['max_color'].write(self.color * settings.display['solid_color_front'])
			self.shader['refl_color'].write(self.reflect)
			self.shader['layer'] = self.layer
			self.shader['world'].write(self.vertices.world)
			self.shader['view'].write(view.uniforms['view'])
			self.shader['proj'].write(view.uniforms['proj'])
			# render on self.context
			self.reflectmap.use(0)
			self.va.render(mgl.TRIANGLES)
	
	def identify(self, view):
		if self.va:
			self.ident_shader['layer'] = self.layer
			self.ident_shader['start_ident'] = view.identstep(self.vertices.nident)
			self.ident_shader['view'].write(view.uniforms['view'] * self.vertices.world)
			self.ident_shader['proj'].write(view.uniforms['proj'])
			# render on self.context
			self.va_ident.render(mgl.TRIANGLES)

class GhostDisplay:
	def __init__(self, scene, vertices, normals, faces, color, layer=0):
		self.color = color
		self.layer = layer
		self.vertices = vertices
		
		# load the shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/solid.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/ghost.frag').read(),
						)
			# setup some uniforms
			return shader
		self.shader = scene.ressource('shader_ghost', load)
		self.ident_shader = scene.ressource('shader_subident')
		# allocate buffers
		if faces is not None and len(faces) and vertices.vb_positions:
			self.vb_faces = scene.ctx.buffer(np.array(faces, 'u4', copy=False))
			self.vb_normals = scene.ctx.buffer(np.array(normals, 'f4', copy=False))
			self.va = scene.ctx.vertex_array(
					self.shader, 
					[	(vertices.vb_positions, '3f', 'v_position'), 
						(self.vb_normals, '3f', 'v_normal'),
						(vertices.vb_flags, 'u1', 'v_flags')],
					self.vb_faces,
					)
			
			self.va_ident = scene.ctx.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_faces,
					)
		else:
			self.va = None
			
	def __del__(self):
		if self.va:
			self.va.release()
			self.va_ident.release()
			self.vb_faces.release()
			self.vb_normals.release()
	
	def render(self, view):
		if self.va:
			# setup uniforms
			self.shader['select_color'].write(settings.display['select_color_line'])
			self.shader['normal_color'].write(self.color)
			self.shader['world'].write(self.vertices.world)
			self.shader['view'].write(view.uniforms['view'])
			self.shader['proj'].write(view.uniforms['proj'])
			self.shader['layer'] = self.layer
			view.scene.ctx.disable(mgl.DEPTH_TEST)
			# render on self.context
			self.va.render(mgl.TRIANGLES)
			view.scene.ctx.enable(mgl.DEPTH_TEST)
	
	def identify(self, view):
		if self.va:
			self.ident_shader['start_ident'] = view.identstep(self.vertices.nident)
			self.ident_shader['view'].write(view.uniforms['view'] * self.vertices.world)
			self.ident_shader['proj'].write(view.uniforms['proj'])
			self.ident_shader['layer'] = self.layer
			# render on self.context
			self.va_ident.render(mgl.TRIANGLES)
	
class LinesDisplay:
	def __init__(self, scene, vertices, lines, color, alpha=1, layer=0):
		self.layer = layer
		self.color = fvec4(fvec3(color), alpha)
		self.select_color = fvec4(settings.display['select_color_line'], alpha)
		self.vertices = vertices
		
		# load the line shader
		self.shader = scene.ressource('shader_wire', shader_wire)
		self.ident_shader = scene.ressource('shader_subident')
		if lines is not None and len(lines) and vertices.vb_positions:
			# allocate buffers
			self.vb_lines = scene.ctx.buffer(np.array(lines, dtype='u4', copy=False))
			self.va = scene.ctx.vertex_array(
						self.shader,
						[	(vertices.vb_positions, '3f', 'v_position'),
							(vertices.vb_flags, 'u1', 'v_flags')],
						self.vb_lines,
						)
			self.va_ident = scene.ctx.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_lines,
					)
		else:
			self.va = None
			
	def __del__(self):
		if self.va:
			self.va.release()
			self.va_ident.release()
			self.vb_lines.release()
	
	def render(self, view):
		if self.va:
			self.shader['color'].write(self.color)
			self.shader['select_color'].write(self.select_color)
			self.shader['view'].write(view.uniforms['view'] * self.vertices.world)
			self.shader['proj'].write(view.uniforms['proj'])
			self.shader['layer'] = self.layer
			self.va.render(mgl.LINES)
		
	def identify(self, view):
		if self.va:
			self.ident_shader['start_ident'] = view.identstep(self.vertices.nident)
			self.ident_shader['view'].write(view.uniforms['view'] * self.vertices.world)
			self.ident_shader['proj'].write(view.uniforms['proj'])
			self.ident_shader['layer'] = self.layer
			self.va_ident.render(mgl.LINES)
		
class PointsDisplay:
	def __init__(self, scene, vertices, indices=None, color=None, ptsize=3, layer=0):
		self.color = fvec4(color or settings.display['point_color'], 1)
		self.select_color = fvec4(settings.display['select_color_line'], 1)
		self.ptsize = ptsize
		self.layer = layer
		self.vertices = vertices
		
		# load the line shader
		self.shader = scene.ressource('shader_wire', shader_wire)
		self.ident_shader = scene.ressource('shader_subident')
		# allocate GPU objects
		if indices is not None and len(indices) and vertices.vb_positions:
			self.vb_indices = scene.ctx.buffer(np.array(indices, dtype='u4', copy=False))
			self.va = scene.ctx.vertex_array(
						self.shader,
						[	(vertices.vb_positions, '3f', 'v_position'),
							(vertices.vb_flags, 'u1', 'v_flags')],
						self.vb_indices,
						)
			self.va_ident = scene.ctx.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_indices,
					)
		else:
			self.va = None
			
	def __del__(self):
		if self.va:
			self.va.release()
			self.va_ident.release()
			self.vb_indices.release()
	
	def render(self, view):
		if self.va:
			self.shader['layer'] = self.layer
			self.shader['color'].write(self.color)
			self.shader['select_color'].write(self.select_color)
			self.shader['view'].write(view.uniforms['view'] * self.vertices.world)
			self.shader['proj'].write(view.uniforms['proj'])
			view.scene.ctx.point_size = self.ptsize
			self.va.render(mgl.POINTS)
	
	def identify(self, view):
		if self.va:
			scene.subident_shader['layer'] = self.layer
			scene.subident_shader['start_ident'] = view.identstep(self.vertices.nident)
			scene.subident_shader['view'].write(view.uniforms['view'] * self.vertices.world)
			scene.subident_shader['proj'].write(view.uniforms['proj'])
			view.ctx.point_size = self.ptsize
			self.va_ident.render(mgl.POINTS)

def digitfit(n):
	''' return the number of zeros in the digital representation of n '''
	s = 0
	while n:
		if not n%10:	s += 1
		n //=10
	return s

class GridDisplay(Display):
	''' display a grid clipped to the view, helping to appreciate distances
		The grid display distances in the plane of the `center` point
		
		:unit:      factor on the distance between dots
		:color:     base color of the grid, with an alpha channel
		:contrast:  contrast between the smallest division and the biggest
	'''
	def __init__(self, scene, center, unit=1, color=None, contrast=1.8):
		def load(scene):
			n = 101
			h = n//2
			pts = np.empty((n,n), dtype='f4, f4, f2')
			for i in range(n):
				for j in range(n):
					pts[i,j] = (i-h, j-h, (1+min(digitfit(i), digitfit(j))))
			
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/viewgrid.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/viewgrid.frag').read(),
						)
			vb = scene.ctx.buffer(pts)
			va = scene.ctx.vertex_array(shader, [(vb, '2f4 f2', 'v_position', 'v_opacity')])
			return shader, va
		self.shader, self.va = scene.ressource('viewgrid', load)
		self.unit = unit
		self.center = fvec3(center or 0)
		self.color = fvec4(color) if color else fvec4(settings.display['point_color'],1)
		self.contrast = contrast
	
	def render(self, view):
		center = fvec3( view.uniforms['view'] * self.world * fvec4(self.center,1) )
		if not center.z < 0:	return
		zlog = log(-center.z)/log(10)
		sizelog = floor(zlog)
		
		view.scene.ctx.point_size = 1/400 * view.fb_screen.height
		self.shader['color'].write(fvec4(
				self.color.rgb, 
				self.color.a * exp(self.contrast*(-1+sizelog-zlog)),
				))
		self.shader['size'] = self.unit * 10**(sizelog-1)
		self.shader['centerdist'] = -center.z
		self.shader['proj'].write(view.uniforms['proj'])
		self.shader['contrast'] = self.contrast
		self.va.render(mgl.POINTS)
	
	def stack(self, scene):
		return ((), 'screen', 3, self.render),
		

class SplineDisplay(Display):
	''' display for spline curve, with handles around'''
	def __init__(self, scene, handles, curve, color=None):
		self.color = color or fvec4(settings.display['line_color'], 1)
		self.color_handles = fvec4(settings.display['annotation_color'], 0.6)
		self.box = npboundingbox(handles)
		ctx = scene.ctx
		self.vb_handles = ctx.buffer(handles)
		self.vb_curve = ctx.buffer(curve)
		
		self.shader = scene.ressource('shader_uniformcolor', shader_uniformcolor)
		self.va_handles = ctx.vertex_array(self.shader, [(self.vb_handles, '3f4', 'v_position')])
		self.va_curve = ctx.vertex_array(self.shader, [(self.vb_curve, '3f4', 'v_position')])
		
		self.shader_ident = scene.ressource('shader_ident')
		self.va_ident = ctx.vertex_array(self.shader_ident, [(self.vb_curve, '3f4', 'v_position')])
		
	def __del__(self):
		self.va_handles.release()
		self.va_curve.release()
		self.va_ident.release()
		self.vb_handles.release()
		self.vb_curve.release()
		
	def render(self, view):
		self.shader['view'].write(view.uniforms['view'] * self.world)
		self.shader['proj'].write(view.uniforms['proj'])
		view.scene.ctx.point_size = 4
		
		self.shader['layer'] = -2e-6
		self.shader['color'].write(self.color_handles if not self.selected else fvec4(settings.display['select_color_line'],self.color_handles[3]))
		self.va_handles.render(mgl.POINTS)
		self.va_handles.render(mgl.LINE_STRIP)
		
		self.shader['layer'] = -1e-6
		self.shader['color'].write(self.color if not self.selected else fvec4(settings.display['select_color_line'],self.color[3]))
		self.va_curve.render(mgl.LINE_STRIP)
		
	def identify(self, view):
		self.shader_ident['ident'] = view.identstep(1)
		self.shader_ident['view'].write(view.uniforms['view'] * self.world)
		self.shader_ident['proj'].write(view.uniforms['proj'])
		self.va_ident.render(mgl.LINE_STRIP)
		
	def stack(self, scene):
		return (	((), 'screen', 1, self.render),
					((), 'ident', 1, self.identify)	)
					
					
					
class VoxelDisplay(Display):
	''' display a voxel as a scalar field.
		its opacity is computed using its density as absorbance.
		
		Parameters:
			
			voxel:	a numpy array with dimension 3, no specific value range is required
			
		Attributes:
		
			space:	transformation matrix from voxel normalized coordinates (position in the array as range 0-1) to local coordinates
			world:	transformation matrix from local to world coordinates
			value_range:	
				
				tuple `(min, max)`  giving the min and max values to display
				- values below min will be transparent
				- values above max will be fully opaq
				
			color_range:	tuple `(fvec4, fvec4)` of colors matching the value range bounds
		
		value examples:
		
			* value_range
			
				```
				(0.4, 0.9)  # erase the lower value and saturate the maxium ones
				```
				
			* color_range
			
				```
				(fvec4(0,0,1,1), fvec4(0,1,0,1))	 # blue-green
				(fvec4(0,0.5,1,1), fvec4(1,1,0,1))   # lightblue-yellow
				(fvec4(1,0,0.3,1), fvec4(1,1,0,1))   # red-yellow
				(fvec4(0,0,1,1), fvec4(1,0.5,0.1,1)) # blue-orange
				(fvec4(0,0,1,1), fvec4(1,0.1,0.1,1)) # blue-red
				(fvec4(0,0,0,1), fvec4(1,1,1,1))     # black-white
				(fvec4(1,1,1,1), fvec4(1,1,1,1))     # white-white
				```
	'''
	def __init__(self, scene, voxel: 'ndarray', space: 'fmat4', 
				value=(0, 1), 
				color=(fvec4(0,0,1,1), fvec4(0,1,0,1))):
		
		self.voxel = scene.ctx.texture3d(
			voxel.shape, 
			1, 
			voxel.transpose(2,1,0).copy().astype('f2'),  # axis permutation for opengl
			dtype='f2',
			)
		self.voxel.repeat_x = False
		self.voxel.repeat_y = False
		self.voxel.repeat_z = False
		self.space = space
		self.value_range = value
		self.color_range = color

		def load(scene):
			from . import generation
			from .mesh import glmarray
			
			# load shader
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/voxel.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/voxel.frag').read(),
						)
			# load vertex buffer for the brick
			brick = generation.brick(min=vec3(0), max=vec3(1)) .flip()
			pts = []
			for face in brick.faces:
				pts.extend(brick.facepoints(face))
			vb = scene.ctx.buffer(glmarray(pts))
			
			return shader, vb
		
		self.shader, self.vb = scene.ressource('shader_voxel', load)
		self.va = scene.ctx.vertex_array(
				self.shader,
				[(self.vb, '3f', 'v_position')],
				)
				
	def __del__(self):
		self.va.release()
		self.voxel.release()

	def render(self, view):
		view.scene.ctx.enable(mgl.DEPTH_TEST | mgl.CULL_FACE)
		self.shader['value_min'] = self.value_range[0]
		self.shader['value_max'] = self.value_range[1]
		self.shader['color_min'].write(self.color_range[0])
		self.shader['color_max'].write(self.color_range[1])
		self.shader['view'].write(view.uniforms['view'] * self.world * self.space)
		self.shader['proj'].write(view.uniforms['proj'])
		self.voxel.use(0)
		self.va.render(mgl.vertex_array.TRIANGLES)
		
	def stack(self, scene):
		return ((), 'screen', 10, self.render),




def tupledisplay(scene, t):
	if primitives.isaxis(t):	return AxisDisplay(scene, t)
	# if not found: empty display
	return Display()

overrides.update({
	vec3:   PointDisplay,
	tuple:  tupledisplay,
	Box:    BoxDisplay,
	})

	
