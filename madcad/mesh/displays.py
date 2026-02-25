# This file is part of pymadcad,  distributed under license LGPL v3
import numpy as np
import moderngl as mgl
from PIL import Image
from pyglm.glm import fvec3, fmat4, normalize, dot, length, fvec4

from .. import settings
from ..common import resourcedir
from ..rendering import Display
from ..rendering.d3 import npboundingbox, load_shader_wire, load_shader_subident

__all__ = ['MeshDisplay', 'WebDisplay']


class MeshDisplay(Display):
	''' Display render Meshes '''
	def __init__(self, scene, positions, normals, faces, lines, idents, color=None):
		self.box = npboundingbox(positions, ignore=True)
		
		color = fvec3(color or settings.colors['surface'])
		line = ( (length(settings.colors['line']) + dot(
					color - settings.colors['surface'], 
					settings.colors['line'] - settings.colors['surface']))
				* normalize(color + 1e-6)  )
		#if length(s['line_color']) > length(color)
		reflect = normalize(color + 1e-6) * settings.display['solid_reflectivity']
		
		self._vertices = Vertices(scene.context, positions, idents)
		self._disp_faces = FacesDisplay(scene, self._vertices, normals, faces, color=color, reflect=reflect, layer=0)
		self._disp_ghost = GhostDisplay(scene, self._vertices, normals, faces, color=line, layer=0)
		self._disp_groups = LinesDisplay(scene, self._vertices, lines, color=line, alpha=1, layer=-2e-6)
		self._disp_points = PointsDisplay(scene, self._vertices, range(len(positions)), color=line, layer=-3e-6)
		wire = []
		for f in faces:
			wire.append((f[0], f[1]))
			wire.append((f[1], f[2]))
			wire.append((f[2], f[0]))
		self._disp_wire = LinesDisplay(scene, self._vertices, wire, color=line, alpha=0.3, layer=-1e-6)
		
	def stack(self, scene):
		yield (self, 'screen', -1, self._vertices.prerender)
		if scene.options['display_faces']:	
			yield (self, 'screen', 0, self._disp_faces.render)
			yield (self, 'ident', 0, self._disp_faces.identify)
		else:								
			yield (self, 'screen', 1, self._disp_ghost.render)
			yield (self, 'ident', 0, self._disp_ghost.identify)
		if scene.options['display_groups']:	yield (self, 'screen', 1, self._disp_groups.render)
		if scene.options['display_points']:	yield (self, 'screen', 2, self._disp_points.render)
		if scene.options['display_wire']:	yield (self, 'screen', 2, self._disp_wire.render)
	
	def _get_world(self):	
		return self._vertices.world
	def _set_world(self, value):
		self._vertices.world = value
	world:fmat4 = property(_get_world, _set_world)
	
	def _get_selected(self):
		return self._vertices.selected
	def _set_selected(self, value):
		if not isinstance(value, set):
			if value:
				self._vertices.selected.add(None)
			else:
				self._vertices.selected.clear()
			value = self._vertices.selected
		self._vertices.selected = value
		self._vertices.flags_updated = True
	selected:set = property(_get_selected, _set_selected)
	
	def _get_hovered(self):
		return self._vertices.hovered
	def _set_hovered(self, value):
		if not isinstance(value, set):
			if value:
				self._vertices.hovered.add(None)
			else:
				self._vertices.hovered.clear()
			value = self._vertices.hovered
		self._vertices.hovered = value
		self._vertices.flags_updated = True
	hovered:set = property(_get_hovered, _set_hovered)
	

class WebDisplay(Display):
	''' Display to render Webs '''
	def __init__(self, scene, positions, lines, points, idents, color=None):
		self.box = npboundingbox(positions, ignore=True)
		color = color or settings.colors['line']
		self._vertices = Vertices(scene.context, positions, idents)
		self._disp_edges = LinesDisplay(scene, self._vertices, lines, color=color, alpha=1, layer=-2e-6)
		self._disp_groups = PointsDisplay(scene, self._vertices, points, layer=-3e-6)
		self._disp_points = PointsDisplay(scene, self._vertices, range(len(positions)), layer=-1e-6)

	def stack(self, scene):
		yield (self, 'screen', -1, self._vertices.prerender)
		if scene.options['display_groups']:		yield (self, 'screen', 2, self._disp_groups.render)
		if scene.options['display_points']:		yield (self, 'screen', 2, self._disp_points.render)
		yield (self, 'screen', 1, self._disp_edges.render)
		yield (self, 'ident', 1, self._disp_edges.identify)
	
	def _get_world(self):	
		return self._vertices.world
	def _set_world(self, value):	
		self._vertices.world = value
	world = property(_get_world, _set_world)
	
	def _get_selected(self):
		return self._vertices.selected
	def _set_selected(self, value):
		if not isinstance(value, set):
			if value:
				self._vertices.selected.add(None)
			else:
				self._vertices.selected.clear()
			value = self._vertices.selected
		self._vertices.selected = value
		self._vertices.flags_updated = True
	selected:set = property(_get_selected, _set_selected)
	
	def _get_hovered(self):
		return self._vertices.hovered
	def _set_hovered(self, value):
		if not isinstance(value, set):
			if value:
				self._vertices.hovered.add(None)
			else:
				self._vertices.hovered.clear()
			value = self._vertices.hovered
		self._vertices.hovered = value
		self._vertices.flags_updated = True
	hovered:set = property(_get_hovered, _set_hovered)


class Vertices:
	''' convenient class to share vertices between SolidDisplay, WebDisplay, PointsDisplay '''
	# vertex flags for shaders
	HOVERED = 1<<0
	SELECTED = 1<<1
	
	def __init__(self, ctx, positions, idents):
		self.idents = idents
		self.nident = int(max(idents))+1
		self.u_flags = 0
		self.v_flags = np.zeros(len(positions), dtype='u1')
		self.flags_updated = False
		self.selected = set()
		self.hovered = set()
		
		assert len(idents) == len(positions)
		self.vb_positions = ctx.buffer(np.asarray(positions, dtype='f4', order='C'))
		self.vb_idents = ctx.buffer(np.asarray(idents, dtype='u2', order='C'))
		self.vb_flags = ctx.buffer(self.v_flags, dynamic=True)
		self.world = fmat4(1)
		
	def prerender(self, view):
		if self.flags_updated:
			self.flags_updated = False
			self.u_flags = (
				(None in self.hovered)*self.HOVERED
				| (None in self.selected)*self.SELECTED
				)
			if (None not in self.selected) or (None not in self.hovered):
				for i,ident in enumerate(self.idents):
					self.v_flags[i] = (
						(ident in self.hovered)*self.HOVERED
						| (ident in self.selected)*self.SELECTED
						)
				self.vb_flags.write(self.v_flags)


class FacesDisplay:
	def __init__(self, scene, vertices, normals, faces, color, reflect, layer=0):
		self.color = color
		self.layer = layer
		self.reflect = reflect
		self._vertices = vertices
	
		# load the skybox texture
		def load(scene):
			img = Image.open(resourcedir+'/textures/'+settings.display['solid_reflect'])
			return scene.context.texture(img.size, 3, img.tobytes())
		self.reflectmap = scene.share('skybox', load)
		
		# load the shader
		self.shader = scene.share(type(self), self._share)
		self.ident_shader = scene.share(load_shader_subident, load_shader_subident)
		# allocate buffers
		if faces is not None and len(faces) and vertices.vb_positions:
			self.vb_faces = scene.context.buffer(np.asarray(faces, 'u4', order='C'))
			self.vb_normals = scene.context.buffer(np.asarray(normals, 'f4', order='C'))
			self.va = scene.context.vertex_array(
					self.shader, 
					[	(vertices.vb_positions, '3f', 'v_position'), 
						(self.vb_normals, '3f', 'v_normal'),
						(vertices.vb_flags, 'u1', 'v_flags')],
					self.vb_faces,
					mode=mgl.TRIANGLES,
					)
			
			self.va_ident = scene.context.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_faces,
					mode=mgl.TRIANGLES,
					)
		else:
			self.va = None
	
	def _share(self, scene):
		shader = scene.context.program(
			vertex_shader=open(resourcedir+'/shaders/solid.vert').read(),
			fragment_shader=open(resourcedir+'/shaders/solid.frag').read(),
			)
		# setup some uniforms
		shader['reflectmap'] = 0
		return shader
	
	def render(self, view):
		if self.va:
			self.reflectmap.use(0)
			self.va.program['u_flags'] = self._vertices.u_flags
			self.va.program['selected_color'].write(settings.display['selection_color'])
			self.va.program['hovered_color'].write(settings.display['hover_color'])
			self.va.program['min_color'].write(self.color * settings.display['solid_color_side'])
			self.va.program['max_color'].write(self.color * settings.display['solid_color_front'])
			self.va.program['refl_color'].write(self.reflect)
			self.va.program['layer'] = self.layer
			self.va.program['world'].write(self._vertices.world)
			self.va.program['view'].write(view.uniforms['view'])
			self.va.program['proj'].write(view.uniforms['proj'])
			self.va.render()
	
	def identify(self, view):
		if self.va:
			self.va_ident.program['layer'] = self.layer
			self.va_ident.program['start_ident'] = view.identstep(self._vertices.nident)
			self.va_ident.program['view'].write(view.uniforms['view'] * self._vertices.world)
			self.va_ident.program['proj'].write(view.uniforms['proj'])
			self.va_ident.render()

class GhostDisplay:
	def __init__(self, scene, vertices, normals, faces, color, layer=0):
		self.color = color
		self.layer = layer
		self._vertices = vertices
		
		self.shader = scene.share(type(self), self._share)
		self.ident_shader = scene.share(load_shader_subident, load_shader_subident)
		# allocate buffers
		if faces is not None and len(faces) and vertices.vb_positions:
			self.vb_faces = scene.context.buffer(np.asarray(faces, 'u4', order='C'))
			self.vb_normals = scene.context.buffer(np.asarray(normals, 'f4', order='C'))
			self.va = scene.context.vertex_array(
					self.shader, 
					[	(vertices.vb_positions, '3f', 'v_position'), 
						(self.vb_normals, '3f', 'v_normal'),
						(vertices.vb_flags, 'u1', 'v_flags')],
					self.vb_faces,
					mode=mgl.TRIANGLES,
					)
			
			self.va_ident = scene.context.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_faces,
					mode=mgl.TRIANGLES,
					)
		else:
			self.va = None
	
	def _share(self, scene):
		return scene.context.program(
			vertex_shader=open(resourcedir+'/shaders/solid.vert').read(),
			fragment_shader=open(resourcedir+'/shaders/ghost.frag').read(),
			)
	
	def render(self, view):
		if self.va:
			# setup uniforms
			view.scene.context.enable_only(mgl.BLEND)
			self.va.program['u_flags'] = self._vertices.u_flags
			self.va.program['normal_color'].write(self.color)
			self.va.program['selected_color'].write(settings.display['selection_color'])
			self.va.program['hovered_color'].write(settings.display['hover_color'])
			self.va.program['world'].write(self._vertices.world)
			self.va.program['view'].write(view.uniforms['view'])
			self.va.program['proj'].write(view.uniforms['proj'])
			self.va.program['layer'] = self.layer
			self.va.render()
			view.scene.context.enable_only(mgl.BLEND | mgl.DEPTH_TEST)
	
	def identify(self, view):
		if self.va:
			self.ident_shader['start_ident'] = view.identstep(self._vertices.nident)
			self.ident_shader['view'].write(view.uniforms['view'] * self._vertices.world)
			self.ident_shader['proj'].write(view.uniforms['proj'])
			self.ident_shader['layer'] = self.layer
			# render on self.context
			self.va_ident.render()

class LinesDisplay:
	def __init__(self, scene, vertices, lines, color, alpha=1, layer=0):
		self.layer = layer
		self.color = fvec4(fvec3(color), alpha)
		self._vertices = vertices
		
		# load the line shader
		self.shader = scene.share(load_shader_wire, load_shader_wire)
		self.ident_shader = scene.share(load_shader_subident, load_shader_subident)
		if lines is not None and len(lines) and vertices.vb_positions:
			# allocate buffers
			self.vb_lines = scene.context.buffer(np.asarray(lines, dtype='u4', order='C'))
			self.va = scene.context.vertex_array(
						self.shader,
						[	(vertices.vb_positions, '3f', 'v_position'),
							(vertices.vb_flags, 'u1', 'v_flags')],
						self.vb_lines,
						mode = mgl.LINES,
						)
			self.va_ident = scene.context.vertex_array(
					self.ident_shader, 
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_idents, 'u2', 'item_ident')], 
					self.vb_lines,
					mode = mgl.LINES,
					)
		else:
			self.va = None
	
	def render(self, view):
		if self.va:
			self.va.program['u_flags'] = self._vertices.u_flags
			self.va.program['color'].write(self.color)
			self.va.program['selected_color'].write(settings.display['selection_color'])
			self.va.program['hovered_color'].write(settings.display['hover_color'])
			self.va.program['view'].write(view.uniforms['view'] * self._vertices.world)
			self.va.program['proj'].write(view.uniforms['proj'])
			self.va.program['layer'] = self.layer
			self.va.render()
		
	def identify(self, view):
		if self.va:
			self.va_ident.program['start_ident'] = view.identstep(self._vertices.nident)
			self.va_ident.program['view'].write(view.uniforms['view'] * self._vertices.world)
			self.va_ident.program['proj'].write(view.uniforms['proj'])
			self.va_ident.program['layer'] = self.layer
			self.va_ident.render()
		
class PointsDisplay:
	def __init__(self, scene, vertices, indices=None, color=None, ptsize=3, layer=0):
		self.color = fvec4(color or settings.colors['point'], 1)
		self.select_color = fvec4(settings.display['selection_color'].rgb, 1)
		self.ptsize = ptsize
		self.layer = layer
		self._vertices = vertices
		
		# load the line shader
		self.shader = scene.share(load_shader_wire, load_shader_wire)
		self.ident_shader = scene.share(load_shader_subident, load_shader_subident)
		# allocate GPU objects
		if indices is not None and len(indices) and vertices.vb_positions:
			self.vb_indices = scene.context.buffer(np.asarray(indices, dtype='u4', order='C'))
			self.va = scene.context.vertex_array(
				self.shader,
				[	(vertices.vb_positions, '3f', 'v_position'),
					(vertices.vb_flags, 'u1', 'v_flags')],
				self.vb_indices,
				mode=mgl.POINTS,
				)
			self.va_ident = scene.context.vertex_array(
				self.ident_shader, 
				[	(vertices.vb_positions, '3f', 'v_position'),
					(vertices.vb_idents, 'u2', 'item_ident')], 
				self.vb_indices,
				mode=mgl.POINTS,
				)
		else:
			self.va = None
	
	def render(self, view):
		if self.va:
			view.scene.context.point_size = self.ptsize
			self.va.program['u_flags'] = self._vertices.u_flags
			self.va.program['layer'] = self.layer
			self.va.program['color'].write(self.color)
			self.va.program['selected_color'].write(settings.display['selection_color'])
			self.va.program['hovered_color'].write(settings.display['hover_color'])
			self.va.program['view'].write(view.uniforms['view'] * self._vertices.world)
			self.va.program['proj'].write(view.uniforms['proj'])
			self.va.render()
	
	def identify(self, view):
		if self.va:
			view.ctx.point_size = self.ptsize
			self.va_ident.program['layer'] = self.layer
			self.va_ident.program['start_ident'] = view.identstep(self._vertices.nident)
			self.va_ident.program['view'].write(view.uniforms['view'] * self._vertices.world)
			self.va_ident.program['proj'].write(view.uniforms['proj'])
			self.va_ident.render()
