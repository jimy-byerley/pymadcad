# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import vec3, fvec3, fvec4, fmat4, mix, normalize, length, distance, dot, noproject, dirbase, Box, isnan, isinf
from .mathutils import transform as mktransform
from . import settings
from . import view
from .common import ressourcedir
from PIL import Image
import numpy.core as np
import moderngl as mgl

class Displayable:
	''' simple object for declaration of a display that will be constructed when the scene will be ready.
		allow to plan displays before the scene declaration or window initialization
	'''
	def __init__(self, disp, *args, **kwargs):
		self.display = lambda scene: (disp(scene, *args, **kwargs),)

class PointDisplay:
	renderindex = 3
	def __init__(self, scene, position, size=10, color=None, selected=False, transform=fmat4(1)):
		self.position = fvec3(position)
		self.size = size
		self.selected = selected
		self.color = fvec3(color or settings.display['line_color'])
		self.transform = fmat4(transform)
		
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
			va_idents = scene.ctx.vertex_array(ident_shader, [(vb, '2f', 'v_uv')])
			return texture, shader, va, ident_shader, va_idents
		
		(	self.texture, 
			self.shader, 
			self.va, 
			self.ident_shader, 
			self.va_idents	) = scene.ressource('pointhalo', load)
	
	def render(self, scene):
		self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
		self.shader['position'].write(fvec3(self.transform * fvec4(self.position,1)))
		self.shader['view'].write(scene.view_matrix)
		self.shader['proj'].write(scene.proj_matrix)
		self.shader['ratio'] = (
				self.size / scene.width(),
				self.size / scene.height(),
				)
		self.texture.use(0)
		self.va.render(mgl.TRIANGLES)

	def identify(self, scene, startident):
		self.ident_shader['ident'] = startident
		self.ident_shader['position'].write(fvec3(self.transform * fvec4(self.position,1)))
		self.ident_shader['view'].write(scene.view_matrix)
		self.ident_shader['proj'].write(scene.proj_matrix)
		self.ident_shader['ratio'] = (
				self.size / scene.width(),
				self.size / scene.height(),
				)
		self.va_idents.render(mgl.TRIANGLES)
		return 1
	
	#def control(self, scene, rdr, ident, evt):
		#self.selected = not self.selected
	
	def select(self, idents, state=None):
		if state is not None:	self.selected = state
		else:					return self.selected

class ArrowDisplay:
	renderindex = 2
	def __init__(self, scene, axis, color=None, transform=fmat4(1)):
		x,y,z = dirbase(axis[1])
		self.local = fmat4(mktransform(axis[0],x,y,z))
		self.color = fvec3(color or settings.display['annotation_color'])
		self.transform = fmat4(transform)
		self.selected = False
		
		# load shader
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/uniformcolor.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/uniformcolor.frag').read(),
						)
		self.shader = scene.ressource('shader_uniformcolor', load)
		self.va, self.va_idents = scene.ressource('arrow', self.load)
	
	def load(self, scene):
		pts = [	(0,0,0), (0,0,1), 
				(0,0,1), ( 0.16,0,1-0.2), 
				(0,0,1), (-0.16,0,1-0.2),
				]
		vb = scene.ctx.buffer(np.array(pts, 'f4'))
		va = scene.ctx.vertex_array(self.shader, [(vb, '3f', 'v_position')])
		va_idents = scene.ctx.vertex_array(scene.ident_shader, [(vb, '3f', 'v_position')])
		return va, va_idents
	
	def render(self, scene):
		self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
		self.shader['proj'].write(scene.proj_matrix)
		self.shader['view'].write(scene.view_matrix * self.transform * self.local)
		self.va.render(mgl.LINES)
	
	def identify(self, scene, startident):
		scene.ident_shader['proj'].write(scene.proj_matrix)
		scene.ident_shader['view'].write(scene.view_matrix * self.transform * self.local)
		scene.ident_shader['ident'] = startident
		self.va_idents.render(mgl.LINES)
		return 1
		
	#def control(self, scene, rdr, ident, evt):
		#self.selected = not self.selected
	
	def select(self, idents, state=None):
		if state is not None:	self.selected = state
		else:					return self.selected

class TangentDisplay:
	renderindex = 2
	def __init__(self, scene, axis, size=None, color=None, transform=fmat4(1)):
		self.size = size
		self.axis = axis
		self.arrow1 = ArrowDisplay(scene, axis, color, transform)
		self.arrow2 = ArrowDisplay(scene, axis, color, transform)
		self.selected = False
	
	def render(self, scene):
		size = self.size or -scene.view_matrix[3][2] * 10/scene.height()
		x,y,z = dirbase(self.axis[1])
		self.arrow1.local = fmat4(mktransform(self.axis[0], size*x, size*y, size*z))
		self.arrow2.local = fmat4(mktransform(self.axis[0], size*x, size*y, -size*z))
		self.arrow1.render(scene)
		self.arrow2.render(scene)
	
	def identify(self, scene, startident):
		self.arrow1.identify(scene, startident)
		self.arrow2.identify(scene, startident)
		return 1
	
	#def control(self, scene, rdr, ident, evt):
		#self.select(0, not self.select(0))
	
	def select(self, idents, state=None):
		r = self.arrow1.select(idents, state)
		self.arrow2.select(idents, state)
		return r

class AxisDisplay:
	renderindex = 2
	pattern = [0, 0.25, 0.45, 0.55, 0.75, 1]
	repetitions = 3
	def __init__(self, scene, axis, interval=None, color=None, transform=fmat4(1)):
		self.origin = fvec3(axis[0])
		self.direction = fvec3(axis[1])
		self.interval = interval
		self.color = fvec3(color or settings.display['line_color'])
		self.transform = fmat4(transform)
		self.selected = False
		
		self.shader, self.va, self.ident_shader, self.va_idents = scene.ressource('axis', self.load)
	
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
		va_idents = scene.ctx.vertex_array(ident_shader, [(vb, 'f 12x', 'v_absciss')])
		return shader, va, ident_shader, va_idents
	
	def _disp_interval(self, scene):
		if self.interval:	return self.interval
		else:
			size = -scene.view_matrix[3][2]/6
			return (-0.5*size, size)
	
	def render(self, scene):
		self.shader['projview'].write(scene.proj_matrix * scene.view_matrix)
		self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
		self.shader['origin'].write(self.origin)
		self.shader['direction'].write(self.direction)
		self.shader['transform'].write(self.transform)
		self.shader['interval'] = self._disp_interval(scene)
		self.va.render(mgl.LINES)
	
	def identify(self, scene, startident):
		self.ident_shader['projview'].write(scene.proj_matrix * scene.view_matrix)
		self.ident_shader['origin'].write(self.origin)
		self.ident_shader['direction'].write(self.direction)
		self.ident_shader['transform'].write(self.transform)
		self.ident_shader['interval'] = self._disp_interval(scene)
		self.ident_shader['ident'] = startident
		self.va_idents.render(mgl.LINES)
		return 1
	
	# axis = (vec3(0), vec3(1))
		
	#def control(self, scene, rdr, ident, evt):
		#self.selected = not self.selected
	
	def select(self, idents, state=None):
		if state is not None:	self.selected = state
		else:					return self.selected


class AnnotationDisplay:
	renderindex = 3
	def __init__(self, scene, points, color, transform):
		self.color = fvec3(color or settings.display['annotation_color'])
		self.transform = fmat4(transform)
		self.selected = False
		# load shader
		def load(scene):
			return scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/annotation.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/annotation.frag').read(),
						)
		self.shader = scene.ressource('shader_annotation', load)
		# allocate buffers
		vb_pts = scene.ctx.buffer(points)
		self.va = scene.ctx.vertex_array(self.shader, [(vb_pts, '3f f', 'v_position', 'v_alpha')])
		self.va_idents = scene.ctx.vertex_array(scene.ident_shader, [(vb_pts, '3f 4x', 'v_position')])
	
	@staticmethod
	def buff_ptsalpha(points, alpha):
		return np.hstack((
				np.array([tuple(p) for p in points], 'f4'), 
				np.array(alpha, 'f4', ndmin=2).transpose(),
				))

	def render(self, scene):
		self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
		self.shader['proj'].write(scene.proj_matrix)
		self.shader['view'].write(scene.view_matrix * self.transform)
		self.va.render(mgl.LINES)
	
	def identify(self, scene, startident):
		scene.ident_shader['proj'].write(scene.proj_matrix)
		scene.ident_shader['view'].write(scene.view_matrix * self.transform)
		scene.ident_shader['ident'] = startident
		self.va_idents.render(mgl.LINES)
		return 1
		
	#def control(self, scene, rdr, ident, evt):
		#self.selected = not self.selected
	
	def select(self, idents, state=None):
		if state is not None:	self.selected = state
		else:					return self.selected

class RadiusMeasure(AnnotationDisplay):
	def __init__(self, scene, circle, location=None, color=None, transform=fmat4(1)):
		center = circle.axis[0]
		x,y,z = dirbase(circle.axis[1], normalize(location-center) if location else circle.alignment)
		d = abs(dot(location-center, x))/circle.radius if location else 2
		r = circle.radius*x
		sizeref = d*circle.radius
		arrowx = 0.16*sizeref*x
		arrowy = 0.06*sizeref*y
		
		pts = [
			r, r+arrowx-arrowy,
			r, r+arrowx+arrowy,
			r, d*r,
			]
		for i in range(len(pts)):	pts[i] = center+pts[i]
		alpha = [1] * 6
		self.textplace = center + d*r
		
		super().__init__(scene, self.buff_ptsalpha(pts, alpha), color, transform)
		

class LengthMeasure(AnnotationDisplay):
	def __init__(self, scene, a, b, location=None, color=None, transform=fmat4(1)):
		# place points
		a = fvec3(a)
		b = fvec3(b)
		if location:	location = fvec3(location)
		middle = (a + b)/2
		if not location:	location = middle
		dir = normalize(a-b)
		top = noproject(location-middle, dir)
		topdist = length(top)
		sizeref = max(distance(a,b), topdist)
		arrowx = sizeref*0.08*dir
		arrowy = sizeref*0.03*(top/topdist if topdist/distance(a,b) > 1e-6 else fvec3(dirbase(vec3(dir))[0]))
		pts = [	a, a+top+arrowy*0.75, 
				a+top, a+top-arrowx-arrowy, 
				a+top, a+top-arrowx+arrowy,
				b, b+top+arrowy*0.75, 
				b+top, b+top+arrowx-arrowy, 
				b+top, b+top+arrowx+arrowy,
				a+top, b+top,
			]
		o = 0.4
		alpha = [o, o, 1, 1, 1, 1, o, o, 1, 1, 1, 1, 1, 1]
		self.textplace = middle + top

		super().__init__(scene, self.buff_ptsalpha(pts, alpha), color, transform)

class ArcMeasure(AnnotationDisplay):
	def __init__(self, scene, arc, location=None, color=None, arrows=None, transform=fmat4(1)):
		# place points
		middle = (a + b)/2
		if not location:	location = middle
		dir = normalize(a-b)
		top = noproject(location-middle, dir)
		topdist = length(top)
		sizeref = max(distance(a,b), topdist)
		arrowx = sizeref*0.08*dir
		arrowy = sizeref*0.03*(top/topdist if topdist/distance(a,b) > 1e-6 else fvec3(dirbase(vec3(dir))[0]))
		pts = [	a, a+top+arrowy*0.75, 
				a+top, a+top-arrowx-arrowy, 
				a+top, a+top-arrowx+arrowy,
				b, b+top+arrowy*0.75, 
				b+top, b+top+arrowx-arrowy, 
				b+top, b+top+arrowx+arrowy,
				a+top, b+top,
			]
		o = 0.4
		alpha = [o, o, 1, 1, 1, 1, o, o, 1, 1, 1, 1, 1, 1]
		self.textplace = middle + top
		
		super().__init__(scene, self.buff_ptsalpha(pts, alpha), color, transform)
		
class BoxDisplay(AnnotationDisplay):
	def __init__(self, scene, box, color=None, transform=fmat4(1)):
		# place points
		w = box.width
		x,y,z = w
		c = 0.1*min(x,y,z)	# corner
		o = 0.3 # wire alpha
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

		super().__init__(scene, pts, color, transform)
	

class SolidDisplay:
	''' Display render Meshes '''
	renderindex = 1
	def __init__(self, scene, positions, normals, faces, lines, idents, color=None, transform=fmat4(1)):
		self.options = scene.options		
		color = fvec3(color or settings.display['solid_color'])
		line_color = fvec3(settings.display['line_color'])
		self.vertices = Vertices(scene.ctx, positions, idents, fmat4(transform))
		self.disp_faces = FacesDisplay(scene, self.vertices, normals, faces, color) if len(faces) else None
		self.disp_groups = LinesDisplay(scene, self.vertices, lines, line_color) if len(lines) else None
		self.disp_points = PointsDisplay(scene, self.vertices, color=line_color) if len(positions) else None
		wire = []
		for f in faces:
			wire.append((f[0], f[1]))
			wire.append((f[1], f[2]))
			wire.append((f[2], f[0]))
		self.disp_wire = LinesDisplay(scene, self.vertices, wire, mix(color, line_color, 0.3)) if len(wire) else None
	
	def render(self, scene):
		self.vertices.update(scene)
		if self.options['display_faces'] and self.disp_faces:	self.disp_faces.render(scene)
		if self.options['display_groups'] and self.disp_groups:	self.disp_groups.render(scene)
		if self.options['display_points'] and self.disp_points:	self.disp_points.render(scene)
		if self.options['display_wire'] and self.disp_wire:	self.disp_wire.render(scene)
	
	def identify(self, scene, startident):
		# NOTE only the face identifications are used, whatever is the display option, the va_ident from the other objects are useless here
		self.disp_faces.identify(scene, startident)
		return self.vertices.nident
	
	#def control(self, *args):	return self.vertices.control(*args)
	def select(self, *args):	return self.vertices.select(*args)
	
	@property
	def transform(self):
		return self.vertices.transform
	@transform.setter
	def transform(self, value):
		self.vertices.transform = value
		

class WebDisplay:
	''' Display to render Webs '''
	renderindex = 2
	def __init__(self, scene, positions, lines, points, idents, color=None, transform=fmat4(1)):
		self.options = scene.options
		
		color =  fvec3(color or settings.display['line_color'])
		self.vertices = Vertices(scene.ctx, positions, idents, fmat4(transform))
		self.disp_edges = LinesDisplay(scene, self.vertices, lines, color)
		self.disp_groups = PointsDisplay(scene, self.vertices, points, color=color)
		self.disp_points = PointsDisplay(scene, self.vertices, color=color)
		
	def render(self, scene):
		self.vertices.update(scene)
		self.disp_edges.render(scene)
		if self.options['display_groups']:		self.disp_groups.render(scene)
		if self.options['display_points']:		self.disp_points.render(scene)
	
	def identify(self, scene, startident):
		self.disp_edges.identify(scene, startident)
		return self.vertices.nident
	
	#def control(self, *args):	self.vertices.control(*args)
	def select(self, *args):	self.vertices.select(*args)
	
	@property
	def transform(self):
		return self.vertices.transform
	@transform.setter
	def transform(self, value):
		self.vertices.transform = value
		

class Vertices(object):
	''' convenient class to share vertices between SolidDisplay, WebDisplay, PointsDisplay '''
	__slots__ = 'transform', 'idents', 'nident', 'flags', 'flags_updated', 'vb_positions', 'vb_idents', 'vb_flags'
	def __init__(self, ctx, positions, idents, transform):
		self.transform = fmat4(transform)
		self.idents = idents
		self.nident = int(max(idents))+1
		self.flags = np.zeros(len(positions), dtype='u1')
		self.flags_updated = False
		self.vb_positions = ctx.buffer(np.array(positions, dtype='f4', copy=False))
		self.vb_idents = ctx.buffer(np.array(idents, dtype=view.IDENT_TYPE, copy=False))
		self.vb_flags = self.vb_flags = ctx.buffer(self.flags, dynamic=True)
	
	#def control(self, scene, rdr, sub, evt):
		#self.select(sub, not self.select(sub))
	
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
			


class FacesDisplay:
	renderindex = 1
	def __init__(self, scene, vertices, normals, faces, color):
		self.color = color
		self.vertices = vertices
	
		# load the skybox texture
		def load(scene):
			img = Image.open(ressourcedir+'/textures/skybox-violet-saturated.png')
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
			shader['select_color'] = settings.display['select_color_face']
			return shader
		self.shader = scene.ressource('shader_solid', load)
		# allocate buffers
		vb_faces = scene.ctx.buffer(np.array(faces, 'u4', copy=False))
		vb_normals = scene.ctx.buffer(np.array(normals, 'f4', copy=False))
		self.va = scene.ctx.vertex_array(
				self.shader, 
				[	(vertices.vb_positions, '3f', 'v_position'), 
					(vb_normals, '3f', 'v_normal'),
					(vertices.vb_flags, 'u1', 'v_flags')],
				vb_faces,
				)
		
		self.va_ident = scene.ctx.vertex_array(
				scene.subident_shader, 
				[	(vertices.vb_positions, '3f', 'v_position'),
					(vertices.vb_idents, view.IDENT_TYPE, 'item_ident')], 
				vb_faces,
				)
	
	def render(self, scene):
		# setup uniforms
		self.shader['min_color'].write(self.color * settings.display['solid_color_side'])
		self.shader['max_color'].write(self.color * settings.display['solid_color_front'])
		self.shader['refl_color'].write(self.color)
		self.shader['pose'].write(self.vertices.transform)
		self.shader['view'].write(scene.view_matrix)
		self.shader['proj'].write(scene.proj_matrix)
		# render on self.context
		self.reflectmap.use(0)
		self.va.render(mgl.TRIANGLES)
	
	def identify(self, scene, startident):
		scene.subident_shader['start_ident'] = startident
		scene.subident_shader['view'].write(scene.view_matrix * self.vertices.transform)
		scene.subident_shader['proj'].write(scene.proj_matrix)
		self.va_ident.render(mgl.TRIANGLES)
		return self.vertices.nident

class LinesDisplay:
	renderindex = 2
	def __init__(self, scene, vertices, lines, color):
		self.color = color
		self.vertices = vertices
		
		# load the line shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/wire.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/wire.frag').read(),
						)
			shader['select_color'] = settings.display['select_color_line']
			return shader
		self.shader = scene.ressource('shader_wire', load)
		# allocate buffers
		vb_lines = scene.ctx.buffer(np.array(lines, dtype='u4', copy=False))
		self.va = scene.ctx.vertex_array(
					self.shader,
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_flags, 'u1', 'v_flags')],
					vb_lines,
					)
		self.va_ident = scene.ctx.vertex_array(
				scene.subident_shader, 
				[	(vertices.vb_positions, '3f', 'v_position'),
					(vertices.vb_idents, view.IDENT_TYPE, 'item_ident')], 
				vb_lines,
				)
	
	def render(self, scene):
		self.shader['color'].write(self.color)
		self.shader['view'].write(scene.view_matrix * self.vertices.transform)
		self.shader['proj'].write(scene.proj_matrix)
		self.va.render(mgl.LINES)
		
	def identify(self, scene, startident):
		scene.subident_shader['start_ident'] = startident
		scene.subident_shader['view'].write(scene.view_matrix * self.vertices.transform)
		scene.subident_shader['proj'].write(scene.proj_matrix)
		self.va_ident.render(mgl.LINES)
		return self.vertices.nident
		
class PointsDisplay:
	renderindex = 3
	def __init__(self, scene, vertices, indices=None, color=None, ptsize=3):
		self.color = color
		self.ptsize = ptsize
		self.vertices = vertices
		
		# load the line shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/wire.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/wire.frag').read(),
						)
			shader['select_color'] = settings.display['select_color_line']
			return shader
		self.shader = scene.ressource('shader_wire', load)
		# allocate GPU objects
		vb_indices = scene.ctx.buffer(np.array(indices, dtype='u4', copy=False)) if indices else None
		self.va = scene.ctx.vertex_array(
					self.shader,
					[	(vertices.vb_positions, '3f', 'v_position'),
						(vertices.vb_flags, 'u1', 'v_flags')],
					vb_indices,
					)
		self.va_ident = scene.ctx.vertex_array(
				scene.subident_shader, 
				[	(vertices.vb_positions, '3f', 'v_position'),
					(vertices.vb_idents, view.IDENT_TYPE, 'item_ident')], 
				vb_indices,
				)
	def render(self, scene):
		self.shader['color'].write(self.color)
		self.shader['view'].write(scene.view_matrix * self.vertices.transform)
		self.shader['proj'].write(scene.proj_matrix)
		scene.ctx.point_size = self.ptsize
		self.va.render(mgl.POINTS)
	
	def identify(self, scene, startident):
		scene.subident_shader['start_ident'] = startident
		scene.subident_shader['view'].write(scene.view_matrix * self.vertices.transform)
		scene.subident_shader['proj'].write(scene.proj_matrix)
		scene.ctx.point_size = self.ptsize
		self.va_idents.render(mgl.POINTS)
		return self.vertices.nident




view.dispoverrides.update({
	vec3: 	lambda p,scene: [PointDisplay(scene,p)],
	tuple:	lambda a,scene: [AxisDisplay(scene,a)],
	Box:	lambda a,scene: [BoxDisplay(scene,a)],
	})
