# This file is part of pymadcad,  distributed under license LGPL v3
import numpy.core as np
import moderngl as mgl
from PIL import Image

from ... import settings, primitives
from ...mathutils import *
from ...mesh import typedlist_to_numpy
from ...common import resourcedir
from ...qt import Qt, QEvent
from .. import Display, Scene, writeproperty, highlight_color
from . import npboundingbox, load_shader_ident, load_shader_subident, load_shader_wire, load_shader_uniformcolor

__all__ = ['PointDisplay', 'AxisDisplay', 'BoxDisplay', 'SplineDisplay', 'GridDisplay']


class PointDisplay(Display):
	position: fvec3
	size: float
	color: fvec3
	selected: bool
	hovered: bool
			
	@property
	def box(self):
		return Box(center=self.position, width=fvec3(0))
	
	def __init__(self, scene, position, size=10, color=None):
		self.position = fvec3(position)
		self.size = size
		self.color = fvec3(color or settings.colors['line'])
		self.selected = False
		self.hovered = False

		vars(self).update(scene.share(type(self), self._shared))
	
	def _shared(self, scene):
		img = Image.open(resourcedir+'/textures/point.png')
		texture = scene.context.texture(img.size, 1, img.convert('L').tobytes())
		shader = scene.context.program(
								vertex_shader=open(resourcedir+'/shaders/pointhalo.vert').read(),
								fragment_shader=open(resourcedir+'/shaders/pointhalo.frag').read(),
								)
		shader['halotex'].value = 0
		ident_shader = scene.context.program(
								vertex_shader=open(resourcedir+'/shaders/pointhalo-ident.vert').read(),
								fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
								)
		vb = scene.context.buffer(np.array([(0,0), (0,1), (1,1), (0,0), (1,1), (1,0)], 'f4'))
		
		return dict(
				_texture = texture,
				_va_screen = scene.context.vertex_array(shader, [(vb, '2f', 'v_uv')], mode=mgl.TRIANGLES),
				_va_ident = scene.context.vertex_array(ident_shader, [(vb, '2f', 'v_uv')], mode=mgl.TRIANGLES),
				)
	
	def stack(self, scene):
		return ( (self, 'ident', 2, self._identify),
				(self, 'screen', 2, self._render))

	def _render(self, view):
		self._va_screen.program['color'].write(highlight_color(self, self.color))
		self._va_screen.program['position'].write(self.world * self.position)
		self._va_screen.program['view'].write(view.uniforms['view'])
		self._va_screen.program['proj'].write(view.uniforms['proj'])
		self._va_screen.program['ratio'] = (
				self.size / view.screen.width,
				self.size / view.screen.height,
				)
		self._texture.use(0)
		self._va_screen.render()

	def _identify(self, view):
		self._va_ident.program['ident'] = view.identstep(1)
		self._va_ident.program['position'].write(self.world * self.position)
		self._va_ident.program['view'].write(view.uniforms['view'])
		self._va_ident.program['proj'].write(view.uniforms['proj'])
		self._va_ident.program['ratio'] = (
				1.5 * self.size / view.screen.width,
				1.5 * self.size / view.screen.height,
				)
		self._va_ident.render()


class AxisDisplay(Display):
	origin: fvec3
	direction: fvec3
	interval: (float, float)
	color: fvec3
	selected: bool
	hovered: bool

	pattern = [0, 0.25, 0.45, 0.55, 0.75, 1]
	repetitions = 3
	
	@property
	def box(self):
		return Box(center=self.origin, width=fvec3(0))
	
	def __init__(self, scene, axis, interval=None, color=None):
		self.origin = fvec3(axis[0])
		self.direction = fvec3(axis[1])
		self.interval = interval
		self.color = fvec3(color or settings.colors['line'])
		self.selected = False
		self.hovered = False
		
		vars(self).update(scene.share(type(self), self._shared))
	
	def _shared(self, scene):
		shader = scene.context.program(
					vertex_shader=open(resourcedir+'/shaders/axis.vert').read(),
					fragment_shader=open(resourcedir+'/shaders/axis.frag').read(),
					)
		ident_shader = scene.context.program(
					vertex_shader=open(resourcedir+'/shaders/axis-ident.vert').read(),
					fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
					)
		pts = []
		for i in range(-1, self.repetitions+1):
			for pt in self.pattern:
				if i == -1:					alpha = pt
				elif i == self.repetitions:	alpha = 1-pt
				else:						alpha = 1
				pts.append(((pt+i)/self.repetitions, alpha))
		vb = scene.context.buffer(np.array(pts, 'f4'))
		return dict(
			_va_screen = scene.context.vertex_array(shader, [(vb, 'f f', 'v_absciss', 'v_alpha')], mode=mgl.LINES),
			_va_ident = scene.context.vertex_array(ident_shader, [(vb, 'f 4x', 'v_absciss')], mode=mgl.LINES),
			)
	
	def stack(self, scene):
		return ( (self, 'ident', 2, self._identify),
				(self, 'screen', 2, self._render))
	
	def _render(self, view):
		self._va_screen.program['color'].write(highlight_color(self, self.color))
		self._va_screen.program['projview'].write(view.uniforms['projview'])
		self._va_screen.program['world'].write(self.world)
		self._va_screen.program['origin'].write(self.origin)
		self._va_screen.program['direction'].write(self.direction)
		self._va_screen.program['interval'] = self._disp_interval(view)
		self._va_screen.render()
	
	def _identify(self, view):
		self._va_ident.program['projview'].write(view.uniforms['projview'])
		self._va_ident.program['world'].write(self.world)
		self._va_ident.program['origin'].write(self.origin)
		self._va_ident.program['direction'].write(self.direction)
		self._va_ident.program['interval'] = self._disp_interval(view)
		self._va_ident.program['ident'] = view.identstep(1)
		self._va_ident.render()
	
	def _disp_interval(self, view):
		if self.interval:	
			return self.interval
		else:
			size = -view.uniforms['view'][3][2]/6
			return (-0.5*size, size)
	


class AnnotationDisplay(Display):
	box: Box
	color: fvec4
	selected: bool
	hovered: bool
	
	def __init__(self, scene, points, color):
		self.color = fvec3(color or settings.colors['annotation'])
		self.box = npboundingbox(points)
		self.selected = False
		self.hovered = False
		
		shader_screen = scene.share(AnnotationDisplay, self._shared)
		shader_ident = scene.share(load_shader_ident, load_shader_ident)
		
		# allocate buffers
		vb = scene.context.buffer(points)
		self._va_screen = scene.context.vertex_array(shader_screen, [(vb, '3f f', 'v_position', 'v_alpha')], mode=mgl.LINES)
		self._va_ident = scene.context.vertex_array(shader_ident, [(vb, '3f 4x', 'v_position')], mode=mgl.LINES)
	
	def _shared(self, scene):
		return scene.context.program(
			vertex_shader=open(resourcedir+'/shaders/annotation.vert').read(),
			fragment_shader=open(resourcedir+'/shaders/annotation.frag').read(),
			)
	
	def stack(self, scene):
		return ( (self, 'ident', 2, self._identify),
				(self, 'screen', 2, self._render)) 
	
	def _render(self, view):
		self._va_screen.program['color'].write(highlight_color(self, self.color))
		self._va_screen.program['proj'].write(view.uniforms['proj'])
		self._va_screen.program['view'].write(view.uniforms['view'] * self.world)
		self._va_screen.render()
	
	def _identify(self, view):
		self._va_ident.program['ident'] = view.identstep(1)
		self._va_ident.program['proj'].write(view.uniforms['proj'])
		self._va_ident.program['view'].write(view.uniforms['view'] * self.world)
		self._va_ident.render()

class BoxDisplay(AnnotationDisplay):
	def __init__(self, scene, box, color=None):
		# place points
		x,y,z = box.size
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

		super().__init__(scene, pts, color)

	
class SplineDisplay(Display):
	''' display for spline curve, with handles around'''
	color: fvec4
	box: Box
	selected: bool
	hovered: bool
	
	def __init__(self, scene, handles, curve, color=None):
		self.color = fvec3(color or settings.colors['line'])
		self.color_handles = fvec3(settings.colors['annotation'])
		self.box = npboundingbox(handles)
		self.selected = False
		self.hovered = False
		
		ctx = scene.context
		vb_handles = ctx.buffer(handles)
		vb_curve = ctx.buffer(curve)
		
		shader = scene.share(load_shader_uniformcolor, load_shader_uniformcolor)
		self._va_handles = ctx.vertex_array(shader, [(vb_handles, '3f4', 'v_position')])
		self._va_curve = ctx.vertex_array(shader, [(vb_curve, '3f4', 'v_position')], mode=mgl.LINE_STRIP)
		
		shader = scene.share(load_shader_ident, load_shader_ident)
		self._va_ident = ctx.vertex_array(shader, [(vb_curve, '3f4', 'v_position')], mode=mgl.LINE_STRIP)
	
	def stack(self, scene):
		return (	(self, 'screen', 1, self._render),
					(self, 'ident', 1, self._identify)	)
	
	def _render(self, view):
		self._va_handles.program['color'].write(fvec4(highlight_color(self, settings.colors['annotation']), 0.6))
		self._va_curve.program['color'].write(fvec4(highlight_color(self, self.color), 1))
		
		self._va_handles.program['view'].write(view.uniforms['view'] * self.world)
		self._va_handles.program['proj'].write(view.uniforms['proj'])
		view.scene.context.point_size = 4
		
		self._va_handles.program['layer'] = -2e-6
		self._va_handles.program['color'].write(fvec4(highlight_color(self, self.color_handles), 0.5))
		self._va_handles.render(mgl.POINTS)
		self._va_handles.render(mgl.LINE_STRIP)
		
		self._va_curve.program['layer'] = -1e-6
		self._va_curve.program['color'].write(fvec4(highlight_color(self, self.color), 1))
		self._va_curve.render()
		
	def _identify(self, view):
		self._va_ident.program['ident'] = view.identstep(1)
		self._va_ident.program['view'].write(view.uniforms['view'] * self.world)
		self._va_ident.program['proj'].write(view.uniforms['proj'])
		self._va_ident.render()
		

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
	'''
	center: fvec3
	''' 3d position of the center of the grid '''
	unit: float
	''' factor on the distance between dots '''
	color: fvec3
	''' base color of the grid, with an alpha channel '''
	contrast: float
	''' contrast between the smallest division and the biggest '''
			
	def __init__(self, scene, center=None, unit=1., color=None, contrast=1.8):
		self.unit = unit
		self.center = fvec3(center or 0)
		self.color = fvec4(color) if color else fvec4(settings.colors['point'],1)
		self.contrast = contrast
		
		self._va = scene.share(type(self), self._share)

	def _share(self, scene):
		n = 101
		h = n//2
		pts = np.empty((n,n), dtype='f4, f4, f2')
		for i in range(n):
			for j in range(n):
				pts[i,j] = (i-h, j-h, (1+min(digitfit(i), digitfit(j))))
		
		shader = scene.context.program(
			vertex_shader=open(resourcedir+'/shaders/viewgrid.vert').read(),
			fragment_shader=open(resourcedir+'/shaders/viewgrid.frag').read(),
			)
		vb = scene.context.buffer(pts)
		return scene.context.vertex_array(shader, [(vb, '2f4 f2', 'v_position', 'v_opacity')], mode=mgl.POINTS)
	
	def stack(self, scene):
		return (self, 'screen', 3, self._render),
	
	def _render(self, view):
		center = fvec3( view.uniforms['view'] * self.world * fvec4(self.center,1) )
		if not center.z < 0:	return
		zlog = log(-center.z)/log(10)
		sizelog = floor(zlog)
		
		view.scene.context.point_size = 1/400 * min(view.uniforms['size'])
		self._va.program['color'].write(fvec4(
				self.color.rgb, 
				self.color.a * exp(self.contrast*(-1+sizelog-zlog)),
				))
		self._va.program['size'] = self.unit * 10**(sizelog-1)
		self._va.program['centerdist'] = -center.z
		self._va.program['proj'].write(view.uniforms['proj'])
		self._va.program['contrast'] = self.contrast
		self._va.render()


# class CutPlane(Displau):
#     ''' show a plane cutting all meshes '''


def tupledisplay(scene: Scene, t:tuple) -> Display:
	if primitives.isaxis(t):	return AxisDisplay(scene, t)
	# if not found: empty display
	return Display()

Scene.overrides.update({
	dvec3:   PointDisplay,
	fvec3:  PointDisplay,
	tuple:  tupledisplay,
	Box:    BoxDisplay,
	})
