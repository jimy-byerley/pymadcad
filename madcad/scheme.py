# This file is part of pymadcad,  distributed under license LGPL v3

''' This module provides annotation functions to quickly measure and show things on meshes
	
	This is more to be considered as a rendering feature, rather than a proper measuring system.
	Everything is built on the class `Scheme` that provide a very versatile way to structure and render simple but animated schematics
	
	Those functions are designed to be very simple to use, (sometimes even minimalistic)
		
		>>> mesh = brick(width=vec3(3,2,1))
		>>> show([
		... 	mesh,
		... 	note_distance_planes(mesh.group(4), mesh.group(2)),
		... 	note_leading(mesh.group(3), text='truc'),
		... 	])
			
'''

import moderngl as mgl
import numpy.core as np
import glm
from operator import itemgetter
from collections import deque

from .mathutils import *
from .rendering import Display
from .common import ressourcedir
from .mesh import Mesh, Web, Wire, web, wire, mesh_distance, connef, connpe, edgekey, arrangeface, arrangeedge
from .rendering import Displayable, writeproperty, overrides
from .primitives import *
from . import mathutils
from . import generation as gt
from . import text as txt
from . import settings

__all__ = ['Scheme', 
			'note_leading', 
			'note_floating', 
			'note_label', 
			'note_distance', 
			'note_angle', 
			'note_distance_planes',
			'note_distance_set',
			'note_angle_planes', 
			'note_angle_edge',
			'note_radius',
			'note_bounds',
			]


class Scheme:
	''' an object containing schematics. 
	
		This is a buffer object, it isnot intended to be useful to modify a scheme.
		
		Attributes:
		
			spaces (list):       a space is any function giving a mat4 to position a point on the screen (openGL convensions as used)
			
			vertices (list):	    
			
				a vertex is a tuple
				
				`(space id, position, normal, color, layer, track, flags)`
			
			primitives (list):   
			
				list of buffers (of point indices, edges, triangles, depending on the exact primitive type), associaded to each supported shader in the scheem
				
				currently supported shaders are:
				
				- `'line'`  uniform opaque/transparent lines
				- `'fill'`  uniform opaque/transparent triangles
				- `'ghost'` triangles of surface that fade when its normal is close to the view
			
			components (list):   objects to display setting their local space to one of the spaces
				
			annotation (bool):     whether this object must be considered as an annotation (and hidden with others when requested)
			
			current (dict):     last vertex definition, implicitely reused for convenience
	'''
	def __init__(self, vertices=None, spaces=None, primitives=None, annotation=True, **kwargs):
		self.vertices = vertices or [] # list of vertices
		self.spaces = spaces or []	# definition of each space
		self.primitives = primitives or {} # list of indices for each shader
		self.components = []	# displayables associated to spaces
		self.annotation = annotation	# flag saying if this object is an annotation
		# for creation: last vertex inserted
		self.current = {'color':fvec4(settings.display['annotation_color'],1), 'flags':0, 'layer':0, 'space':world, 'shader':'wire', 'track':0, 'normal':fvec3(0)}
		self.set(**kwargs)
		
	def __iadd__(self, other):
		''' concatenante the content of an other scheme in the current one
			
			the current settings stay as before concatenation
		'''
		ls = len(self.spaces)
		lv = len(self.vertices)
		lt = max(self.tracks)+1
		self.spaces.extend(other.spaces)
		self.components.extend(other.components)
		self.vertices.extend([
							v[0] + ls, 
							*v[1:5],
							v[5] + lt, 
							v[6],
						]  for v in other.vertices)
		for shader, prims in other.primitives:
			if shader not in self.primitives:
				self.primitives[shader] = []
			self.primitives[shader].extend(tuple(i+lv  for i in p) for p in prims)
		
		return self
		
	def __add__(self, other):
		''' return the union of the two schemes 
		
			the current settings are those from first scheme
		'''
		r = deepcopy(self)
		r += other
		return r
		
	def set(self, *args, **kwargs):
		''' change the specified attributes in the current default vertex definition '''
		if args:
			if len(args) == 1 and isinstance(args[0], dict):
				kwargs = args[0]
			else:
				raise TypeError('Scheme.set expects keywords argument or one unique dictionnary argument')
		self.current.update(kwargs)
		# register the space if not already known
		if not isinstance(self.current['space'], int):
			try:	i = self.spaces.index(self.current['space'])
			except ValueError:	
				i = len(self.spaces)
				self.spaces.append(self.current['space'])
			self.current['space'] = i
		if not isinstance(self.current['color'], fvec4):
			self.current['color'] = fvec4(self.current['color'])
		
		return self
	
	def add(self, obj, **kwargs):
		''' add an object to the scheme
			if it is a mesh it's merged in the current buffers 
			else it is added as a component to the current space
		'''
		self.set(kwargs)
		if self.current['shader'] not in self.primitives:
			self.primitives[self.current['shader']] = indices = []
		else:
			indices = self.primitives[self.current['shader']]
		l = len(self.vertices)
		
		if isinstance(obj, (Mesh,Web)):
			self.vertices.extend([
								self.current['space'], 
								fvec3(p), 
								self.current['normal'], 
								self.current['color'], 
								self.current['layer'], 
								self.current['track'], 
								self.current['flags'],
							]  for p in obj.points)
		if isinstance(obj, Mesh):
			indices.extend(((a+l, b+l, c+l)  for a,b,c in obj.faces))
			for f, track in zip(obj.faces, obj.tracks):
				for p in f:
					self.vertices[p+l][5] = track
			for i,n in enumerate(obj.vertexnormals()):
				self.vertices[i+l][2] = n
		elif isinstance(obj, Web):
			indices.extend(((a+l, b+l)  for a,b in obj.edges))
			for e, track in zip(obj.edges, obj.tracks):
				for p in e:
					self.vertices[p+l][5] = track
		
		elif hasattr(obj, '__iter__'):
			n = len(self.vertices)
			for obj in obj:
				if isinstance(obj, (fvec3, vec3)):
					self.vertices.append([
								self.current['space'], 
								fvec3(obj), 
								self.current['normal'], 
								self.current['color'], 
								self.current['layer'], 
								self.current['track'], 
								self.current['flags'],
								])
					n += 1
				else:
					self.add(obj)
			indices.extend((i,i+1)	for i in range(l, n-1))
		else:
			self.component(obj)
			
		return self
	
	def component(self, obj, **kwargs):
		''' add an object as component associated to the current space '''
		self.set(**kwargs)
		self.components.append((self.current['space'], obj))
		
		return self
	
	class display(Display):
		''' display for schemes
			
			attributes:
			:spaces:       numpy array of matrices for each space, sent as uniform to the shader
			:vb_vertices:  vertex buffer for vertices
			:vas:          vertex array associated to each shader
		'''
		max_spaces = 32
		
		def __init__(self, scene, sch):
			ctx = scene.ctx
			
			# set display params
			self.annotation = sch.annotation
			
			# load the ressources
			self.shaders, self.shader_ident = scene.ressource('scheme', self.load)
			
			# switch to array indexed spaces
			self.spaces = glm.array.zeros(self.max_spaces, fmat4)
			self.spacegens = list(sch.spaces)
			if len(self.spacegens) > self.max_spaces:		
				print('warning: the number of local spaces exceeds the arbitrary build-in limit of {}'.format(self.max_spaces))
			
			self.components = [(space,scene.display(obj))	for space,obj in sch.components]
			
			
			self.nidents = max(v[5] for v in sch.vertices)+1
			self.box = boundingbox(
						(fvec3(v[1]) for v in sch.vertices if self.spacegens[v[0]] is world), 
						default=Box(center=fvec3(0), width=fvec3(-inf)),
						)
			
			# prepare the buffer of vertices
			vertices = np.empty(len(sch.vertices), 'u1, 3f4, 3f4, 4u1, f4, u2, u1')
			for i,v in enumerate(sch.vertices):
				vertices[i] = (
					*v[:3],
					u8vec4(v[3]*255), 
					*v[4:]
					)			
			self.vb_vertices = ctx.buffer(vertices)
			verticesdef = [(self.vb_vertices, 'u1 3f4 3f4 4f1 f4 u2 u1', 
								'space', 
								'v_position', 
								'v_normal', 
								'v_color', 
								'v_layer', 
								'v_ident', 
								'v_flags')]
			
			# prepare the rending commands
			ident_triangles = []
			ident_lines = []
			self.vas = {}
			self.vai_triangles = None
			self.vai_lines = None
			for shname,batch in sch.primitives.items():
				if not batch:	continue
				if shname not in self.shaders:	raise KeyError('no shader for name {}'.format(repr(shname)))
				
				prim, shader = self.shaders[shname]
				vb_indices = ctx.buffer(np.array(batch, 'u4'))
				self.vas[shname] = (prim, ctx.vertex_array(shader, verticesdef, vb_indices, skip_errors=True))
				if prim == mgl.LINES:			ident_lines.extend(batch)
				elif prim == mgl.TRIANGLES:		ident_triangles.extend(batch)
			
			if ident_triangles:	self.vai_triangles	= ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_triangles, 'u4')), skip_errors=True)
			if ident_lines:		self.vai_lines 		= ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_lines, 'u4')), skip_errors=True)
			
		def __del__(self):
			for prim, va in self.vas.values():
				va.release()
			self.vas.clear()
			if self.vai_triangles:
				self.vai_triangles.release()
			if self.vai_lines:
				self.vai_lines.release()
			if self.vb_vertices:
				self.vb_vertices.release()
				self.vb_vertices = None
			
		def load(self, scene):
			''' load shaders and all static data for the current opengl context '''
			vert = open(ressourcedir+'/shaders/scheme.vert').read()
			shader_ident = scene.ctx.program(
						vertex_shader=vert,
						fragment_shader=open(ressourcedir+'/shaders/scheme-ident.frag').read(),
						)
			shaders = {
				'line': (mgl.LINES, scene.ctx.program(
						vertex_shader=vert,
						fragment_shader=open(ressourcedir+'/shaders/scheme-uniform.frag').read(),
						)),
				'fill': (mgl.TRIANGLES, scene.ctx.program(
						vertex_shader=vert,
						fragment_shader=open(ressourcedir+'/shaders/scheme-uniform.frag').read(),
						)),
				'ghost': (mgl.TRIANGLES, scene.ctx.program(
						vertex_shader=vert,
						fragment_shader=open(ressourcedir+'/shaders/scheme-ghost.frag').read(),
						)),
				}
			return shaders, shader_ident
			
		def compute_spaces(self, view):
			''' computes the new spaces for this frame
				this is meant to be overriden when new spaces are required 
			'''
			view.uniforms['world'] = self.world
			for i,gen in enumerate(self.spacegens):
				self.spaces[i] = gen(view)
			if self.components:
				invview = affineInverse(view.uniforms['view'])
				for space,disp in self.components:
					disp.world = invview * self.spaces[space]
		
		def render(self, view):
			''' render each va in self.vas '''
			#self.compute_spaces(view)
			for name in self.vas:
				shader = self.shaders[name][1]
				prim, va = self.vas[name]
				shader['spaces'].write(self.spaces)
				shader['proj'].write(view.uniforms['proj'])
				shader['highlight'].write( fvec4(fvec3(settings.display['select_color_line']), 0.5) if self.selected else fvec4(0) )
				va.render(prim)
		
		def identify(self, view):
			''' render all the triangles and lines for identification '''
			self.shader_ident['startident'] = view.identstep(self.nidents)
			self.shader_ident['spaces'].write(self.spaces)
			self.shader_ident['proj'].write(view.uniforms['proj'])
			
			if self.vai_lines:		self.vai_lines.render(mgl.LINES)
			if self.vai_triangles:	self.vai_triangles.render(mgl.TRIANGLES)
		
		def stack(self, scene):
			if self.annotation and not scene.options['display_annotations']:
				return
			yield ((), 'screen', -1, self.compute_spaces)
			yield ((), 'screen', 2, self.render) 
			yield ((), 'ident', 2, self.identify)
			for space,disp in self.components:
				yield from disp.stack(scene)
				
class SchemeInstance:
	def __init__(self, name, scheme, *kwargs):
		self.name = name
		self.scheme = scheme
		vars(self).update(kwargs)
	
	class display(Scheme.display):
		def __init__(self, scene, instance):
			self.instance = instance
			disp = scene.ressource(id(instance.scheme), lambda: inst.scheme.display(scene, instance.scheme))
			vars(self).update(vars(disp))
			self.spaces = deepcopy(disp.spaces)
			
		def compute_spaces(self, view):
			''' computes the new spaces for this frame
				this is meant to be overriden when new spaces are required 
			'''
			view.uniforms['world'] = self.world
			for i,gen in enumerate(self.spacegens):
				self.spaces[i] = gen(view, self.instance)
			if self.components:
				invview = affineInverse(view.uniforms['view'])
				for space,disp in self.components:
					disp.world = invview * self.spaces[space]

# create standard spaces

def view(view):
	proj = view.uniforms['proj']
	return fmat4(1/proj[0][0],  0,0,0,
				0, 1/proj[1][1], 0,0,
				0,0,1,0,
				0,0,-1,1)

def screen(view):
	return fmat4(view.target.width/2,0,0,0,
				0,view.target.height/2,0,0,
				0,0,1,0,
				0,0,-1,1)

def ubiquity(view):
	#return translate(fvec3(0,0,-1)) * fmat4(fmat3(view.uniforms['view']) * fmat3(view.uniforms['world']))
	proj = view.uniforms['proj']
	f = - (proj[3][2]-proj[3][3]) / (proj[2][2] - proj[2][3])
	n = - (proj[3][2]+proj[3][3]) / (proj[2][2] + proj[2][3])
	d = 3*n
	e = proj * fvec4(1,1,d,1)
	e /= e[3]
	return translate(fvec3(0,0,d)) * fmat4(
				fmat3(1/e[1]) 
				* fmat3(view.uniforms['view']) 
				* fmat3(view.uniforms['world']))

def world(view):
	return view.uniforms['view'] * view.uniforms['world']

def halo_world(position):
	position = fvec4(position,1)
	def mat(view):
		center = view.uniforms['view'] * (view.uniforms['world'] * position)
		m = fmat4(1)
		m[3] = center
		return m
	return mat
def halo_view(position):
	position = fvec4(position,1)
	def mat(view):
		center = view.uniforms['view'] * (view.uniforms['world'] * position)
		proj = view.uniforms['proj']
		e = proj * fvec4(1,1,center.z,1)
		e /= e[3]
		m = fmat4(1/e[1])
		m[3] = center
		return m
	return mat
def halo_screen(position):
	position = fvec4(position,1)
	def mat(view):
		center = view.uniforms['view'] * (view.uniforms['world'] * position)
		e = view.uniforms['proj'] * fvec4(1,1,center.z,1)
		e /= e[3]
		m = fmat4(2/(e[1]*view.target.height))
		m[3] = center
		return m
	return mat

def scale_screen(center):
	def mat(view):
		m = view.uniforms['view'] * view.uniforms['world']
		e = view.uniforms['proj'] * fvec4(1,1,(m*center).z,1)
		e /= e[3]
		return m * translate(center) * scale(fvec3(2 / (e[1]*view.target.height)))
	return mat

def scale_view(center):
	def mat(view):
		m = view.uniforms['view'] * view.uniforms['world']
		e = view.uniforms['proj'] * fvec4(1,1,(m*center).z,1)
		e /= e[3]
		return m * translate(center) * scale(fvec3(1 / e[1]))
	return mat



class note_leading_display(Display):
	def __init__(self, scene, origin, offset, comment, annotation=True):
		self.origin = fvec3(origin)
		self.offset = fvec3(offset)
		self.comment = comment
		self._world = fmat4(1)
		self.annotation = annotation
		
		def build(side):
			color = settings.display['annotation_color']
			sch = Scheme(color=fvec4(color,0.7))
			sch.add([self.origin, self.origin+self.offset], shader='line', space=world)
			x,y,z = dirbase(normalize(vec3(self.offset)))
			sch.add(
				gt.revolution(2*pi, (vec3(0), z), 
					web([vec3(0), 16*z+4*x]), 
					resolution=('div',8),
					), 
				shader='fill',
				space=scale_screen(self.origin),
				)
			sch.set(space=halo_screen(self.origin+self.offset))
			font = settings.display['view_font_size']
			sch.add([vec3(0), vec3(side*font*2, 0, 0)], shader='line')
			sch.add(txt.Text(vec3(side*font*3, 0, 0), self.comment, align=('left' if side>0 else 'right', 0.5), color=fvec4(color,1), size=font))
			
			return scene.display(sch)
		self.disp = build(-1), build(1)
		
	@property
	def selected(self):
		return self.disp[0].selected
	@selected.setter
	def selected(self, value):
		for disp in self.disp:
			disp.selected = value
		
	def side(self, view):
		return int((fmat3(view.uniforms['view']) * (fmat3(self.world) * self.offset))[0] > 0)
		
	def compute_spaces(self, view):
		side = int((fmat3(view.uniforms['view']) * (fmat3(self.world) * self.offset))[0] > 0)
		self.active_disp = self.disp[side]
		self.active_disp.compute_spaces(view)
	
	def render(self, view):
		self.active_disp.render(view)
		for _,comp in self.active_disp.components:
			comp.render(view)		
	
	def identify(self, view):
		self.active_disp.identify(view)
	
	def stack(self, scene):
		if self.annotation and not scene.options['display_annotations']:
			return ()
		return (	((), 'screen', -1, self.compute_spaces),
					((), 'screen', 2, self.render), 
					((), 'ident', 2, self.identify),
					)
		
	@writeproperty
	def world(self, value):
		for disp in self.disp:	disp.world = value


def mesh_placement(mesh) -> '(pos,normal)':
	''' return an axis for placement of a note on the given object '''
	if isinstance(mesh, Mesh):
		# group center normal
		center = mesh.barycenter()
		f = min(mesh.faces,
				key=lambda f:  distance2(center, sum(mesh.facepoints(f))/3)
				)
		normal = mesh.facenormal(f)
		pos = sum(mesh.facepoints(f)) / 3
	
	elif isinstance(mesh, Web):
		center = mesh.barycenter()
		e = min(mesh.edges,
				key=lambda e:  distance2(center, (mesh.points[e[0]]+mesh.points[e[1]])/2)
				)
		normal = dirbase(normalize(mesh.points[e[0]]-mesh.points[e[1]]))[0]
		pos = mix(mesh.points[e[0]], mesh.points[e[1]], 0.5)
		
	elif isinstance(mesh, Wire):
		return mesh_placement(web(mesh))
		
	elif isaxis(mesh):
		pos, normal = mesh
	
	elif isinstance(mesh, vec3):
		normal = vec3(0)
		pos = mesh
		
	elif hasattr(mesh, 'mesh'):
		return mesh_placement(mesh.mesh())
	
	else:
		raise TypeError('unable to place note on a {}'.format(type(mesh)))
	return pos, normal

def note_leading(placement, offset=None, text='here'):
	''' place a leading note at given position
	
		`placement` can be any of `Mesh, Web, Wire, axis, vec3`
	'''
	origin, normal = mesh_placement(placement)
	if not offset:
		offset = 0.2 * length(boundingbox(placement).width) * normal
	elif isinstance(offset, (float,int)):
		offset = offset*normal
	elif not isinstance(offset, vec3):
		raise TypeError('offset must be scalar or vector')
	return Displayable(note_leading_display, origin, offset, text)


def note_floating(position, text):
	''' place a floating note at given position '''
	return txt.Text(position, text, align=(0,0), color=settings.display['annotation_color'], size=settings.display['view_font_size'])

def note_distance(a, b, offset=0, project=None, d=None, tol=None, text=None, side=False):
	''' place a distance quotation between 2 points, 
		the distance can be evaluated along vector `project` if specified 
	'''
	# get text to display
	if not project:					project = normalize(b-a)
	elif dot(project, b-a) < 0:		project = -project
	if not d:	d = abs(dot(b-a, project))
	if not text:
		if isinstance(tol,str): text = '{d:.4g}  {tol}'
		elif tol:               text = '{d:.4g}  ± {tol}'
		else:                   text = '{d:.4g}'
	text = text.format(d=d, tol=tol)
	color = settings.display['annotation_color']
	# convert input vectors
	x = dirbase(project, b-a)[0]
	z = project if side else -project
	if not isinstance(offset, vec3):
		offset = offset * x
	shift = 0.5 * mathutils.project(b-a, x)	if length2(x) else 0
	ao = a + offset + shift
	bo = b + offset - shift
	# create scheme
	sch = Scheme()
	sch.set(shader='line', layer=1e-4, color=fvec4(color,0.3))
	sch.add([a, ao])
	sch.add([b, bo])
	sch.set(layer=-1e-4, color=fvec4(color,0.7))
	sch.add([ao, bo])
	sch.add(txt.Text(
				mix(ao,bo,0.5), 
				text, 
				align=('center', 0.5), 
				size=settings.display['view_font_size'], 
				color=fvec4(color,1)))
	sch.set(shader='fill')
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),project), 
				web([vec3(0), 3*x-12*z]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(ao)))
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),project), 
				web([vec3(0), 3*x+12*z]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(bo)))
	return sch
	
def note_distance_planes(s0, s1, offset=None, d=None, tol=None, text=None):
	''' place a distance quotation between 2 meshes 
		
		`s0` and `s1` can be any of `Mesh, Web, Wire`
	'''
	#n0 = s0.facenormal(0)
	#n1 = s1.facenormal(1)
	#print('angle', length(cross(n0,n1)), n0, n1)
	
	p0, n0 = mesh_placement(s0)
	p1, n1 = mesh_placement(s1)
	if length2(cross(n0,n1)) > 1e-10:
		raise ValueError('surfaces are not parallel')
	if not offset:
		offset = length(noproject(boundingbox(s0,s1).width, n0)) * 0.6
	return note_distance(p0, p1, offset, n0, d, tol, text)
	
def note_distance_set(s0, s1, offset=0, d=None, tol=None, text=None):
	''' place a distance quotation between 2 objects. This is the distance between the closest elements of both sets 
	
		`s0` and `s1` can be any of `Mesh, Web, Wire, vec3`
	'''
	dist, p0, p1 = mesh_distance(s0, s1)
	if not d:	d = dist
	# take the point and the higher dimension primitive
	if not isinstance(p0,int):	s0,p0, s1,p1 = s1,p1, s0,p0
	p0 = s0.points[p0]
	pts = s1.points
	if isinstance(p1, tuple):
		if len(p1) == 2:	p1 = pts[p1[0]] + project(p0-pts[p1[0]], pts[p1[1]]-pts[p1[0]])
		elif len(p1) == 3:	p1 = p0 + project(pts[p1[0]]-p0, cross(pts[p1[1]]-pts[p1[0]], pts[p1[2]]-pts[p1[0]]))
	else:
		p1 = s1.points[p1]
	
	return note_distance(p0, p1, offset, None, d, tol, text)
		

def note_angle(a0, a1, offset=0, d=None, tol=None, text=None, unit='deg', side=False):
	''' place an angle quotation between 2 axis '''
	o0, d0 = a0
	o1, d1 = a1
	z = normalize(cross(d0,d1))
	if not isfinite(z):	
		raise ValueError('axis are parallel')
	x0 = cross(d0,z) * (1 if side else -1)
	x1 = cross(d1,z) * (1 if side else -1)
	shift = project(o1-o0, z) * 0.5
	# add it but in a new copy of the vectors
	o0 = o0 + shift
	o1 = o1 + shift
	# get text to display
	if not d:	d = anglebt(d0, d1)
	if unit == 'deg':
		d *= 180/pi
		unit = '°'
	if not text:
		if isinstance(tol,str): text = '{d:.4g}{unit}  {tol}'
		elif tol:               text = '{d:.4g}{unit}  ± {tol}'
		else:                   text = '{d:.4g}{unit}'
	text = text.format(d=d, tol=tol, unit=unit)
	color = settings.display['annotation_color']
	# arc center
	if o1 == o0:	center = o0
	else:			center = o0 + unproject(project(o1-o0, x1), d0)
	radius = mix(distance(o0,center), distance(o1,center), 0.5) + offset
	if not radius:	radius = 1
	# arc extremities
	p0 = center+radius*d0
	p1 = center+radius*d1
	sch = Scheme()
	sch.set(shader='line', layer=1e-4, color=fvec4(color,0.3))
	sch.add([p0, o0])
	sch.add([p1, o1])
	arc = ArcCentered((center,z), p0, p1, ('rad',0.05)).mesh()
	sch.add(arc, color=fvec4(color,0.7))
	sch.set(layer=-1e-4)
	sch.add(txt.Text(
				arc[len(arc)//2], 
				text, 
				align=('center', 0.5), 
				size=settings.display['view_font_size'], 
				color=fvec4(color,1)))
	sch.set(shader='fill')
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),x0), 
				web([vec3(0), 3*d0+12*x0]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(p0)))
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),x1), 
				web([vec3(0), 3*d1-12*x1]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(p1)))
	return sch
	
def note_radius(mesh, offset=None, d=None, tol=None, text=None, propagate=2):
	''' place a curvature radius quotation. This will be the minimal curvature radius ovserved in the mesh 
		As a mesh is generaly speaking an approximation of the desired shape, the radius may be approximative as well
	'''
	if isinstance(mesh, Mesh):
		normals = mesh.vertexnormals()
		radius, place = mesh_curvature_radius(mesh, normals=normals, propagate=propagate)
		normal = normals[place]
	elif isinstance(mesh, Web):
		conn = connpp(mesh.edges)
		radius, place = mesh_curvature_radius(mesh, conn=conn, propagate=propagate)
		normal = - normalize(sum(normalize(mesh.points[p]-mesh.points[place])  for p in conn[place]))
	elif isinstance(mesh, wire):
		normals = mesh.vertexnormals()
		radius, place = mesh_curvature_radius(mesh, normals=normals, propagate=propagate)
		normal = normals[place]
	else:
		raise TypeError('input mesh must be Mesh, Web or Wire')
	if not offset:
		offset = 0.2 * length(boundingbox(mesh).width)
	
	if isinstance(place, tuple):
		place = sum(mesh.points[i]  for i in place) / len(place)
	else:
		place = mesh.points[place]
	
	return note_leading((place,normal), offset=offset, text=text or 'R {:.4g}'.format(radius))
	
import numpy.linalg
import numpy as np
from .mesh import connpp
	
def mesh_curvature_radius(mesh, conn=None, normals=None, propagate=2) -> '(distance, point)':
	''' find the minimum curvature radius of a mesh.
	
		Parameters:
		
			mesh:			the surface/line to search
			conn:			a point-to-point connectivity (computed if not provided)
			normals:		the vertex normals (computed if not provided)
			propagate(int):	the maximum propagation rank for points to pick for the regression
	
		Returns:	`(distance: float, point: int)` where primitives varies according to the input mesh dimension
	'''
		
	def propagate_pp(conn, start, maxrank):
		front = [(0,s) for s in start]
		seen = set()
		while front:
			rank, p = front.pop()
			if p in seen:	continue
			seen.add(p)
			yield p
			if rank < maxrank:
				for n in conn[p]:
					if n not in seen:	
						front.append((rank+1, n))
	
	if isinstance(mesh, Mesh):		
		if not conn:	conn = connpp(mesh.faces)
		if not normals:	normals = mesh.vertexnormals()
		it = ( (p, list(propagate_pp(conn, [p], propagate)))   for p in range(len(mesh.points)) )
	elif isinstance(mesh, Web):	
		if not conn:	conn = connpp(mesh.edges)
		if not normals:	
			normals = [vec3(0)  for p in mesh.points]
			pts = mesh.points
			for e in mesh.edges:
				d = pts[e[0]] - pts[e[1]]
				normals[e[0]] += d
				normals[e[1]] -= d
			for i,n in enumerate(normals):
				normals[i] = normalize(n)
		it = ( (p, list(propagate_pp(conn, [p], propagate)))   for p in range(len(mesh.points)) )
	elif isinstance(mesh, Wire):
		if not normals:	normals = mesh.vertexnormals()
		it = ( (mesh.indices[i], mesh.indices[i-propagate:i+propagate])   for i in range(len(mesh.indices)) )
	else:
		raise TypeError('bad input type')
	
	def analyse():
		pts = mesh.points		
		for p, neigh in it:
			# decide a local coordinate system
			u,v,w = dirbase(normals[p])
			# get neighboors contributions
			b = np.empty(len(neigh))
			a = np.empty((len(neigh), 14))
			for i,n in enumerate(neigh):
				e = pts[n] - pts[p]
				b[i] = dot(e,w)
				eu = dot(e,u)
				ev = dot(e,v)
				# these are the monoms to compose to build a polynom approximating the surface until 4th-order derivatives
				a[i] = (	eu**2, ev**2, eu*ev,
							eu, ev,
							eu**3, eu**2*ev, eu*ev**2, ev**3,
							eu**4, eu**3*ev, eu**2*ev**2, eu*ev**3, ev**4,
							)

			# least squares resulution, the complexity is roughly the same as inverting a mat3
			(au, av, auv, *_), residuals, *_ = np.linalg.lstsq(a, b)
			# diagonalize the curve tensor to get the principal curvatures
			diag, transfer = np.linalg.eigh(mat2(2*au, auv, 
													auv, 2*av))
			yield 1/np.max(np.abs(diag)), p
	
	return min(analyse(), key=itemgetter(0), default=None)
	
	
def mesh_curvature_radius(mesh, conn=None, normals=None, propagate=2) -> '(distance, point)':
	''' find the minimum curvature radius of a mesh.
	
		Parameters:
		
			mesh:			the surface/line to search
			conn:			a point-to-point connectivity (computed if not provided)
			normals:		the vertex normals (computed if not provided)
			propagate(int):	the maximum propagation rank for points to pick for the regression
	
		Returns:	`(distance: float, point: int)` where primitives varies according to the input mesh dimension
	'''
		
	curvatures = mesh_curvatures(mesh, conn, normals, propagate)
	
	place = min(range(len(mesh.points)),
				key=lambda p: 1/np.max(np.abs(curvatures[p][0])), 
				default=None)
	return 1/np.max(np.abs(curvatures[place][0])), place
	
def mesh_curvatures(mesh, conn=None, normals=None, propagate=2):
	''' compute the curvature around a point in a mesh/web/wire
	
		Parameters:
		
			mesh:			the surface/line to search
			conn:			a point-to-point connectivity (computed if not provided)
			normals:		the vertex normals (computed if not provided)
			propagate(int):	the maximum propagation rank for points to pick for the regression
	
		Returns:	
			
			`[(tuple, mat3)]`
			
			where the `tuple` contains the curvature in each of the column directions in the `mat3`. The `mat3` has the principal directions of curvature
	'''
	pts = mesh.points
		
	# propagate though the mesh and return seen points
	def propagate_pp(conn, start, maxrank):
		front = deque((0,s) for s in start)
		seen = set()
		while front:
			rank, p = front.popleft()
			if p in seen:	continue
			seen.add(p)
			if rank < maxrank:
				for n in conn[p]:
					if n not in seen:	
						front.append((rank+1, n))
		return seen
	
	if isinstance(mesh, Mesh):		
		if not conn:	conn = connpp(mesh.faces)
		if not normals:	normals = mesh.vertexnormals()
		it = ( (p, propagate_pp(conn, [p], propagate))   for p in range(len(mesh.points)) )
	elif isinstance(mesh, Web):	
		if not conn:	conn = connpp(mesh.edges)
		if not normals:	
			normals = [vec3(0)  for p in mesh.points]
			for e in mesh.edges:
				d = pts[e[0]] - pts[e[1]]
				normals[e[0]] += d
				normals[e[1]] -= d
			for i,n in enumerate(normals):
				normals[i] = normalize(n)
		it = ( (p, propagate_pp(conn, [p], propagate))   for p in range(len(mesh.points)) )
	elif isinstance(mesh, Wire):
		if not normals:	normals = mesh.vertexnormals()
		it = ( (mesh.indices[i], mesh.indices[i-propagate:i+propagate])   for i in range(len(mesh.indices)) )
	else:
		raise TypeError('bad input shape type')
	
	curvatures = [None]*len(pts)
	
	for p, neigh in it:
		# decide a local coordinate system
		u,v,w = dirbase(normals[p])
		# get neighboors contributions
		b = np.empty(len(neigh))
		a = np.empty((len(neigh), 14))
		for i,n in enumerate(neigh):
			e = pts[n] - pts[p]
			b[i] = dot(e,w)
			eu = dot(e,u)
			ev = dot(e,v)
			# these are the monoms to compose to build a polynom approximating the surface until 4th-order derivatives
			a[i] = (	eu**2, ev**2, eu*ev,  # what we want to get
						# the others terms are only present to catch the surface regularities and not disturb the curvature terms
						eu, ev,
						eu**3, eu**2*ev, eu*ev**2, ev**3,
						eu**4, eu**3*ev, eu**2*ev**2, eu*ev**3, ev**4,
						)

		# least squares resulution, the complexity is roughly the same as inverting a mat3
		(au, av, auv, *_), residuals, *_ = np.linalg.lstsq(a, b, rcond=None)
		# diagonalize the curve tensor to get the principal curvatures
		diag, transfer = np.linalg.eigh(mat2(2*au, auv, 
												auv, 2*av))
		curvatures[p] = diag, mat3(u,v,w) * mat3(mat2(transfer))
		
	return curvatures


def note_bounds(obj):
	''' create dimension annotations on the boundingbox of an object '''
	box = boundingbox(obj)
	size = 0.05 * length(box.width)
	return [
		note_distance(box.min, vec3(box.max.x, box.min.y, box.min.z), offset=size*vec3(0,-1,-1)),
		note_distance(box.min, vec3(box.min.x, box.max.y, box.min.z), offset=size*vec3(-1,0,-1)),
		note_distance(box.min, vec3(box.min.x, box.min.y, box.max.z), offset=size*vec3(-1,-1,0)),
		box,
		]


	
def _mesh_direction(mesh):
	if isinstance(mesh, Mesh):	return mesh.facenormal(0)
	elif isinstance(mesh, Web):	return mesh.edgedirection(0)
	elif isinstance(mesh, Wire):	return mesh[1]-mesh[0]
	elif isaxis(mesh):	return mesh[1]
	else:
		raise TypeError('only Mesh and Web are supported')
	
def note_angle_planes(s0, s1, offset=0, d=None, tol=None, text=None, unit='deg'):
	''' place an angle quotation between 2 meshes considered to be plane (surface) or straight (curve) 
	
		`s0` and `s1` can be any of `Mesh, Web, Wire, Axis`
	'''
	d0, d1 = _mesh_direction(s0), _mesh_direction(s1)
	z = normalize(cross(d0, d1))
	if not isfinite(z):	
		raise ValueError('planes are parallel')
	if isinstance(s0, Mesh) or isaxis(s0):	d0 = cross(d0,z)
	if isinstance(s1, Mesh) or isaxis(s1):	d1 = cross(z,d1)
	return note_angle(
				(mesh_placement(s0)[0], d0), 
				(mesh_placement(s1)[0], d1), 
				offset, d, tol, text, unit)
				
def note_angle_edge(part, edge, offset=0, d=None, tol=None, text=None, unit='deg'):
	''' place an angle quotation around a mesh edge '''
	f0 = None
	f1 = None
	for face in part.faces:
		for i in range(3):
			if face[i-1] == edge[0] and face[i] == edge[1]:	
				f0 = face
			elif face[i-1] == edge[1] and face[i] == edge[0]:
				f1 = face
	if not f0 or not f1:
		raise ValueError("edge {} doesn't exist or is not between 2 faces".format(f0))
	d0 = part.facenormal(f0)
	d1 = part.facenormal(f1)
	z = normalize(cross(d0,d1))
	o = mix(part.points[edge[0]], part.points[edge[1]], 0.5)
	if not offset:	offset = 0.2 * length(boundingbox(part).width)
	return note_angle(
			(o, cross(z,d0)),
			(o, cross(d1,z)),
			offset, d, tol, text, unit)
	
def note_absciss(axis, pts):
	indev
	
def note_surface(placement, offset=None, roughness=None, method=None):
	indev
	
def note_label(placement, offset=None, text='!', style='rect'):
	''' place a text label upon an object 
	
		`placement` can be any of `Mesh, Web, Wire, axis, vec3`
	'''
	p, normal = mesh_placement(placement)
	if not offset:	
		size = length(boundingbox(placement).width)
		offset = 0.2 * length(boundingbox(placement).width) * normal
	color = settings.display['annotation_color']
	x,_,z = dirbase(normalize(offset))
	sch = Scheme()
	sch.set(color=fvec4(color,0.7))
	sch.add(gt.revolution(
				2*pi,
				(vec3(0),z),
				web([6*x, 10*z]),
				resolution=('div',8),
				),
			space=scale_screen(fvec3(p)), shader='fill')
	sch.add([p, p+offset], space=world, shader='line', layer=2e-4)
	font = int(settings.display['view_font_size'] * 1.2)
	r = font*0.8
	print(font, r)
	if style == 'circle':	outline = web(Circle((vec3(0),vec3(0,0,1)), r))
	elif style == 'rect':	outline = [vec3(r,r,0), vec3(-r,r,0), vec3(-r,-r,0), vec3(r,-r,0), vec3(r,r,0)]
	else:
		raise ValueError("style must be 'rect' or 'circle'")
	sch.set(space=halo_screen(fvec3(p+offset)))
	sch.add(outline, shader='line', layer=0)
	sch.add(gt.flatsurface(wire(outline)), color=fvec4(settings.display['background_color'],0), shader='fill', layer=1e-3)
	sch.add(txt.Text(p+offset, text, align=('center','center'), size=font, color=color), space=world)
	return sch
	

def note_iso(p, offset, type, text, refs=(), label=None):
	indev



#from .scheme import SchemeInstance

#class BaseDisplay(Display):
	#scheme = None
	
	#def __init__(self, scene, matrix, color=None):
		#if not color:	color = settings.display['annotation_color']
		
		#if not self.scheme:
			#type(self).scheme = sch = Scheme(layer=1e-4)
			#space = lambda view, instance: world(view) * instance.matrix
			
			#directions = [vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)]
			#for i in range(len(directions)):
				#o = vec3(0)
				#x,z = directions[i-1], directions[i]
				
				#def tip(view, instance, z=z): 
					#s = space(view, instance)
					#return scale_screen(s*z) * fmat4(fmat3(s))
				
				#sch.add([o, z], 
					#space=tip, 
					#color=fvec4(color,1), 
					#shader='line',
					#)
				#sch.add(revolution(2*pi, (o,z), web([o, -20*z+9*x])), 
					#space=tip, 
					#color=fvec4(color,0.8), 
					#shader='fill',
					#)
		
		#if isinstance(matrix, (dmat3,fmat3,dquat,fquat)):	matrix = fmat4(fmat3(matrix))
		#elif isinstance(matrix, fmat4):						matrix = fmat4(matrix)
		#self.disp = scene.display(SchemeInstance(self.scheme, matrix=matrix))
	
	#def stack(self, scene):
		#yield from self.disp.stack(scene)
		
def note_base(matrix, color=None, labels=('X', 'Y', 'Z'), name=None, size=0.4):
	if not color:	color = settings.display['annotation_color']
	if not name:	name = ''
	if not labels:	labels = [None]*3
	
	if isinstance(matrix, (dmat3,fmat3)):
		center = None
		directions = mat3(matrix)
	elif isinstance(matrix, (dmat4,fmat4)):
		if transpose(matrix)[3] != vec4(0,0,0,1):
			raise TypeError('mat4 must be affine in order to be displayed')
		center = vec3(matrix[3])
		directions = mat3(mat4(matrix))
	else:
		raise TypeError('only matrix with dimension 3 or 4 can be displayed')
	
	sch = Scheme(layer=1e-4)
	for axislabel, direction in zip(labels, directions):
		o = vec3(0)
		x,y,z = dirbase(normalize(direction))
		
		if center:
			sch.add([center], space=world)
		
		sch.add([o, size*direction], 
			space=scale_view(fvec3(center)) if center else ubiquity, 
			color=fvec4(color,1), 
			shader='line',
			)
		sch.add(gt.revolution(2*pi, (o,z), web([o, -14*z+3.6*x])), 
			space=scale_screen_scale_view(fvec3(center), fvec3(size*direction)) if center else scale_screen_ubiquity(fvec3(size*direction)), 
			color=fvec4(color,0.8), 
			shader='fill',
			)
		if axislabel or name:
			sch.add(txt.Text(5*z, axislabel+' '+name, align=(0.5,0.5), color=color))
	
	return sch
	
def note_rotation(quaternion, color=None, size=0.4):
	if not color:	color = settings.display['annotation_color']
	
	direction = vec3(glm.axis(quaternion))
	angle = glm.angle(quaternion)
	o = vec3(0)
	x,y,z = dirbase(direction)
	tend = -sin(angle)*x + cos(angle)*y
	rend = cos(angle)*x + sin(angle)*y
	
	sch = Scheme(layer=1e-4)
	sch.set(
		space=ubiquity,
		color=fvec4(color,0.6),
		shader='line',
		)
	sch.add([-size*direction, -0.2*size*direction])
	sch.add([-0.1*size*direction, 0.1*size*direction])
	sch.add([0.2*size*direction, size*direction])
	sch.add(gt.revolution(2*pi, (o,z), web([o, -14*z+3.6*x])), 
		space=scale_screen_ubiquity(fvec3(size*direction)), 
		color=fvec4(color,0.6), 
		shader='fill',
		)
	
	sch.set(
		space=ubiquity,
		color=fvec4(color,1),
		shader='line',
		)
	sch.add([size*cos(t)*x + size*sin(t)*y  for t in linrange(0, angle, step=0.1)])
	sch.add([size*cos(t)*x + size*sin(t)*y  for t in linrange(0, 2*pi, step=0.1)], color=fvec4(color,0.2))
	sch.add(gt.revolution(2*pi, (size*rend, tend), web([size*tend, size*tend-14*tend+3.6*rend])), 
		space=scale_screen_ubiquity(fvec3(size*rend)), 
		color=fvec4(color,0.8), 
		shader='fill',
		)
		
	return sch
	
'''
transforms = [
	mat3(),
	angleAxis(pi/2, normalize(Z+X)),
	translate(2*Y) * scale(vec3(0.2, 0.3, 1)),
	translate(2*X),
	rotate(1, vec3(1,2,3)),
	translate(-2*X) * mat4(scaledir(vec3(1,2,-3)*0.1)),
	]
'''
	
	
def scale_screen_scale_view(center, offset):
	def mat(view):
		m = view.uniforms['view'] * view.uniforms['world']
		e = view.uniforms['proj'] * fvec4(1,1,(m*center).z,1)
		e /= e[3]
		return m * translate(center) * scale(fvec3(1/e[1])) * translate(offset) * scale(fvec3(2/view.target.height))
	return mat
	
def scale_screen_ubiquity(offset):
	def mat(view):
		proj = view.uniforms['proj']
		f = - (proj[3][2]-proj[3][3]) / (proj[2][2] - proj[2][3])
		n = - (proj[3][2]+proj[3][3]) / (proj[2][2] + proj[2][3])
		d = 3*n
		e = proj * fvec4(1,1,d,1)
		e /= e[3]
		d2 = 2/view.target.height
		return translate(fvec3(0,0,d)) * fmat4(
					fmat3(1/e[1]) 
					* fmat3(view.uniforms['view']) 
					* fmat3(view.uniforms['world'])) * translate(offset) * scale(fvec3(d2))
	return mat
	
def base_display(scene, mat):
	return scene.display(note_base(mat))
	#try:	return scene.display(note_base(mat))
	#except TypeError as err:	return Display()

def quat_display(scene, quat):
	try:	return scene.display(note_rotation(quat))
	except TypeError:	return Display()
		
overrides.update({
	dmat4: base_display,
	fmat4: base_display,
	dmat3: base_display,
	fmat3: base_display,
	dquat: quat_display,
	fquat: quat_display,
	})
