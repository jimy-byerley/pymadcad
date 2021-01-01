import moderngl as mgl
import numpy.core as np
from collections import namedtuple

from .mathutils import *
from .rendering import Display
from .common import ressourcedir
from .mesh import Container, Mesh, Web, Wire, web
from .rendering import Displayable
from . import generation as gt
from . import text
from . import settings

SVertex = namedtuple('SVertex', ['space', 'pos', 'normal', 'color', 'layer', 'track', 'flags'])

class Scheme:
	def __init__(self, vertices=None, spaces=None, primitives=None, annotation=True, **kwargs):
		self.vertices = vertices or [] # list of vertices
		self.spaces = spaces or []	# definition of each space
		self.primitives = primitives or {} # list of indices for each shader
		self.components = []	# displayables associated to spaces
		self.annotation = annotation	# flag saying if this object is an annotation
		# for creation: last vertex inserted
		self.current = {'color':fvec4(settings.display['annotation_color'],1), 'flags':0, 'layer':0, 'space':world, 'shader':'wire', 'track':0, 'normal':fvec3(0)}
		self.set(**kwargs)
		
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
	
	def component(self, obj, **kwargs):
		''' add an object as component associated to the current space '''
		self.set(**kwargs)
		self.components.append((self.current['space'], obj))
	
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
			self.spacegens = list(sch.spaces)
			if len(self.spacegens) > self.max_spaces:		
				print('warning: the number of local spaces exceeds the arbitrary build-in limit of {}'.format(self.max_spaces))
			self.spaces = np.empty((self.max_spaces, 4,4), 'f4')
			
			self.components = [(space,scene.display(obj))	for space,obj in sch.components]
			
			# prepare the buffer of vertices
			vertices = np.empty(len(sch.vertices), 'u1, 3f4, 3f4, 4u1, f4, u2, u1')
			for i,v in enumerate(sch.vertices):
				vertices[i] = (
					*v[:3],
					u8vec4(v[3]*255), 
					*v[4:]
					)
			self.nidents = max(v[5] for v in sch.vertices)+1
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
				if prim == mgl.LINES:			ident_triangles.extend(batch)
				elif prim == mgl.TRIANGLES:		ident_lines.extend(batch)
			
			if ident_triangles:	self.vai_triangles	= ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_triangles, 'u4')), skip_errors=True)
			if ident_lines:		self.vai_lines 		= ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_lines, 'u4')), skip_errors=True)
			
		def load(self, scene):
			''' load shaders and all static data for the current opengl context '''
			shader_ident = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/scheme.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/scheme-ident.frag').read(),
						)
			shaders = {
				'line': (mgl.LINES, scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/scheme.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/scheme-uniform.frag').read(),
						)),
				'fill': (mgl.TRIANGLES, scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/scheme.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/scheme-uniform.frag').read(),
						)),
				#'ghost': (glm.TRIANGLES, scene.ctx.program(
						#vertex_shader=open(ressourcedir+'/shaders/scheme.vert').read(),
						#fragment_shader=open(ressourcedir+'/shaders/scheme_transp.frag').read(),
						#)),
				}
			return shaders, shader_ident
			
		def compute_spaces(self, view):
			''' computes the new spaces for this frame
				this is meant to be overriden when new spaces are required 
			'''
			view.uniforms['world'] = self.world
			for i,gen in enumerate(self.spacegens):
				self.spaces[i] = gen(view)
			invview = affineInverse(view.uniforms['view'])
			for space,disp in self.components:
				disp.world = invview * fmat4(self.spaces[space])
		
		def render(self, view):
			''' render each va in self.vas '''
			self.compute_spaces(view)
			for name in self.vas:
				shader = self.shaders[name][1]
				prim, va = self.vas[name]
				shader['spaces'].write(self.spaces)
				shader['proj'].write(view.uniforms['proj'])
				va.render(prim)
		
		def identify(self, view):
			''' render all the triangles and lines for identification '''
			self.shader_ident['startident'] = view.identstep(self.nidents)
			self.shader_ident['spaces'].write(self.spaces)
			self.shader_ident['proj'].write(view.uniforms['proj'])
			
			if self.vai_lines:		self.vai_lines.render(mgl.LINES)
			if self.vai_triangles:	self.vai_triangles.render(mgl.TRIANGLES)
		
		def stack(self, scene):
			yield ((), 'screen', 2, self.render) 
			yield ((), 'ident', 2, self.identify)
			for space,disp in self.components:
				yield from disp.stack(scene)

# create standard spaces

def view(view):
	proj = view.uniforms['proj']
	return fmat4(1/proj[0][0],  0,0,0,
				0, 1/proj[1][1], 0,0,
				0,0,1,0,
				0,0,0,1)

def screen(view):
	return fmat4(view.width()/2,0,0,0,
				0,view.height()/2,0,0,
				0,0,1,0,
				0,0,0,1)

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
		m = fmat4(1)
		m[3] = center
		m[0][0] = center.z/proj[0][0]
		m[1][1] = center.z/proj[1][1]
		return m
	return mat
def halo_screen(position):
	position = fvec4(position,1)
	def mat(view):
		center = view.uniforms['view'] * (view.uniforms['world'] * position)
		m = fmat4(1)
		m[3] = center
		d = center.z/view.height()
		m[0][0] = d
		m[1][1] = d
		return m
	return mat

def scale_screen(center):
	def mat(view):
		m = view.uniforms['view'] * view.uniforms['world']
		d = (m*fvec4(center,1)).z /view.height()
		return scale(translate(m, center), fvec3(d))
	return mat

def scale_view(center):
	def mat(view):
		m = view.uniforms['view'] * view.uniforms['world']
		d = (m*fvec4(center,1)).z
		return scale(translate(m, -center), fvec3(d))
	return mat



class Annotation:
	def __init__(self, *args, **kwargs):
		for i,(k,v) in enumerate(self.defaults):
			if i < len(args):	setattr(self, k, args[i])
			elif k in kwargs:	setattr(self, k, kwargs[k])
			else:				setattr(self, k, v)


class note_leading_display(Display):
	def __init__(self, scene, origin, offset, comment):
		self.origin = fvec3(origin)
		self.offset = fvec3(offset)
		self.comment = comment
		
		def build(side):
			color = fvec4(settings.display['annotation_color'],0.7)
			sch = Scheme(color=color)
			sch.add([self.origin, self.origin+self.offset], shader='line', space=world)
			x,y,z = dirbase(normalize(vec3(self.offset)))
			sch.add(
				gt.revolution(2*pi, (vec3(0), z), 
					web([vec3(0), -8*z+2*x]), 
					resolution=('div',8),
					), 
				shader='fill',
				space=scale_screen(self.origin),
				)
			sch.set(space=halo_screen(self.origin+self.offset))
			sch.add([vec3(0), vec3(side*(20 - 9*0.4), 0, 0)], shader='line')
			sch.add(text.Text(vec3(side*20, 0, 0), self.comment, align=('right' if side>0 else 'left', 0.5), color=color, size=9))
			
			return scene.display(sch)
		self.disp = build(1), build(-1)
		
	def side(self, view):
		return int((fmat3(view.uniforms['view']) * (fmat3(self.world) * self.offset))[0] > 0)
	
	def render(self, view):
		disp = self.disp[self.side(view)]
		disp.render(view)
		for _,comp in disp.components:
			comp.render(view)		
	
	def identify(self, view):
		self.disp[self.side(view)].identify(view)
	
	def stack(self, scene):
		return ((), 'screen', 2, self.render), ((), 'ident', 2, self.identify)


def note_leading(origin, direction, comment):
	return Displayable(note_leading_display, origin, direction, comment)

def note_group(part, group, comment):
	# group center
	center = sum(	sum(part.facepoints(f))
					for f,t in zip(part.faces, part.tracks)
					if t == group
					) / (3*len(part.faces))
	# group center normal
	f = min(part.faces,
			key=lambda f:  length2(center - sum(part.facepoints(f))/3)
			)
	size = length(boundingbox(part).width)
	return note_leading(sum(part.facepoints(f))/3, 0.3*size*part.facenormal(f), comment)

def note_floating(position, comment, *args, **kwargs):
	return text.Text(position, comment, *args, **kwargs, color=settings.display['annotation_color'], size=9)

def note_distance(a, b, offset, d=None, tol=None):
	if not d:	d = distance(a,b)
	a = fvec3(a)
	b = fvec3(b)
	offset = fvec3(offset)
	ao = a+offset
	bo = b+offset
	sch = Scheme()
	sch.set(shader='line', layer=-1e-4, color=fvec4(settings.display['annotation_color'],0.3))
	sch.add([a, ao])
	sch.add([b, bo])
	sch.set(layer=1e-4, color=fvec4(settings.display['annotation_color'],0.7))
	sch.add([ao, bo])
	sch.add(text.Text(mix(ao,bo,0.5), '{:.6g}  ± {}'.format(d, tol) if tol else '{:.6g}'.format(d), align=('center','center'), size=9, color=fvec4(settings.display['annotation_color'],1)))
	sch.set(shader='fill')
	n,_,x = dirbase(normalize(vec3(a-b)))
	sch.add(gt.revolution(2*pi, (vec3(0),x), web([vec3(0), 1.5*n+6*x]), resolution=('div',8)), space=scale_screen(ao))
	sch.add(gt.revolution(2*pi, (vec3(0),x), web([vec3(0), 1.5*n-6*x]), resolution=('div',8)), space=scale_screen(bo))
	return sch
