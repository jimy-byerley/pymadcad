import moderngl as mgl
import numpy.core as np

from .mathutils import *
from .rendering import Display
from .common import ressourcedir
from .mesh import Container, Mesh, Web, Wire, web, wire, mesh_distance
from .rendering import Displayable, writeproperty
from .primitives import *
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
			'note_angle_edge']


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
			self.spaces = np.empty((self.max_spaces, 4,4), 'f4')
			self.spacegens = list(sch.spaces)
			if len(self.spacegens) > self.max_spaces:		
				print('warning: the number of local spaces exceeds the arbitrary build-in limit of {}'.format(self.max_spaces))
			
			self.components = [(space,scene.display(obj))	for space,obj in sch.components]
			
			
			self.nidents = max(v[5] for v in sch.vertices)+1
			self.box = boundingbox(fvec3(v[1]) for v in sch.vertices if self.spacegens[v[0]] is world)
			
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
				if prim == mgl.LINES:			ident_triangles.extend(batch)
				elif prim == mgl.TRIANGLES:		ident_lines.extend(batch)
			
			if ident_triangles:	self.vai_triangles	= ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_triangles, 'u4')), skip_errors=True)
			if ident_lines:		self.vai_lines 		= ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_lines, 'u4')), skip_errors=True)
			
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
			if self.annotation and not scene.options['display_annotations']:
				return
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
					web([vec3(0), -8*z+2*x]), 
					resolution=('div',8),
					), 
				shader='fill',
				space=scale_screen(self.origin),
				)
			sch.set(space=halo_screen(self.origin+self.offset))
			sch.add([vec3(0), vec3(side*(20 - 9*0.4), 0, 0)], shader='line')
			sch.add(txt.Text(vec3(side*20, 0, 0), self.comment, align=('right' if side>0 else 'left', 0.5), color=fvec4(color,1), size=9))
			
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
		if self.annotation and not scene.options['display_annotations']:
			return ()
		return ((), 'screen', 2, self.render), ((), 'ident', 2, self.identify)
		
	@writeproperty
	def world(self, value):
		for disp in self.disp:	disp.world = value


def mesh_placement(mesh) -> '(pos,normal)':
	''' return an axis for placement of a note on the given object '''
	if isinstance(mesh, Mesh):
		# group center normal
		center = mesh.barycenter()
		f = min(mesh.faces,
				key=lambda f:  dot(center - sum(mesh.facepoints(f))/3, mesh.facenormal(f))
				)
		normal = mesh.facenormal(f)
		pos = sum(mesh.facepoints(f)) / 3
	
	elif isinstance(mesh, Web):
		center = mesh.barycenter()
		e = min(mesh.edges,
				key=lambda e:  length2(center - (mesh.points[e[0]]+mesh.points[e[1]])/2)
				)
		normal = dirbase(normalize(mesh.points[e[0]]-mesh.points[e[1]]))[0]
		pos = mix(mesh.points[e[0]], mesh.points[e[1]], 0.5)
		
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
	return txt.Text(position, text, align=(0,0), color=settings.display['annotation_color'], size=9)

def note_distance(a, b, offset=0, project=None, d=None, tol=None, text=None):
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
	shift = 0.5 * dot(b-a, x)
	ao = a + (offset + shift) * x
	bo = b + (offset - shift) * x
	# create scheme
	sch = Scheme()
	sch.set(shader='line', layer=1e-2, color=fvec4(color,0.3))
	sch.add([a, ao])
	sch.add([b, bo])
	sch.set(layer=-1e-2, color=fvec4(color,0.7))
	sch.add([ao, bo])
	sch.add(txt.Text(
				mix(ao,bo,0.5), 
				text, 
				align=('center', 0.5), 
				size=9, 
				color=fvec4(color,1)))
	sch.set(shader='fill')
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),project), 
				web([vec3(0), 1.5*x-6*project]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(ao)))
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),project), 
				web([vec3(0), 1.5*x+6*project]), 
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
		

def note_angle(a0, a1, offset=0, d=None, tol=None, text=None, unit='deg'):
	''' place an angle quotation between 2 axis '''
	o0, d0 = a0
	o1, d1 = a1
	z = normalize(cross(d0,d1))
	if not isfinite(z):	
		raise ValueError('axis are parallel')
	x0 = cross(d0,z)
	x1 = cross(d1,z)
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
	sch.set(shader='line', layer=1e-2, color=fvec4(color,0.3))
	sch.add([p0, o0])
	sch.add([p1, o1])
	arc = ArcCentered((center,z), p0, p1, ('rad',0.05)).mesh()
	sch.add(arc, color=fvec4(color,0.7))
	sch.set(layer=-1e-2)
	sch.add(txt.Text(
				arc[len(arc)//2], 
				text, 
				align=('center', 0.5), 
				size=9, 
				color=fvec4(color,1)))
	sch.set(shader='fill')
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),x0), 
				web([vec3(0), 1.5*d0+6*x0]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(p0)))
	sch.add(gt.revolution(
				2*pi, 
				(vec3(0),x1), 
				web([vec3(0), 1.5*d1-6*x1]), 
				resolution=('div',8)), 
			space=scale_screen(fvec3(p1)))
	return sch
	
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
				web([3*x, -5*z]),
				resolution=('div',8),
				),
			space=scale_screen(fvec3(p)), shader='fill')
	sch.add([p, p+offset], space=world, shader='line', layer=2e-3)
	r = 5
	if style == 'circle':	outline = web(Circle((vec3(0),vec3(0,0,1)), r))
	elif style == 'rect':	outline = [vec3(r,r,0), vec3(-r,r,0), vec3(-r,-r,0), vec3(r,-r,0), vec3(r,r,0)]
	else:
		raise ValueError("style must be 'rect' or 'circle'")
	sch.set(space=halo_screen(fvec3(p+offset)))
	sch.add(outline, shader='line', layer=0)
	sch.add(gt.flatsurface(wire(outline)), color=fvec4(settings.display['background_color'],0), shader='fill', layer=1e-3)
	sch.add(txt.Text(p+offset, text, align=('center','center'), size=10, color=color), space=world)
	return sch
	

def note_iso(p, offset, type, text, refs=(), label=None):
	indev
