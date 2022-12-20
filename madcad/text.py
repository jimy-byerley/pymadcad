# This file is part of pymadcad,  distributed under license LGPL v3
'''	
	This file is to render floating texts and mesh texts
	
	The current implementation of floating texts uses a bitmap font texture baked at program start.
	It's following the first technique described in this tutorial: https://learnopengl.com/In-Practice/Text-Rendering
		
	For mesh texts, freetype (https://freetype.org/) is used to read the font files and extract curve primitives from it. 
	The primitives are then discretized and triangulated using the madcad functions.
	
	When specified by their name, fonts are searched in the system font directories plus the directories in module variable `fontpath`
'''

import os
import sys
from dataclasses import dataclass

from .common import ressourcedir
from .mathutils import (Box, vec3, fvec3, fvec4, vec2, fvec2, fmat4, 
						ceil, sqrt, distance, anglebt, linrange,
						)
from .triangulation import triangulation
from .primitives import Segment
from .mesh import Wire, Web, Mesh, web
from . import settings, rendering

from PIL import Image, ImageFont, ImageDraw
import numpy.core as np
import moderngl as mgl



# TODO: utiliser les methodes et attributs ImgeFont.size, .getmetrics(), etc pour avoir les hauteur et largeur de police

fontpath = [ressourcedir]

def font_locations():
	''' yield system and user font directories 
		it looks at module variable `fontpath` and then at system font paths
		
		The function is a generator so if iteration is stopped in its beginning, no more than necessary system call is performed
	'''
	yield from fontpath
	
	home = os.getenv('HOME')
	
	if sys.platform == 'linux':
		yield from exploredir(os.path.join(home, '.fonts'))
		yield from exploredir('/usr/local/share/fonts')
		yield from exploredir('/usr/share/fonts')
		yield '/usr/share/fonts'
	
	elif sys.platform == 'win32':
		yield os.path.join(home, r'AppData\Local\Microsoft\Windows\Fonts')
		yield os.path.join(os.environ['WINDIR'],'fonts')
	
	elif sys.platform == 'darwin':
		yield os.path.join(home, 'Library/Fonts')
		yield '/Library/Fonts'
		yield '/System/Library/Fonts'

def exploredir(dir):
	''' list directory elements as string file paths '''
	for sub in os.listdir(dir):
		yield os.path.join(dir, sub)
	
def font_list():
	''' yield all font files found '''
	for location in font_locations():
		yield from os.path.listdir(location)

def font_path(name):
	''' find the font file path for the given font name 
		if the given name is already a path to an existing font file, return it immediately
	'''
	if os.path.exists(name):
		return name
	
	filename = name+'.ttf'
	for location in font_locations():
		path = os.path.join(location, filename)
		if os.path.exists(path):	return path
	raise FileNotFoundError('unable to find font {}'.format(repr(name)))

def create_font_texture(font: ImageFont, maxchar=1100) -> '(Image, (c,l))':
	''' create a font texture by rasterizing a given font '''
	# determine the size of the needed texture
	fontsize = font.size + 4
	width = 64
	while 2*(width//fontsize)**2 < maxchar:
		width *= 2
	c = width//(fontsize//2)
	l = maxchar//c +1
	width = c * (fontsize//2)
	height = l * fontsize
	# create texture
	tex = Image.new('L', (width,height), 0)
	draw = ImageDraw.Draw(tex)
	# draw the font onto
	for i in range(maxchar):
		draw.text(char_placement(font.size, c, l, i), chr(i), fill=255, font=font)
	#tex.show()
	
	return tex, (c, l)

def char_placement(fontsize, c, l, n):
	fontsize += 4
	return ((fontsize//2) * (n%c), fontsize * (n//c))


class TextDisplay(rendering.Display):
	''' halo display of a monospaced text 
	
		This class is usually used through `scheme.note_floating()`
	'''
	pointsdef = [
		[0, 0],
		[0,-1],
		[1,-1],
		[0, 0],
		[1,-1],
		[1, 0],
		]
	
	def __init__(self, scene, position, text, size=None, color=None, font=None, align=(0,0), layer=0):
		if not color:	color = settings.display['annotation_color']
		if not size:	size = settings.display['view_font_size']
		self.position = fvec3(position)
		self.color = fvec3(color)
		self.size = size
		self.layer = layer
		
		# load font
		fontfile = font_path(font or 'NotoMono-Regular')
		def load(scene):
			img, align = create_font_texture(ImageFont.truetype(fontfile, 2*size))
			return scene.ctx.texture(img.size, 1, img.tobytes()), align
		self.fonttex, self.fontalign = scene.ressource(('fontfile', size), load)
		
		# load shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open(ressourcedir+'/shaders/font.vert').read(),
						fragment_shader=open(ressourcedir+'/shaders/font.frag').read(),
						)
			shader['fonttex'].value = 0
			return shader
		self.shader = scene.ressource('shader_font', load)
		
		# place triangles
		points = np.zeros((len(self.pointsdef)*len(text), 4), 'f4')
		l = 0
		c = 0
		for i,char in enumerate(text):
			if char == '\n':
				c = 0
				l += 1
			elif char == '\t':
				c += 4 - c%4	# TODO: use a tab size in settings
			elif char == ' ':
				c += 1
			else:
				n = ord(char)
				#placement = char_placement(2*size, *self.fontalign, n)
				for j,add in enumerate(self.pointsdef):
					points[len(self.pointsdef)*i + j] = [
						add[0]+c, 
						add[1]-l, 
						(n % self.fontalign[0] +add[0]) / self.fontalign[0], 
						(n //self.fontalign[0] -add[1]) / self.fontalign[1],
						]
				c += 1
		l += 1		
		align = (processalign(align[0], c), -processalign(align[1], l))
		points -= [*align, 0, 0]
		self.textsize = (c,l)
		self.visualsize = (-align[0], -align[1], c//2-align[0], l-align[1])
			
		
		# create the buffers on the GPU
		self.vb_points = scene.ctx.buffer(points)
		self.vb_faces = scene.ctx.buffer(np.arange(points.shape[0], dtype='u4'))
		self.va = scene.ctx.vertex_array(
			self.shader,
			[(self.vb_points, '2f 2f', 'v_position', 'v_uv')],
			)
	
	def render(self, view):		
		self.shader['layer'] = self.layer
		self.shader['color'].write(self.color)
		self.shader['position'].write(fvec3(self.world * fvec4(self.position,1)))
		self.shader['view'].write(view.uniforms['view'])
		self.shader['proj'].write(view.uniforms['proj'])
		self.shader['ratio'].value = (
				(self.size-2.5) / view.width()*2,
				(self.size-2.5) / view.height()*4,
				)
		self.fonttex.use(0)
		self.va.render(mgl.TRIANGLES)

	def stack(self, view):
		return ((), 'screen', 2, self.render),

def processalign(align, size):
	''' return the concrete alignment, expanding alignments such as 'left', or 'center' to numeric values '''
	if isinstance(align, str):
		if align in ('left', 'top'):		return 0
		elif align in ('right', 'bottom'):	return size
		elif align == 'center':	return size/2
		else:
			raise ValueError("align must be int, float or any of 'left', 'right', 'center'")
	else:
		return align

def textsize(text, tab=4):
	''' visual size of a text as vec2(column,line) '''
	l = 0
	c = 0
	for i,char in enumerate(text):
		if char == '\n':
			c = 0
			l += 1
		elif char == '\t':
			c += tab - c%tab
		else:
			c += 1
	l += 1
	return fvec2(c,l)




BezierLinear = Segment

@dataclass
class BezierQuadratic:
	''' primitive representing a quadratic Bezier spline '''
	a: vec3
	b: vec3
	c: vec3
	
	def __call__(self, t):
		u = 1-t
		return u**2*self.a + 2*u*t*self.b + t**2*self.c
		
	def mesh(self, resolution=None):
		div = settings.curve_resolution(
			distance(self.a, self.b) + distance(self.b, self.c),
			anglebt(self.b-self.a, self.c-self.b),
			resolution)
		return Wire(self(t)  for t in linrange(0, 1, div=div))
		
	def display(self, scene):
		return self.mesh().display(scene)
	
@dataclass
class BezierCubic:
	''' primitive representing a cubic Bezier spline '''
	a: vec3
	b: vec3
	c: vec3
	d: vec3
	
	def __call__(self, t):
		u = 1-t
		return u**3*self.a + 3*u**2*t*self.b + 3*u*t**2*self.c + t**3*self.d
		
	def mesh(self, resolution=None):
		div = settings.curve_resolution(
			distance(self.a, self.b) + distance(self.b, self.c) + distance(self.c, self.d),
			anglebt(self.b-self.a, self.c-self.b) + angletbt(self.c-self.b, self.d-self.c),
			resolution)
		return Wire(self(t)  for t in linrange(0, 1, div=div))
		
	def display(self, scene):
		return self.mesh().display(scene)

''' font caching dictionnary 
	for each font in the cache, an item `'file.ttf': dict` is created
	each key in this dictionnary is a string character, giving a dictionnary of already computed stuff about the matchng glyph in the font file
	
	This is an example of its content:
	
		cache_fonts = {
			'monospace.ttf': {
				'cbox': Box(...),
				'web': Web(...),
				'mesh': Mesh(...),
				}
			}
'''
cache_fonts = {}

try:
	import freetype
except ImportError:
	pass
else:
	def character_primitives(face: freetype.Face) -> list:
		''' return a list of primitives describing the outline of the character currently loaded in the given face '''
		primitives = []
		last = [None]

		def move_to(a, ctx):
			last[0] = ft2vec(a)
		def line_to(b, ctx):
			primitives.append(BezierLinear(last[0], ft2vec(b)))
			last[0] = ft2vec(b)
		def conic_to(b, c, ctx):
			primitives.append(BezierQuadratic(last[0], ft2vec(b), ft2vec(c)))
			last[0] = ft2vec(c)
		def cubic_to(b, c, d, ctx):
			primitives.append(BezierCubic(last[0], ft2vec(b), ft2vec(c), ft2vec(d)))
			last[0] = ft2vec(d)
			# in fact there will be no cubic_to because there is no such things in freetype fonts, but just in case for freetype-py
		
		face.glyph.outline.decompose(move_to=move_to, line_to=line_to, conic_to=conic_to, cubic_to=cubic_to)
		return primitives

	def ft2vec(ft):   return vec3(ft.x, ft.y, 0)

	def character_cbox(face: freetype.Face) -> Box:
		''' return the cbox of the current glyph loaded in the face '''
		box = face.glyph.outline.get_cbox()
		return Box(
				min=vec3(box.xMin, box.yMin, 0), 
				max=vec3(box.xMax, box.yMax, 0))
		
	def character_outline(char: str, font: str, resolution=None) -> dict:
		''' return a discretized character outline, cached '''
		try:	
			return cache_fonts[font][char]
		except (KeyError, AttributeError):
			if font not in cache_fonts:			cache_fonts[font] = {}
			if char not in cache_fonts[font]:	cache_fonts[font][char] = {}
			cache = cache_fonts[font][char]
			
			if isinstance(font, str):
				face = freetype.Face(font_path(font))
			else:
				face = font
			scale = 1024
			face.set_char_size(scale)
			face.load_char(char)
			cache['fixed'] = face.is_fixed_width
			cache['cbox'] = character_cbox(face) .transform(1/scale)
			cache['web'] = web(character_primitives(face), resolution=resolution) .transform(1/scale) .mergegroups()
			return cache

	def character_surface(char, font, resolution=None) -> dict:
		''' return a triangulated character, cached '''
		try:	
			return cache_fonts[font][char]
		except (KeyError, AttributeError):
			cache = character_outline(char, font, resolution)
			cache['mesh'] = triangulation(cache['web'])
			return cache

	def text(text: str, font:str=None, size:float=1, spacing=vec2(0.05, 0.2), fill=True, align=(0,0), resolution=None):
		''' return a Mesh/Web containing the given text written using the given font
		
			The meshed font is cached so long texts are still fast to mesh
			
			Parameters:
				text:   a multiline string to represent
				font:   the string name of a font such as `'NotoMono-Bold'` or the path to a `.ttf` font file
				size:   the character size (metric unit)
				spacing:  spacing ratio between characters `vec2(horizontal, vertical)`
				fill:   if True, the characters are triangulated and the function returns a `Mesh`, else it returns its outline as a `Web`
				
				align:  
				
					text alignment, the tuple items can be:
					
					- 'left' or 'top'
					- 'center'
					- 'right' or 'bottom'
					- any float, as an offset on the position
				
				resolution:  discretisation setting for the character primitives
				
			Example:
				
				>>> part = text('Hello everyone.\\nthis is a great font !!', 
				...             font='NotoSans-Regular', 
				...             align=('left', 0),
				...             fill=True)
			
		'''
		if fill:	
			pool = Mesh()
			character = character_surface
		else:
			pool = Web()
			character = character_outline
		
		face = freetype.Face(font_path(font or 'NotoMono-Regular'))
		position = vec3(0)
		for char in text:
			if char == ' ':
				position.x += 0.3
			elif char == '\t':
				width = 0.3
				position.x += int(position.x/width) * width
			elif char == '\n':
				position.y -= (1+spacing.y)
				position.x = 0
			else:
				cache = character(char, face, resolution)
				if fill:	part = cache['mesh']
				else:		part = cache['web']
				pool += part.transform(position-vec3(cache['cbox'].min.x,0,0))
				if cache['fixed']:
					position.x += 0.5 + spacing.x
				else:
					position.x += cache['cbox'].width.x + spacing.x
		
		if size != 1:	
			pool = pool.transform(size)
		if align != (0,0):	
			width = pool.box().width
			pool = pool.transform(vec3(
					processalign(align[0], -width[0]),
					processalign(align[1], -width[1]),
					0))
		return pool




def test_character_primitives():
	from .rendering import show
	
	face = freetype.Face('cad/pymadcad/madcad/NotoMono-Regular.ttf')
	face.set_char_size(1024)
	face.load_char('&')
	show([ triangulation(character_primitives(face)) ])

def test_character_cached():
	from .rendering import show
	show([ 
		character_outline('&', 'NotoMono-Regular').web,
		character_surface('g', 'NotoMono-Regular').mesh.transform(vec3(1,0,0)),
		])

def test_text():
	from .rendering import show
	from .generation import extrusion
	from .scheme import note_distance
	settings.display['view_font_size'] = 10
	part = text('Hello everyone.\nthis is a great font !!', font='NotoSans-Regular', align=('center', 0))
	part = extrusion(vec3(0,0,-1), part.flip())
	show([
			vec3(0),
			note_distance(vec3(0), vec3(0,1,0), offset=vec3(-0.5,0,0), text='size'),
			part,
			], 
		display_wire=True,
		)
