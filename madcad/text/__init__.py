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

from ..box import Box
from ..common import resourcedir
from ..mathutils import anglebt, distance, linrange, vec2, vec3, fvec2
from ..triangulation import triangulation
from ..primitives import Segment
from ..mesh import Wire, Web, Mesh, web
from .. import settings

__all__ = ['font_locations', 'font_list', 'font_path', 'exploredir', 'text', 'textsize', 'processalign']


fontpath = [resourcedir]
''' list of paths to directories containing fonts '''

cache_fonts = {}
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
	if os.path.exists(dir):
		if os.path.isdir(dir):
			yield dir
			for sub in os.listdir(dir):
				yield from exploredir(os.path.join(dir, sub))
	
def font_list():
	''' yield all font files found '''
	for location in font_locations():
		try:
			for name in os.listdir(location):
				if name.endswith('.ttf'):
					yield name
		except (NotADirectoryError, FileNotFoundError):
			pass

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
	mc = 0
	for i,char in enumerate(text):
		if char == '\n':
			c = 0
			l += 1
		elif char == '\t':
			c += tab - c%tab
		else:
			c += 1
		if c > mc:
			mc = c
	l += 1
	return fvec2(mc, l)




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
			anglebt(self.b-self.a, self.c-self.b) + anglebt(self.c-self.b, self.d-self.c),
			resolution)
		return Wire(self(t)  for t in linrange(0, 1, div=div))
		
	def display(self, scene):
		return self.mesh().display(scene)


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
					position.x += cache['cbox'].size.x + spacing.x
		
		if size != 1:	
			pool = pool.transform(size)
		if align != (0,0):	
			width = pool.box().size
			pool = pool.transform(vec3(
					processalign(align[0], -width[0]),
					processalign(align[1], -width[1]),
					0))
		return pool



