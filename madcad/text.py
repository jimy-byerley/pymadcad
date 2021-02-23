# This file is part of pymadcad,  distributed under license LGPL v3
'''	
	This file is to write text into the 3D view
	
	The current implementation uses a bitmap font texture baked at program start.
	It's following the first technique described here:
		https://learnopengl.com/In-Practice/Text-Rendering
'''

from PIL import Image, ImageFont, ImageDraw
import numpy.core as np
import moderngl as mgl
from .common import ressourcedir
from .mathutils import fvec3, fvec4, fmat4, ceil, sqrt
from . import settings, rendering

# TODO: utiliser les methodes et attributs ImgeFont.size, .getmetrics(), etc pour avoir les hauteur et largeur de police

def create_font_texture(font, maxchar=1100):
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

class Text:
	def __init__(self, position, text, size=None, color=(1,1,1), align=(0,0)):
		self.text = text
		self.position = fvec3(position)
		self.size = size or settings.display['view_font_size']
		self.color = color
		self.align = align
	
	def display(self, scene):
		return TextDisplay(scene, self.position, self.text, self.size, self.color, self.align)

def char_placement(fontsize, c, l, n):
	fontsize += 4
	return ((fontsize//2) * (n%c), fontsize * (n//c))

pointsdef = [
	[0, 0],
	[0,-1],
	[1,-1],
	[0, 0],
	[1,-1],
	[1, 0],
	]

class TextDisplay(rendering.Display):
	
	def __init__(self, scene, position, text, size=None, color=None, align=(0,0), layer=0):
		if not color:	color = settings.display['annotation_color']
		if not size:	size = settings.display['view_font_size']
		self.position = fvec3(position)
		self.color = fvec3(color)
		self.size = size
		self.layer = layer
		
		# load font
		def load(scene):
			img, align = create_font_texture(ImageFont.truetype(ressourcedir+'/NotoMono-Regular.ttf', 2*size))
			return scene.ctx.texture(img.size, 1, img.tobytes()), align
		self.fonttex, self.fontalign = scene.ressource(('fonttex', size), load)
		
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
		points = np.zeros((len(pointsdef)*len(text), 4), 'f4')
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
				for j,add in enumerate(pointsdef):
					points[len(pointsdef)*i + j] = [
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
	if isinstance(align, str):
		if align == 'left':		return 0
		elif align == 'right':	return size
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
	return vec2(c,l)

#class Particles(rendering.Display):
	#def __init__(self, texture, format, vertices):
		#position, tile, size
