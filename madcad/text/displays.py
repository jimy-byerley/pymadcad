# This file is part of pymadcad,  distributed under license LGPL v3
from PIL import Image, ImageFont, ImageDraw
import numpy._core as np
import moderngl as mgl

from ..rendering import Display, highlight_color
from ..common import resourcedir
from ..mathutils import fvec3, fvec4
from .. import settings
from . import font_path, processalign


def create_font_texture(font: ImageFont, maxchar=1100) -> '(Image, (c,l))':
	''' create a font texture by rasterizing a given font '''
    # TODO: utiliser les methodes et attributs ImgeFont.size, .getmetrics(), etc pour avoir les hauteur et largeur de police
	
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


class TextDisplay(Display):
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
		self.selected = False
		self.hovered = False
		
		# load font
		fontfile = font_path(font or 'NotoMono-Regular')
		def load(scene):
			img, align = create_font_texture(ImageFont.truetype(fontfile, 2*size))
			return scene.context.texture(img.size, 1, img.tobytes()), align
		self.fonttex, self.fontalign = scene.share(('fontfile', size), load)
		
		# load shader
		def load(scene):
			shader = scene.context.program(
						vertex_shader=open(resourcedir+'/shaders/font.vert').read(),
						fragment_shader=open(resourcedir+'/shaders/font.frag').read(),
						)
			shader['fonttex'].value = 0
			return shader
		self.shader = scene.share('shader_font', load)
		
		# place triangles
		points = np.zeros((len(self.pointsdef)*len(text), 4), 'f4')
		l = 0
		c = 0
		mc = 0
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
			if c > mc:
				mc = c
		l += 1		
		align = (processalign(align[0], mc), -processalign(align[1], l))
		points -= [*align, 0, 0]
		self.textsize = (mc,l)
		self.visualsize = (-align[0], -align[1], mc//2-align[0], l-align[1])
			
		
		# create the buffers on the GPU
		self.vb_points = scene.context.buffer(points)
		self.vb_faces = scene.context.buffer(np.arange(points.shape[0], dtype='u4'))
		self.va = scene.context.vertex_array(
			self.shader,
			[(self.vb_points, '2f 2f', 'v_position', 'v_uv')],
			)

	def stack(self, view):
		return (self, 'screen', 2, self._render),
	
	def _render(self, view):		
		self.shader['layer'] = self.layer
		self.shader['color'].write(highlight_color(self, self.color))
		self.shader['position'].write(fvec3(self.world * fvec4(self.position,1)))
		self.shader['view'].write(view.uniforms['view'])
		self.shader['proj'].write(view.uniforms['proj'])
		self.shader['ratio'].value = (
				(self.size-2.5) / view.uniforms['size'].x*2,
				(self.size-2.5) / view.uniforms['size'].y*4,
				)
		self.fonttex.use(0)
		self.va.render(mgl.TRIANGLES)
