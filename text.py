from PIL import Image, ImageFont, ImageDraw
import numpy.core as np
import moderngl

# TODO: utiliser les methodes et attributs ImgeFont.size, .getmetrics(), etc pour avoir les hauteur et largeur de police

def create_font_texture(font, maxchar=1100):
	# determine the size of the needed texture
	imsize = 128
	fontsize = font.size + 4
	hold = False
	while 2*(imsize//fontsize)**2 < maxchar:
		imsize *= 2
	# create texture
	tex = Image.new('L', (imsize,imsize), 0)
	draw = ImageDraw.Draw(tex)
	# draw the font onto
	c = 2*imsize // fontsize
	l = imsize // fontsize
	for i in range(maxchar):
		draw.text(char_placement(font.size, c, l, i), chr(i), fill=255, font=font)
	
	return tex, (c, l)

class Text:
	def __init__(self, position, text, size):
		self.text = text
		self.position = position
		self.size = size
	
	def display(self, scene):
		return TextDisplay(scene, self.position, self.text, self.size, color=(0.2, 0.8, 1))	# TODO: ask the color at initialization

def char_placement(fontsize, c, l, n):
	fontsize += 4
	return (fontsize//2 * (n%c), fontsize * (n//c))

pointsdef = [
	[0, 0],
	[0,-1],
	[1,-1],
	[0, 0],
	[1,-1],
	[1, 0],
	]

class TextDisplay:
	def __init__(self, scene, position, text, size, color):
		self.position = position
		self.color = color
		
		# load font
		def load(scene):
			img, align = create_font_texture(ImageFont.truetype('NotoMono-Regular.ttf', 10))
			return scene.ctx.texture(img.size, 1, img.tobytes()), align
		self.fonttex, self.fontalign = scene.ressource(('fonttex', size), load)
		
		# load shader
		def load(scene):
			shader = scene.ctx.program(
						vertex_shader=open('shaders/font.vert').read(),
						fragment_shader=open('shaders/font.frag').read(),
						)
			shader['fonttex'].value = 0
			return shader
		self.shader = scene.ressource('shader_font', load)
		
		# place triangles
		points = np.empty((6*len(text), 4), np.float32)
		l = 0
		c = 0
		for i,char in enumerate(text):
			if char == '\n':
				c = 0
				l += 1
			elif char == '\t':
				c += 4 - c%4	# TODO: use a tab size in settings
			else:
				n = ord(char)
				for j,add in enumerate(pointsdef):
					points[len(pointsdef)*i + j] = [
						(c+add[0])*(size+4)/scene.width(), 
						(-l+add[1])*2*(size+4)/scene.height(), 
						(n % self.fontalign[0] +add[0]) / self.fontalign[0], 
						(n //self.fontalign[0] -add[1]) / self.fontalign[1],
						]
				c += 1
		
		# create the buffers on the GPU
		self.vb_points = scene.ctx.buffer(points)
		self.vb_faces = scene.ctx.buffer(np.arange(points.shape[0], dtype=np.uint32))
		self.va = scene.ctx.vertex_array(
			self.shader,
			[(self.vb_points, '2f 2f', 'v_position', 'v_uv')],
			)
	
	def render(self, scene):		
		self.shader['color'].value = self.color
		self.shader['position'].value = self.position
		self.shader['view'].write(scene.view_matrix)
		self.shader['proj'].write(scene.proj_matrix)
		self.fonttex.use(0)
		scene.ctx.point_size = 3
		self.va.render(moderngl.TRIANGLES)

	def identify(self, scene, ident):
		pass
		

def test_text_display():
	import sys
	from PyQt5.QtWidgets import QApplication
	from view import Scene
	
	app = QApplication(sys.argv)
	scn = Scene()
	scn.add(Text((0,0,0), 'coucou', 10))
	
	scn.show()
	sys.exit(app.exec())

def test_font_texture():
	fontsize = 10
	im, shape = create_font_texture(ImageFont.truetype('NotoMono-Regular.ttf', fontsize))
	im.show()


if __name__ == '__main__':
	#test_font_texture()
	test_text_display()
