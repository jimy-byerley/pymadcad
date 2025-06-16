import freetype

from madcad import *
from madcad.text import text, character_primitives, character_surface, character_outline, font_path
from madcad.generation import extrusion
from madcad.scheme import note_distance
from madcad.triangulation import triangulation
from . import visualcheck

@visualcheck
def test_character_primitives():
	face = freetype.Face(font_path('NotoMono-Regular'))
	face.set_char_size(1024)
	face.load_char('&')
	return [ triangulation(character_primitives(face)) ]

@visualcheck
def test_character_cached():
	return [ 
		character_outline('&', 'NotoMono-Regular')['web'],
		character_surface('g', 'NotoMono-Regular')['mesh'].transform(vec3(1,0,0)),
		]

@visualcheck
def test_text():
	settings.display['view_font_size'] = 10
	part = text('Hello everyone.\nthis is a great font !!', font='NotoSans-Regular', align=('center', 0))
	part = extrusion(part.flip(), vec3(0,0,-1))
	return [
		vec3(0),
		note_distance(vec3(0), vec3(0,1,0), offset=vec3(-0.5,0,0), text='size'),
		part,
		]
