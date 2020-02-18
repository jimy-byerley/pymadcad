'''	Utilities and default parameters for pymadcad
	
	Attributes:
		*  primitives		- dictionnary of parameters for primitives
'''

from math import pi, ceil, sqrt
from PyQt5.QtCore import Qt


display = {
	'view_font_size': 8,
	'field_of_view': pi/6,
	'view_limits': (0.1, 1024.),
	'solid_color': (0.2, 0.2, 0.2),
	'solid_color_front': 1.,
	'solid_color_side': 0.2,
	'line_color': (0.9, 0.9, 0.9),
	'select_color_face': (0.01, 0.05, 0.03),
	'select_color_line': (0.5, 1, 0.6),
	'schematics_color': (0.3, 0.8, 1),
	'line_width': 1.,
	}

controls = {
	'zoom_sensitivity':	1.,
	'orbit_sensitivity': 4.,
	'pan_sensitivity': 0.8,
	}

primitives = {
	'merge_limit': 0.01,	# distance minimale avant fusion des points lors de la simplification
	'curve_resolution': ('rad', pi/16),	# angle maximal pour discretisation des courbes
	}

def curve_resolution(length, angle, param=None):
	''' procedure donnant le nombre de points a placer sur une courbe
		- length est la longueur curviligne
		- angle est l'angle entre le debut de la courbe
		length et angle doivent caracteriser une portion de la courbe a courbure monotone
	'''
	kind, prec = param or primitives['curve_resolution']
	if kind == 'div':
		res = prec
	elif kind == 'm':
		res = ceil(length / prec)
	elif kind == 'rad':
		res = ceil(angle / prec)
	elif kind == 'radm':
		res = ceil(angle * length / prec)
	elif kind == 'radm2':
		res = ceil(angle * sqrt(length) / prec)
	else:
		raise ValueError("unknown type for round_limit: {}".format(repr(kind)))
	res = max(res, ceil(angle * 2/pi))
	return res

def getparam(levels: list, key):
	''' Donne la valeur correspondant a la clef dans les dictionnaire
		Les dictionnaires sont interrogés les uns apres les autres, la premiere valeur trouvée est utilisée
	'''
	for d in levels:
		if d is not None:
			if key in d:	return d[key]
	return None
