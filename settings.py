'''	Utilities and default parameters for pymadcad
	
	Attributes:
		*  primitives		- dictionnary of parameters for primitives
'''

from math import pi, ceil
from PyQt5.QtCore import Qt


display = {
	'field_of_view': pi/6,
	'view_limits': (0.1, 1024.),
	'solid_color': (0.8, 0.8, 0.8),
	'line_color': (0.9, 0.9, 0.9),
	'select_color': (1., 0.4, 0.1),
	'line_width': 1.,
	}

controls = {
	'zoom_sensitivity':	1.,
	'orbit_sensitivity': 4.,
	'pan_sensitivity': 4.,
	}

primitives = {
	'merge_limit': 0.01,	# distance minimale avant fusion des points lors de la simplification
	'round_limit': ('rad/m', 0.1),	# angle maximal pour discretisation des courbes
	}

def curve_resolution(radius, angle):
	''' procedure donnant le nombre de points a placer sur un arc couvrant un angle donné 
		utilise le parametre 'round_limit'
	'''
	kind, prec = primitives['round_limit']
	if kind == 'm':
		return ceil(radius * angle / prec)
	elif kind == 'rad':
		return ceil(angle / prec)
	elif kind == 'rad/m':
		return ceil(angle / (prec*radius))
	else:
		raise ValueError("unknown type for round_limit: {}".format(repr(kind)))

def getparam(levels: list, key):
	''' Donne la valeur correspondant a la clef dans les dictionnaire
		Les dictionnaires sont interrogés les uns apres les autres, la premiere valeur trouvée est utilisée
	'''
	for d in levels:
		if d is not None:
			if key in d:	return d[key]
	return None
