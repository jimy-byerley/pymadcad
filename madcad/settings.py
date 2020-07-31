'''	 The settings module holds dictionnaries each aspect of the madcad library.

dictionnaries:
	:primitives:  default settings for mesh and primitive operations
	:display:	visual settings to display the 3D objects
	:scene:		for what and how to display in the 3D scene
	:controls:	preferences for the controls of the Scene widget
'''

from math import pi, ceil, floor, sqrt
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
	'annotation_color': (0.2, 0.7, 1),
	'line_width': 1.,
	}

scene = {
	'display_faces': True,
	'display_groups': True,
	'display_points': False,
	'display_wire': False,
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
	''' return the subdivision number for a curve, using the given or setting specification

		:length:  is the curvilign length of the curve
		:angle:   is the integral of the absolute curvature (total rotation angle)
	'''
	kind, prec = param or primitives['curve_resolution']
	if kind == 'div':
		res = prec
	elif kind == 'm':
		res = ceil(length / prec)
	elif kind == 'rad':
		res = floor(angle / prec)
	elif kind == 'radm':
		res = floor(angle * length / prec)
	elif kind == 'radm2':
		res = floor(angle * sqrt(length) / prec)
	else:
		raise ValueError("unknown type for round_limit: {}".format(repr(kind)))
	res = max(res, floor(angle * 2/pi))
	return res

def getparam(levels: list, key):
	''' get the first found value for key through the given dictionnaries.
		Dictionnaries are tested successively until the matching value is found. If no value is found, None is returned
	'''
	for d in levels:
		if d is not None:
			if key in d:	return d[key]
	return None
