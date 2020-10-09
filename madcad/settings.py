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
	'field_of_view': pi/6,
	'view_font_size': 8,
	'line_width': 1.,
	
	'background_color': (0,0,0),
	'select_color_face': (0.01, 0.05, 0.03),
	'select_color_line': (0.5, 1, 0.6),
	'highlight_color': (0.1, 0.2, 0.2),
	
	'solid_color': (0.2, 0.2, 0.2),
	'solid_color_front': 1.,
	'solid_color_side': 0.2,
	'line_color': (0.9, 0.9, 0.9),
	'point_color': (0.9, 0.9, 0.9),
	'schematics_color': (0.3, 0.8, 1),
	'solver_error_color': (1, 0.3, 0.2),
	'annotation_color': (0.2, 0.7, 1),
	
	'system_theme': True,
	}

scene = {
	'projection': 'Perspective',
	
	'display_faces': True,
	'display_groups': True,
	'display_points': False,
	'display_wire': False,
	'surface_shading': True,
	
	'debug_points': False,
	'debug_faces': False,
	'debug_groups': False,
	}
		
controls = {
	'navigation': 'Turntable',
	'snap_dist': 10,	# pixel distance to click items
	}

primitives = {
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

def use_qt_colors():
	from .mathutils import fvec3, mix, distance
	from PyQt5.QtWidgets import QApplication
	palette = QApplication.instance().palette()
	
	def qtc(role):
		c = palette.color(role)
		return (c.red()/255, c.green()/255, c.blue()/255)
	
	selection = mix(fvec3(0.4, 1, 0), qtc(palette.Highlight), 0.6)
	selection *= mix(1/max(selection), max(qtc(palette.Text)), 0.3)
	display.update({
		'background_color': qtc(palette.Base),
		'select_color_face': tuple(selection * 0.05),
		'select_color_line': tuple(selection * 1.1),
		'line_color': qtc(palette.Text),
		'point_color': qtc(palette.Text),
		'solid_color': qtc(palette.Midlight),
		'schematics_color': qtc(palette.Link),
		'annotation_color': fvec3(qtc(palette.Highlight)),
		})
