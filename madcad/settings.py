'''	 The settings module holds dictionnaries each aspect of the madcad library.

dictionnaries:
	:primitives:  default settings for mesh and primitive operations
	:display:	visual settings to display the 3D objects
	:scene:		for what and how to display in the 3D scene
	:controls:	preferences for the controls of the Scene widget
'''

from math import pi, ceil, floor, sqrt
from glm import fvec3, fvec4
import sys, os, yaml
from os.path import dirname, exists


display = {
	'field_of_view': pi/6,
	'view_font_size': 8,
	'line_width': 1.,
	
	'background_color': fvec3(0,0,0),
	'select_color_face': fvec3(0.01, 0.05, 0.03),
	'select_color_line': fvec3(0.5, 1, 0.6),
	'highlight_color': fvec3(0.1, 0.2, 0.2),
	
	'solid_color': fvec3(0.2, 0.2, 0.2),
	'solid_color_front': 1.,
	'solid_color_side': 0.2,
	'solid_reflectivity': 6,
	'solid_reflect': 'skybox-white.png',
	'line_color': fvec3(0.9, 0.9, 0.9),
	'point_color': fvec3(0.9, 0.9, 0.9),
	'schematics_color': fvec3(0.3, 0.8, 1),
	# According to moderngl docs changing this is not guaranteed to work
	# (depends on OpenGL implementation)
	'line_width': 1.0,
	'solver_error_color': fvec3(1, 0.3, 0.2),
	'annotation_color': fvec3(0.2, 0.7, 1),
	'sharp_angle': pi/6,
	
	'system_theme': True,
	}

scene = {
	'projection': 'Perspective',
	
	'display_faces': True,
	'display_groups': True,
	'display_points': False,
	'display_wire': False,
	'display_grid': True,
	'display_annotations': True,
	'surface_shading': True,
	
	'lock_solids': True,
	
	'debug_points': False,
	'debug_faces': False,
	'debug_groups': False,
	}
		
controls = {
	'navigation': 'Turntable',
	'snap_dist': 10,	# pixel distance to click items
	}

primitives = {
	'curve_resolution': ['rad', pi/16],	# angle maximal pour discretisation des courbes
	}

# get configuration directory depending on OS
if sys.platform == 'win32':
	home = os.getenv('USERPROFILE')
	configdir = home+'/AppData/Local'
else:
	home = os.getenv('HOME')
	configdir = home+'/.config'

config = configdir+'/madcad/pymadcad.yaml'
settings = {'display':display, 'scene':scene, 'controls':controls, 'primitives':primitives}


def install():
	''' create and fill the config directory if not already existing '''
	if not exists(config):
		os.makedirs(dirname(config), exist_ok=True)
		dump()
		
def clean():
	''' delete the default configuration file '''
	os.rm(config)

def load(file=None):
	''' load the settings directly in this module, from the specified file or the default one '''
	if not file:	file = config
	if isinstance(file, str):	file = open(file, 'r')
	changes = yaml.safe_load(file)
	def update(dst, src):
		for key in dst:
			if key in src:
				if isinstance(dst[key], dict) and isinstance(src[key], dict):	
					update(dst[key], src[key])
				elif isinstance(dst[key], fvec3):	dst[key] = fvec3(src[key])
				elif isinstance(dst[key], fvec4):	dst[key] = fvec4(src[key])
				else:
					dst[key] = src[key]
	update(settings, changes)

def dump(file=None):
	''' load the current settings into the specified file or to the default one '''
	if not file:	file = config
	if isinstance(file, str):	file = open(file, 'w')
	yaml.add_representer(fvec3, lambda dumper, data: dumper.represent_list(round(f,3) for f in data))
	yaml.add_representer(fvec4, lambda dumper, data: dumper.represent_list(round(f,3) for f in data))
	file.write(yaml.dump(settings, default_flow_style=None, width=40, indent=4))




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
		res = floor(length*angle / prec)
	elif kind == 'sqradm':
		res = floor(sqrt(length*angle) / prec)
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
	''' set the color settings to fit the current system colors '''
	from .mathutils import fvec3, mix, distance
	from PyQt5.QtWidgets import QApplication
	palette = QApplication.instance().palette()
	def qtc(role):
		''' convert a QColor or QPalette role to fvec3'''
		c = palette.color(role)
		return fvec3(c.red(), c.green(), c.blue()) / 255
		
	
	selection = mix(fvec3(0.4, 1, 0), qtc(palette.Highlight), 0.6)
	selection *= mix(1/max(selection), max(qtc(palette.Text)), 0.3)
	display.update({
		'background_color': qtc(palette.Base),
		'select_color_face': selection * 0.05,
		'select_color_line': selection * 1.1,
		'line_color': qtc(palette.Text),
		'point_color': qtc(palette.Text),
		'solid_color': mix(qtc(palette.Text), qtc(palette.Window), 0.7),
		'schematics_color': qtc(palette.Link),
		'annotation_color': mix(qtc(palette.Highlight), qtc(palette.Text), 0.5),
		})


# automatically load settings in the file exist
try:	load()
except FileNotFoundError:	pass
