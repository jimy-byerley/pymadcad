# This file is part of pymadcad,  distributed under license LGPL v3
'''	
	Display module of pymadcad
	
	This module provides a render pipeline system featuring:

	- Class `Scene` to gather the data to render, basically instances of `Display`
	- many kind of `View` that actually renders the scene 
	- The `Display` base class meant to be subclassed into any kind of display
	- The `Group` class that allows to nest displays in the scene

	The `Scene` is only to manage the objects to render . Almost all madcad data types can be rendered to scenes being converted into an appropriate subclass of `Display`. Since the conversion from madcad data types into display instance is automatically handled via the object's `display` method (or scene overrides), you usually don't need to deal with displays directly.
	
	The views can be widgets or simply a bunch of buffers to render the scene onto. There is different view implementations for different use cases.
		
	WARNING
	-------
		As the GPU native precision is f4 (float 32 bits), all the vector stuff regarding rendering is made using simple precision types: `fvec3, fvec4, fmat3, fmat4, ...`
'''
from __future__ import annotations

import numpy as np

from ..box import Box
from .. import settings
from ..mathutils import fmat4, uvec2
from .utils import (
	writeproperty, forwardproperty, sceneshare, receiver, Weak, Rc, CheapMap,
	snail, snailaround, glsize, highlight_color, vec_to_qpoint, qpoint_to_vec,
	vec_to_qsize, qsize_to_vec,
)
from .base import Scene, Step, Display, Group, Displayable
from .d3.view import GLView3D, Offscreen3D, QView3D, Orbit, Turntable, Perspective, Orthographic
# from .d2.view import *

__all__ = [
	"CheapMap",
	"Display",
	"Displayable",
	"GLView3D",
	"Group",
	"Offscreen3D",
	"Orbit",
	"Orthographic",
	"Perspective",
	"QView3D",
	"Rc",
	"Scene",
	"Step",
	"Turntable",
	"Weak",
	"forwardproperty",
	"glsize",
	"highlight_color",
	"qpoint_to_vec",
	"qsize_to_vec",
	"receiver",
	"sceneshare",
	"snail",
	"snailaround",
	"vec_to_qpoint",
	"vec_to_qsize",
	"writeproperty",
	"show",
	"render",
]

try:
	from ..qt import QApplication
except ImportError:
	pass
else:

	def show(scene:Scene|dict|list, interest: Box = None, size=uvec2(400, 400), projection=None, navigation=True, **options):
		'''
		Easy and convenient way to create a window containing a `View3D` on a `Scene`

		If a `QApplication` is not already running, a `QApplication` is created and the functions only returns when the window has been closed and all GUI destroyed

		Parameters:
			scene:     a mapping (dict or list) giving the objects to render in the scene
			interest:  the region of interest to zoom on at the window initialization
			size:      the window size (pixel)
			projection: an object handling the camera projection (aka intrinsic parameters), see `QView3D.projection`
			navigation: an object handling the camera movements, see `QView3D.navigation`
			options:   options to set in `Scene.options`

		Tip:
			For integration in a Qt window or to manipulate the view, you should directly use `View`
		'''
		import sys
		
		if not isinstance(scene, Scene):
			scene = Scene(scene, options)

		# detect existing QApplication
		app = QApplication.instance()
		created = False
		if not app:
			app = QApplication(sys.argv)
			created = True

		# use the Qt color scheme if specified
		if settings.display['system_theme']:
			settings.use_qt_colors()

		# create the scene as a window
		view = QView3D(scene, projection=projection, navigation=navigation)
		view.resize(*size)
		view.show()
		scene.prepare()

		# make the camera see everything
		if not interest:	
			interest = view.scene.root.box
		view.center(interest.center)
		view.adjust(interest)

		# run eventually created QApplication
		if created:
			err = app.exec()
			if err != 0:
				print('error: Qt exited with code', err)


def render(scene:Scene|dict|list, size=uvec2(400, 400), view:fmat4=None, proj:fmat4=None, interest:Box=None, alpha=False, **options) -> np.ndarray:
	'''
	render the given scene to an image

	The output image is a numpy tensor RGB8 (w,h,3) or RGBA8 (w,h,4)
	For repeated renderings or view manipulation, you should directly use `Offscreen3D` rather than calling this function repeatedly

	NOTE:
		The system theme colors cannot be automatically loaded since no running QApplication is assumed in the function
		
	Parameters:
		scene:     a mapping (dict or list) giving the objects to render in the scene
		interest:  the region of interest to zoom on at the window initialization
		size:      the resulting image size (pixel)
		view:    the view matrix, which should be a affine matrix with rotation and translation
		proj:    the openGL projection matrix, could be generated with `perspective` or `orthographic`		
		alpha:   whether to add an alpha channel for the background in the resulting image
	'''
	from ..mathutils import length, translate, Z, fvec3
	
	scene = Scene(scene, options)
	scene.prepare()
	if not interest:
		interest = scene.root.box
	distance = length(interest.size)
	if not proj:
		proj = globals()[settings.scene['projection']]().matrix(size.y/size.x, distance)
	if not view:
		view = translate(interest.center + distance*fvec3(Z))
		# TODO take the projection transformation into account to get an appropriate distance
	view = Offscreen3D(scene, size, view, proj, enable_alpha=alpha)
	return view.render().color
