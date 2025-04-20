from numpy import ndarray

from ..mathutils import uvec2, Box
from .utils import *
from .base import *
from .d3 import *
from .d2 import *

try:
	from ..qt import QApplication
except ImportError:
	pass
else:

	def show(scene:Scene|dict|list, interest: Box = None, size=uvec2(400, 400), projection=None, navigation=None, **options):
		'''
		Easy and convenient way to create a window containing a `View3D` on a `Scene`

		If a `QApplication` is not already running, a `QApplication` is created and the functions only returns when the window has been closed and all GUI destroyed

		Parameters:
			scene:     a mapping (dict or list) giving the objects to render in the scene
			interest:  the region of interest to zoom on at the window initialization
			size:      the window size (pixel)
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



def render(scene:Scene|dict|list, size=uvec2(400, 400), view:fmat4=None, proj:fmat4=None, interest:Box=None, **options) -> ndarray:
	'''
	render the given scene to an image

	The output image is a numpy buffer RGB8
	
	For repeated renderings or view manipulation, you should directly use `Offscreen3D` rather than calling this function repeatedly

	NOTE:
		The system theme colors cannot be automatically loaded since no running QApplication is assumed in the function
	'''
	scene = Scene(scene, options)
	if not interest:
		interest = scene.root.box
	size = length(interest.width)
	if not proj:
		proj = globals()[settings.scene['projection']]().matrix(size)
	if not view:
		view = translate(interest.center + size*Z)
		# TODO take the projection transformation into account to get an appropriate distance
	view = Offscreen3D(scene, size, view, proj)
	scene.prepare()
	return view.render()


