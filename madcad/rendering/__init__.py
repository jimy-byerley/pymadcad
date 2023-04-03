from .d3 import Scene3D, SubView3D, QView3D
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication
from .. import settings
from ..mathutils import *
from copy import deepcopy

try:
    from . import qt
except ImportError:
    pass
else:
    from .d3 import QView3D, opengl_version

    def show(scene: dict, interest: Box = None, size=uvec2(400, 400), projection=None, navigation=None, **options):
        """
        Easy and convenient way to create a window containing a `View` on a created `Scene`

        If a Qt app is not already running, the functions returns when the window has been closed and all GUI destroyed

        Parameters:
            scene:     a mapping (dict or list) giving the objects to render in the scene
            interest:  the region of interest to zoom on at the window initialization
            size:      the window size (pixel)
            options:   options to set in `Scene.options`

        Tip:
            For integration in a Qt window or to manipulate the view, you should directly use `View`
        """
        global global_context

        if isinstance(scene, list):
            scene = dict(enumerate(scene))
        # Retro-compatibility fix, shall be removed in future versions
        if "options" in options:
            options.update(options["options"])
        if not isinstance(scene, Scene3D):
            scene = Scene3D(scene, options)

        app = QApplication.instance()
        created = False
        if not app:
            import sys

            QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
            app = QApplication(sys.argv)
            global_context = None
            created = True

        # Use the Qt color scheme if specified
        if settings.display["system_theme"]:
            settings.use_qt_colors()

        # Create the scene as a window
        view = QView3D(scene, projection=projection, navigation=navigation)
        view.resize(*size)
        view.show()

        # Make the camera see everything
        if not interest:
            interest = view.scene.box()
        view.center(interest.center)
        view.adjust(interest)

        if created:
            err = app.exec()
            if err != 0:
                print("error: Qt exited with code", err)


def render(scene, options=None, interest: Box = None, navigation=None, projection=None, size=uvec2(400, 400)):
    """
    Shortcut to render the given objects to an image, returns a PIL Image

    For repeated renderings or view manipulation, you should directly use `Offscreen`

    NOTE:
            The system theme colors cannot be automatically loaded since no running QApplication is assumed in the function
    """
    if isinstance(scene, list):
        scene = dict(enumerate(scene))
    if not isinstance(scene, Scene):
        scene = Scene(scene, options)

    # Create the scene and an offscreen renderer
    view = Offscreen3D(scene, size, navigation=navigation, projection=projection)

    # Load objects in the scene, so the scene's box can be computed
    with scene.ctx:
        scene.dequeue()

    # Make the camera see everything
    if not navigation:
        if not interest:
            interest = view.scene.box()
        view.center(interest.center)
        view.adjust(interest)

    return view.render()
