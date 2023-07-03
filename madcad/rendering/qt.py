import importlib

for module in ("PySide6", "PyQt6", "PyQt5"):
    try:
        OpenGL_suffix =".QtWidgets" if module == "PyQt5" else ".QtOpenGLWidgets"
        # Modules
        QtCore    = importlib.import_module(module + ".QtCore")
        QtWidgets = importlib.import_module(module + ".QtWidgets")
        QtGui     = importlib.import_module(module + ".QtGui")
        QtOpenGL  = importlib.import_module(module + OpenGL_suffix)

        # Classes
        QPoint  = QtCore.QPoint
        QPointF = QtCore.QPointF
        QEvent  = QtCore.QEvent
        Qt      = QtCore.Qt

        QApplication  = QtWidgets.QApplication
        QWidget       = QtWidgets.QWidget
        QOpenGLWidget = QtOpenGL.QOpenGLWidget

        QSurfaceFormat = QtGui.QSurfaceFormat
        QMouseEvent    = QtGui.QMouseEvent
        QInputEvent    = QtGui.QInputEvent
        QKeyEvent      = QtGui.QKeyEvent
        QTouchEvent    = QtGui.QTouchEvent
        QFocusEvent    = QtGui.QFocusEvent

        # Special objects
        QAllEvents             = QEvent.Type if module == "PyQt6" else QEvent
        QMouseButton           = Qt.MouseButton if module == "PyQt6" else Qt
        QKeys                  = Qt.Key if module == "PyQt6" else Qt
        ApplicationAttribute   = Qt.ApplicationAttribute if module == "PyQt6" else Qt
        AA_ShareOpenGLContexts = ApplicationAttribute.AA_ShareOpenGLContexts

        # Version
        QVersion = module
        break
    except ModuleNotFoundError:
        pass
