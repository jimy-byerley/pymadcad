import importlib

# for module in ("PySide6", "PyQt6", "PyQt5"):
for module in ("PyQt5", ):
    # try:
    OpenGL_suffix = ".QtOpenGLWidgets" if module == "PyQt6" else ".QtWidgets"
    # Modules
    QtCore    = importlib.import_module(module + ".QtCore")
    QtWidgets = importlib.import_module(module + ".QtWidgets")
    QtGui     = importlib.import_module(module + ".QtGui")
    QtOpenGL = importlib.import_module(module + OpenGL_suffix)

    # Classes
    QPoint = QtCore.QPoint
    QEvent = QtCore.QEvent
    Qt     = QtCore.Qt

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
    ApplicationAttribute = Qt.ApplicationAttribute if module == "PyQt6" else Qt
    AA_ShareOpenGLContexts = ApplicationAttribute.AA_ShareOpenGLContexts
    QAllEvents = (QEvent.Type if module == "PyQt6" else QEvent)
    QMouseButton = (Qt.MouseButton if module == "PyQt6" else Qt)
    QKeys = (Qt.Key if module == "PyQt6" else Qt)

    # Version
    print(module)
    QVersion = "PyQt5"
    break
    # except ModuleNotFoundError:
    #     pass
