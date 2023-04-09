import importlib

for module in ("PySide6", "PyQt6", "PyQt5"):
    try:
        # Modules
        QtCore    = importlib.import_module(module + ".QtCore")
        QtWidgets = importlib.import_module(module + ".QtWidgets")
        QtGui     = importlib.import_module(module + ".QtGui")

        # Classes
        QPoint = QtCore.QPoint
        QEvent = QtCore.QEvent
        Qt     = QtCore.Qt

        QApplication  = QtWidgets.QApplication
        QWidget       = QtWidgets.QWidget
        QOpenGLWidget = QtWidgets.QOpenGLWidget

        QSurfaceFormat = QtGui.QSurfaceFormat
        QMouseEvent    = QtGui.QMouseEvent
        QInputEvent    = QtGui.QInputEvent
        QKeyEvent      = QtGui.QKeyEvent
        QTouchEvent    = QtGui.QTouchEvent
        QFocusEvent    = QtGui.QFocusEvent
        break
    except ModuleNotFoundError:
        pass
