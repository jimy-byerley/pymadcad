from importlib.util import find_spec

if find_spec('PySide6'):
    from PySide6.QtWidgets import *
    from PySide6.QtGui import *
    from PySide6.QtCore import *

elif find_spec('PyQt5'):
    from PyQt5.QtWidgets import *
    from PyQt5.QtGui import *
    from PyQt5.QtCore import *

    Signal = pyqtSignal

elif find_spec('PyQt6'):
    from PyQt5.QtWidgets import *
    from PyQt5.QtGui import *
    from PyQt5.QtCore import *

    Signal = pyqtSignal

else:
    raise ImportError('no Qt wrapper found: PySide6, PyQt5, PyQt6')
