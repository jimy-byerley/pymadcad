QtCore = None

if not QtCore:
    try:
        from PySide6 import QtCore
        ...
    except ImportError:
        pass

if not QtCore:
    try:
        from PyQt6 import QtCore
        ...
    except ImportError:
        pass

if not QtCore:
    try:
        from PyQt5 import QtCore
        ...
    except ImportError:
        pass
