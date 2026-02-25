'''
    minimal abstraction layer for the Qt wrapper
    
    it exposes the content of `QtCore`, `QtGui` and `QtWidgets`
    
    when imported this module proceeds as follow:
    - check if a supported Qt library is already loaded, then load it
    - else load the first available Qt library in the priority order
'''
import sys, importlib

alternatives = ['PyQt5', 'PySide6', 'PyQt6']

def use(lib):
    backend = importlib.import_module(lib)
    module = importlib.import_module(__name__+'.'+lib)
    globals().update(vars(module))

for lib in alternatives:
    if lib in sys.modules:
        use(lib)
        break
else:
    for lib in alternatives:
        try:
            use(lib)
        except ImportError as e:
            pass
        else:
            break
    else:
        raise ImportError(
            'no Qt wrapper found: PySide6, PyQt5, PyQt6. \n'
            'Please install pyqt5 or pyqt6 or pyside6 by running one of these commands: \n'
            'pip install pyqt5\n'
            'pip install pyqt6\n'
            'pip install pyside6'
        )
