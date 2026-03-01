from __future__ import annotations

import pickle
from collections.abc import Callable
from os import mkdir
from os.path import abspath, exists
from functools import wraps

from pnprint import nformat

from madcad.qt import QApplication, QMessageBox, QWidget, QLabel, QGridLayout, QPlainTextEdit, QFont
from madcad.rendering import QView3D, Scene

def visualcheck(test: Callable) -> Callable:
    ''' decorator for non-regression tests, providing snapshot caching and visual inspection '''
    @wraps(test)
    def wrapper(*args, **kwargs):
        references = abspath('{}/../__snapshot__'.format(__file__))
        if not exists(references):
            mkdir(references)
            
        fullname = '{}/{}.{}.pickle'.format(
            references,
            test.__globals__['__name__'], 
            test.__qualname__, 
            )
        title = '{}.{}'.format(
            test.__globals__['__name__'],
            test.__qualname__,
            )
        
        result = test(*args, **kwargs)
        reference = None
        try:
            binary_result = pickle.dumps(result)
            binary_reference = open(fullname, 'rb').read()
            reference = pickle.loads(binary_reference)
            if binary_result != binary_reference:
                raise AssertionError("result of {} doesn't match cache".format(test.__name__))
        except (OSError, AssertionError, FileNotFoundError) as err:
            if _visualinspect(title, reference, result):
                open(fullname, 'wb').write(binary_result)
            else:
                raise
    return wrapper

app = None

def _visualinspect(title, reference, result) -> bool:
    ''' pop a 3d view to visually inspect the result and a question box to give user answer '''    
    global app
    if app is None:
        app = QApplication([])
    # app = QApplication([])
    
    # ask the question to the user
    question = QMessageBox()
    question.setText("result is different from last time\n is it correct ?")
    question.setStandardButtons(QMessageBox.Save | QMessageBox.Discard)
    question.setModal(False)
    answer = False
    @question.buttonClicked.connect
    def retreive(button):
        nonlocal answer
        answer = question.buttonRole(button) == QMessageBox.ButtonRole.AcceptRole
        window.close()
        # app.quit()
    
    # text and geometry views for results and reference side by side
    if Scene.displayable(Scene, reference):
        reference_geometry = _geometry_view(reference)
    else:
        reference_geometry = QLabel('non displayable')
    if Scene.displayable(Scene, result):
        result_geometry = _geometry_view(result)
    else:
        result_geometry = QLabel('non displayable')
    reference_text = _text_view(reference)
    result_text = _text_view(result)
    
    reference_text.verticalScrollBar().valueChanged.connect(result_text.verticalScrollBar().setValue)
    result_text.verticalScrollBar().valueChanged.connect(reference_text.verticalScrollBar().setValue)
    
    layout = QGridLayout()
    layout.addWidget(QLabel('reference'), 0, 0)
    layout.addWidget(reference_geometry, 1, 0)
    layout.addWidget(reference_text, 2, 0)
    layout.addWidget(QLabel('result'), 0, 1)
    layout.addWidget(result_geometry, 1, 1)
    layout.addWidget(result_text, 2, 1)
    window = QWidget()
    window.setLayout(layout)
    
    # floating question box
    question.setParent(window)
    question.move(10,430)
    question.resize(question.sizeHint())
    question.show()
    
    window.show()
    # window.resize(800,500)
    window.setWindowTitle(title)
    app.exec_()
    # question.exec_()
    
    return answer
    
def _text_view(obj):
    view = QPlainTextEdit(nformat(obj).replace('\t', '    '))
    view.document().setDefaultFont(QFont('monospace', 7))
    return view

def _geometry_view(obj):
    view = QView3D(Scene(obj, options=dict(display_wire=True, display_points=True)))
    view.scene.prepare()
    interest = view.scene.root.box
    view.center(interest.center)
    view.adjust(interest)
    view.setMinimumSize(500,500)
    return view

class Hidden:
    ''' a data container that compare its content but doesn't show it '''
    def __init__(self, obj):
        self.obj = obj
    def __eq__(self, other):
        return isinstance(other, type(self)) and self.obj == other.obj
