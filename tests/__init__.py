from __future__ import annotations

import inspect, pickle
from os import mkdir
from os.path import abspath, exists
from functools import wraps, reduce
from operator import xor

from madcad.rendering import show

def visualcheck(test:Callable) -> Callable:
    ''' decorator for non-regression tests, providing snapshot caching and visual inspection '''
    @wraps(test)
    def wrapper(*args, **kwargs):
        bound = inspect.signature(test).bind(*args, **kwargs)
        bound.apply_defaults()
        if bound.arguments:
            key = reduce(xor, map(hash, sorted(bound.arguments.items())))
        else:
            key = 0
        
        references = abspath('{}/../__snapshot__'.format(__file__))
        if not exists(references):
            mkdir(references)
            
        fullname = '{}/{}.{}.{:x}.pickle'.format(
            references,
            test.__globals__['__name__'], 
            test.__qualname__, 
            key)
        title = '{}.{}({})'.format(
            test.__globals__['__name__'],
            test.__qualname__,
            repr(bound.arguments)[1:-1],
            )
        
        result = test()
        try:
            reference = pickle.load(open(fullname, 'rb'))
            if result != reference:
                raise AssertionError("result of {} with arguments {} doesn't match cache".format(test.__name__, bound))
        except (OSError, AssertionError) as err:
            if _visualinspect(title, reference, result):
            # if input('result is different from last time\n is it correct ?') == 'y':
                pickle.dump(result, open(fullname, 'wb'))
            else:
                raise
    return wrapper

app = None

def _visualinspect(title, reference, result) -> bool:
    ''' pop a 3d view to visually inspect the result and a question box to give user answer '''
    from madcad.qt import QApplication, QMessageBox
    from madcad.rendering import QView3D, Scene
    from pnprint import nprint
    
    global app
    if app is None:
        app = QApplication([])
    
    # just like in `show`
    view = QView3D(Scene(result, options=dict(display_wire=True, display_points=True)))
    view.scene.prepare()
    interest = view.scene.root.box
    view.center(interest.center)
    view.adjust(interest)
    view.resize(600,600)
    
    # usual Qt stuff
    question = QMessageBox()
    question.setText("result is different from last time\n is it correct ?")
    question.setStandardButtons(QMessageBox.Save | QMessageBox.Discard)
    question.setModal(False)
    answer = False
    @question.buttonClicked.connect
    def retreive(button):
        nonlocal answer
        answer = question.buttonRole(button) == QMessageBox.ButtonRole.AcceptRole
        view.close()
        app.quit()
    question.resize(question.sizeHint())
    
    nprint('\u2022 test result', result)
    nprint('\u2022 test reference', reference)
    
    view.show()
    view.setWindowTitle(title)
    question.setParent(view)
    question.move(0,0)
    question.show()
    app.exec_()
    
    return answer
    
