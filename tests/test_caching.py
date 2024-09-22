from madcad.io import cached, module, cachedmodule
from madcad.nprint import nprint
import madcad.io

from time import sleep
from dataclasses import dataclass
import textwrap

def test_module():
    open('/tmp/test_module_a.py', 'w').write('''x = 1''')
    open('/tmp/test_module_b.py', 'w').write(textwrap.dedent('''
        from madcad.io import cachedmodule
        a = cachedmodule('/tmp/test_module_a.py')
        x = a.x
        '''))
        
    a = module('/tmp/test_module_a.py')
    assert a.x == 1

    b = module('/tmp/test_module_b.py')
    assert b.x == 1
    assert b.a is not a

    a.x = 2
    assert b.x == 1

def test_cachedmodule():
    disk_delay = 0.1  # (s) time to wait to be sure a file date will be different
    
    open('/tmp/test_module_a.py', 'w').write('''x = 1''')
    open('/tmp/test_module_b.py', 'w').write(textwrap.dedent('''
        from madcad.io import cachedmodule
        a = cachedmodule('/tmp/test_module_a.py')
        x = a.x
        '''))
    
    a = cachedmodule('/tmp/test_module_a.py')
    nprint(madcad.io.caches)
    assert a.x == 1
    b = cachedmodule('/tmp/test_module_b.py')
    nprint(madcad.io.caches)
    assert b.x == 1
    assert b.a is a

    sleep(disk_delay)
    open('/tmp/test_module_a.py', 'w').write('''x = 2''')
    b = cachedmodule('/tmp/test_module_b.py')
    assert b.a is not a
    assert b.x != a.x

    a = cachedmodule('/tmp/test_module_a.py')
    assert b.a is a

def test_cached():
    open('/tmp/test_ab.py', 'w').write(textwrap.dedent('''
        from madcad.io import cached
        from __main__ import AB
        @cached
        def foo(a, b=0):
            return AB(a,b)
        '''))
    
    mod = module('/tmp/test_ab.py')
    foo_1_0 = mod.foo(1, 0)
    foo_1_1 = mod.foo(1, 1)
    foo_1 = mod.foo(1)
    nprint(madcad.io.caches)
    assert foo_1_0 == AB(1,0)
    assert foo_1_1 == AB(1,1)
    assert foo_1_0 == foo_1
    
    del foo_1_1
    nprint(madcad.io.caches)

@dataclass
class AB:
    a: object
    b: object

test_module()
test_cachedmodule()
test_cached()
