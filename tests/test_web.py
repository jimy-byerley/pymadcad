from pnprint import nprint

from random import random
from madcad import *
from madcad.mesh import *
from . import visualcheck

def test_islands():
    m = Web(
        [vec3(0), vec3(1,0,0), vec3(1,0,0), vec3(2,1,0), vec3(1,3,0), vec3(1,5,4), vec3(2,5,0)], 
        [(0,1), (1,2), (2,3), (4, 5), (5,6), (6,4)])
    m.check()
    assert len(m.islands()) == 2

def test_frontiers():
    bri = brick(center=vec3(2,0,0), width=vec3(0.5))
    m = bri.frontiers(0,2,3).frontiers({1,2,None})
    assert set(m.indices) == {5, 1, 2}
