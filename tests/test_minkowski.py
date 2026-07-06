
from madcad.minkowski import minkowski
from madcad.generation import icosphere, brick, revolution
from madcad.primitives import ArcCentered
from madcad.mathutils import *

from . import visualcheck

@visualcheck
def test_mesh_mesh():
    results = []
    
    def check(a, b, position):
        result = minkowski(a, b)
        result.check()
        # assert result.issurface()
        # if a.isenvelope() and b.isenvelope():
            # assert result.isenvelope()
        results.append(result.transform(position))
    
    x = 0
    for a, b in [
        (icosphere(O, 1), brick(size=vec3(1))),
        (icosphere(O, 1), revolution(ArcCentered(Axis(O,Z), X*1, -X*1), Axis(O,X), pi/2).finish()),
        ]:
        
        # from random import random
        # for i in range(len(b.points)):
        #     b.points[i] += vec3(random(), random(), random())*1e-4
        # for i in range(len(a.points)):
        #     a.points[i] += vec3(random(), random(), random())*1e-4
        
        assert a.issurface()
        assert b.issurface()
        check(a, b, vec3(x*4, 0, 0))
        check(b, a, vec3(x*4, 4, 0))
        x += 1
    
    return results
