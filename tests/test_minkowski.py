
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
        
    concavebrick = brick(size=vec3(1))
    concavebrick.points[2] = vec3(0)
    
    x = 0
    for a, b in [
        (brick(size=vec3(0.5)).transform(quat(X+Y+Z, Z)), brick(size=vec3(1))),
        (icosphere(O, 1), brick(size=vec3(1))),
        (icosphere(O, 1), brick(size=vec3(1)).group({0,2,3})),
        (icosphere(O, 1), brick(size=vec3(1)).group({0,1,2})),
        (icosphere(O, 0.5), concavebrick),
        (icosphere(O, 1), icosphere(O, 0.8)),
        (icosphere(O, 1), revolution(ArcCentered(Axis(O,Z), X*1, -X*1), Axis(O,X), pi/2).finish()),
        (icosphere(O, 0.3), revolution(ArcCentered(Axis(O,Z), X*1.5, -X*1.5), Axis(O,X), pi/2).finish().flip()),
        ]:
        
        # # temporary perturabtion of points to avoid any coplanarity
        # from random import random, seed
        # seed(42)
        # for i in range(len(b.points)):
        #     b.points[i] += vec3(random(), random(), random())*1e-4
        # for i in range(len(a.points)):
        #     a.points[i] += vec3(random(), random(), random())*1e-4
        
        assert a.issurface()
        assert b.issurface()
        check(a, b, vec3(x*4, 0, 0))
        check(b, a, vec3(x*4, 4, 0))
        x += 1
        
    for mesh in results[-6:]:
        mesh.mergegroups()
    
    return results
