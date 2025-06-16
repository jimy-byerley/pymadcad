from madcad import *
from . import visualcheck

@visualcheck
def test_splines():
    results = []
    def check(curve):
        mesh = curve.mesh()
        mesh.check()
        assert all(isfinite(p) for p in mesh.points)
        results.append(curve)
        results.append(mesh)

    points = [vec3(0), vec3(1), vec3(2,0,1), vec3(3,3,1)]
    check(Softened(points))
    check(Interpolated(points))
    return results
