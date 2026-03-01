from madcad.mesh import web
from madcad.generation import brick, icosphere, uvsphere
from madcad.primitives import Circle
from madcad.mathutils import O, X, Y, Z, vec3
from madcad.hull import convexhull, convexoutline
from . import visualcheck

@visualcheck
def test_hull():
	results = []
	for i, obj in enumerate([
			brick(width=vec3(1)),
			icosphere(O, 1),
			brick(width=vec3(1)) + icosphere(vec3(1,2,3), 1),
			brick(width=vec3(1)) + icosphere(vec3(0,1,0), 1),
			brick(center=vec3(0,0,1), width=vec3(1)) + uvsphere(vec3(0,1,0), 1) + icosphere(vec3(0,-1,0), 0.8),
			web(Circle((O,Z), 1)),
			web(Circle((O,Z), 1), Circle((vec3(0,1,1), vec3(1,2,1)), 1), Circle((vec3(1.2,-1,1), X), 0.5)),
			web(Circle((O,Z), 0.2), Circle((vec3(0,2,0.5),Z), 1)),
			]):
		hull = convexhull(obj)
		hull.check()
		assert hull.isenvelope()
		results.append(hull.transform(i*2*X))
		
		contour = convexoutline(obj, normal=Z)
		contour.check()
		assert contour.isloop()
		results.append(contour.transform(i*2*X+4*Y))
		
		contour = convexoutline(obj, flatten=True)
		contour.check()
		assert contour.isloop()
		results.append(contour.transform(i*2*X+8*Y))

	return results
