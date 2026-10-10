from madcad.mesh import web
from madcad.generation import brick, icosphere, uvsphere
from madcad.primitives import Circle
from madcad.mathutils import O, X, Y, Z, vec3, typedlist, distance2, NUMPREC
from madcad.hull import convexhull, convexoutline, concavehull, restore_groups_ngons
from madcad import core
from . import visualcheck

@visualcheck
def test_hull():
	results = []
	for i, obj in enumerate([
			brick(width=vec3(1)),
			icosphere(O, 1),
			brick(width=vec3(1)) + icosphere(vec3(1,2,3), 1),
			icosphere(O, 1) + icosphere(vec3(1,2,3), 1),
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


@visualcheck
def test_concavehull():
	# same meshes as test_hull uses for convexhull, restricted to the 3D ones (concavehull needs faces)
	# columns: input | convex hull | concave hull ;  rows: one per mesh
	meshes = [
		brick(width=vec3(1)),
		icosphere(O, 1),
		brick(width=vec3(1)) + icosphere(vec3(1,2,3), 1),
		icosphere(O, 1) + icosphere(vec3(1,2,3), 1),
		brick(width=vec3(1)) + icosphere(vec3(0,1,0), 1),
		brick(center=vec3(0,0,1), width=vec3(1)) + uvsphere(vec3(0,1,0), 1) + icosphere(vec3(0,-1,0), 0.8),
		]

	# lay the results in a row, step from the largest object footprint so they just clear each other
	def extent(obj):
		pts = obj.points
		return max(max(p[i] for p in pts) - min(p[i] for p in pts)  for i in range(3))
	step = 1.15 * max(extent(m) for m in meshes)

	results = []
	for i, m in enumerate(meshes):
		concave = concavehull(m, 0.6)
		concave.check()
		results.append(concave.transform(i*step*X - m.barycenter()))
	return results


def test_restore_groups_ngons():
	''' restore_groups_ngons() must give back their original group to the faces of unchanged flat regions '''
	# a cube with its 6 original groups: the hull retriangulates the sides, but each side is
	# still the same flat region, so every face recovers its original group
	mesh = brick(width=vec3(2))
	mesh.strippoints()
	groups = len(mesh.groups)
	normals = {}
	for i in range(len(mesh.faces)):
		normals.setdefault(mesh.tracks[i], mesh.facenormal(i))

	hull = restore_groups_ngons(mesh, core.convexhull_3d(mesh.points))
	hull.check()
	assert hull.isenvelope()
	assert hull.points is mesh.points        # buffers are shared, not copied
	assert len(hull.groups) == groups + 1    # a group is created for the unmatched faces
	assert max(hull.tracks) < groups         # ... and stays unused here
	# each face got the group of the side it belongs to
	for i, track in enumerate(hull.tracks):
		assert distance2(hull.facenormal(i), normals[track]) <= NUMPREC*8

	# same cube with all its sides merged into one group: planarity still separates the ngons,
	# so every face is restored to that unique group
	mesh = brick(width=vec3(2))
	mesh.tracks = typedlist([0]*len(mesh.faces), dtype='I')
	mesh.groups = [None]
	mesh.strippoints()
	hull = restore_groups_ngons(mesh, core.convexhull_3d(mesh.points))
	hull.check()
	assert set(hull.tracks) == {0}
	assert len(hull.groups) == 2             # created group unused

	# failing case: one corner is moved, so the sides touching it are no longer flat.
	# their ngons no longer match and those faces fall into the created group
	mesh = brick(width=vec3(2))
	mesh.points[0] = mesh.points[0] + vec3(0.3, 0.2, 0.4)
	mesh.strippoints()
	created = len(mesh.groups)
	hull = restore_groups_ngons(mesh, core.convexhull_3d(mesh.points))
	hull.check()
	assert created in hull.tracks            # some faces could not be matched
	assert set(hull.tracks) != {created}     # but the untouched sides were restored
