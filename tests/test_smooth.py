import numpy as np

from madcad import *
from copy import deepcopy
from random import random, seed as random_seed

from madcad.mesh import typedlist_to_numpy, Mesh
from madcad.hashing import edgekey
from madcad.smooth import smooth, laplacian_matrix, subdivide_to
from . import visualcheck


@visualcheck
def test_smooth():
	results = []

	for source, size, criterion in [
			# half of a cube: the patch has sharp edges inside, they must be rounded
			(brick(size=vec3(2)), 0.2, lambda normal, center: dot(normal, vec3(1)) > 0),
			(brick(size=vec3(2)), 0.2, lambda normal, center: dot(normal, Z) > 0.5),
			(brick(size=vec3(2)), 0.2, lambda normal, center: center.z > 0.4),
			# the vertical faces of a cube: the patch is a band all around the surface, so its
			# border is 2 separate loops and the result must be a barrel
			(brick(size=vec3(2)), 0.2, lambda normal, center: abs(dot(normal, Z)) < 0.5),
			# only a trunk of the vertical faces: the patch border is in the middle of the flat
			# faces instead of on an edge of the cube, so the rounded part must blend back into
			# the sharp cube above and below
			(brick(size=vec3(2)), 0.2, lambda normal, center: abs(dot(normal, Z)) < 0.5 and abs(center.z) < 0.6),
			(brick(size=vec3(2, 2, 6)), 0.4, lambda normal, center: abs(dot(normal, Z)) < 0.5 and abs(center.z) < 2.5),
			# the patch is already smooth, it only gets slightly flatter: a sphere has a constant
			# curvature but not a null curvature variation, so it is not a solution here
			(icosphere(O, 1, resolution=('div', 2)), 0.2, lambda normal, center: dot(normal, vec3(1)) > 0),
			(icosphere(O, 1, resolution=('div', 2)), 0.2, lambda normal, center: abs(center.z) < 0.5),
			]:
		# subdivide to give the patch interior points to move
		# the whole surface is subdivided and not only the smoothed half, else the coarse half
		# would keep the long edges the dense one splits, leaving T-junctions in between
		dense = subdivide_to(source, size)
		dense.check()
		assert dense.isenvelope()

		# smooth only a part of the surface, the untouched faces hold the patch border in place
		patch = selection(dense, criterion)
		moving = interior(submesh(dense, patch))
		assert len(moving)
		smoothed = smooth(dense, moving)
		smoothed.check()
		assert smoothed.isenvelope()

		# cotan weights depend on the positions, so the operator solved for is the one of the
		# input surface, not the one the result would give
		laplacian = laplacian_matrix(dense)
		bilaplacian = laplacian @ laplacian
		# the curvature variation is canceled over the patch, which is what smooth() solves for
		assert residual(bilaplacian, dense, moving) > 1e-3
		assert residual(bilaplacian, smoothed, moving) < 1e-5
		# the curvature itself is not, the patch is not a minimal surface
		assert residual(laplacian, smoothed, moving) > 1e-3
		# dropping the tangency term cancels the curvature instead of its variation
		membrane = smooth(dense, moving, tangent=False)
		membrane.check()
		assert residual(laplacian, membrane, moving) < 1e-5
		assert residual(bilaplacian, membrane, moving) > 1e-3
		# the points out of the patch did not move
		fixed = set(range(len(dense.points))) - set(moving)
		assert all(smoothed.points[i] == dense.points[i]  for i in fixed)
		# with a purely topological weighting the system does not depend on its own result, so
		# smoothing an already smoothed patch changes nothing
		umbrella = smooth(dense, moving, weight='uniform')
		assert max(distance(a, b)
			for a, b in zip(smooth(umbrella, moving, weight='uniform').points, umbrella.points)) < 1e-5
		# the result depends on the shape of the surface, not on its place
		assert max(distance(a, b - 10*X)
			for a, b in zip(smoothed.points, smooth(dense.transform(10*X), moving).points)) < 1e-4
		# the patch is only relaxed, no face is folded over its neighbors
		assert all(dot(smoothed.facenormal(f), dense.facenormal(f)) > 0
			for f in range(len(dense.faces)))

		# move the smoothed patch to its own group, to tell it apart from the untouched surface
		smoothed = regroup(smoothed, patch)
		membrane = regroup(membrane, patch)
		assert set(smoothed.group('smooth').faces) == set(submesh(dense, patch).faces)

		results.append((smoothed, membrane))

	# one column per case, `tangent=True` on the first row and `tangent=False` on the second
	layout = []
	x = 0
	for smoothed, membrane in results:
		size = smoothed.box().size
		layout.append(smoothed.transform((x+size.x)*X))
		layout.append(membrane.transform((x+size.x)*X + 1.5*size.y*Y))
		x += 1.5*size.x

	return layout


def test_smooth_isotropy():
	''' the result must not depend on the direction the quads are split into triangles '''
	dense = subdivide_to(brick(size=vec3(2)), 0.2)
	moving = interior(submesh(dense, selection(dense, lambda normal, center: abs(dot(normal, Z)) < 0.5)))
	# the barrel has a 4-fold symmetry around Z, the triangulation of its faces has not
	bulges = lambda mesh:  [max(dot(mesh.points[i], d)  for i in moving)  for d in (X, Y, -X, -Y)]

	# the symmetry is only limited by the conditioning of the system, not by the operator
	cotan = bulges(smooth(dense, moving))
	assert max(cotan) - min(cotan) < 1e-6
	# the umbrella weighting gives each point 2 more neighbors along the diagonals of the quads,
	# which drags the result that way
	uniform = bulges(smooth(dense, moving, weight='uniform'))
	assert max(uniform) - min(uniform) > 1e-2


def test_smooth_grading():
	''' the result must not depend on the density of the triangulation '''
	# the triangles of a cone shrink toward its apex, giving a strongly graded mesh
	dense = subdivide_to(cone(3*Z, O, 2), 0.4)
	moving = interior(submesh(dense, selection(dense,
		lambda normal, center: abs(dot(normal, Z)) < 0.9 and 0.4 < center.z < 2.6)))
	smoothed = smooth(dense, moving)
	# the patch must stay close to the straight profile of the cone despite the density variation
	# a laplacian normalized by the neighbors weights instead of by the surface area gives 0.19
	assert max(abs(length(smoothed.points[i]*vec3(1,1,0)) - 2*(1 - smoothed.points[i].z/3))
		for i in moving) < 0.1


def selection(mesh, criterion) -> list:
	''' indices of the faces of the mesh whose normal and center satisfy the criterion '''
	return [i  for i, face in enumerate(mesh.faces)
		if criterion(mesh.facenormal(face), sum(mesh.facepoints(face), vec3(0))/3)]

def submesh(mesh, faces) -> Mesh:
	''' the part of the mesh made of the given face indices '''
	return Mesh(
		mesh.points,
		typedlist((mesh.faces[i]  for i in faces), uvec3),
		typedlist((mesh.tracks[i]  for i in faces), 'I'),
		mesh.groups,
		)

def interior(patch) -> typedlist:
	''' indices of the points of the patch that are not on its border '''
	border = {i  for edge in patch.outlines().edges  for i in edge}
	return typedlist(sorted({i  for face in patch.faces  for i in face} - border), 'I')

def regroup(mesh, faces) -> Mesh:
	''' copy of the mesh with the given faces moved to a new group '''
	new = mesh.own(tracks=True, groups=True)
	new.groups.append({'smooth': None})
	for i in faces:
		new.tracks[i] = len(new.groups)-1
	return new

def residual(operator, mesh, points) -> float:
	''' biggest value of the operator applied to the surface, over the given points '''
	return np.abs((operator @ typedlist_to_numpy(mesh.points, 'f8'))[list(points)]).max()

@visualcheck
def test_subdivide_to():
	cases = [
		# regular faces, all edges subdivided at once
		(icosahedron(O, 1), 0.6),
		(brick(width=vec3(1)), 0.4),
		# flat but elongated faces, the longest edges are subdivided first
		(brick(width=vec3(4, 1, 0.2)), 0.6),
		(parallelogram(4*X, 0.3*Y), 0.5),
		# coarse and fine surfaces mixed, the subdivision must stay local
		(icosahedron(O, 1) + icosphere(4*X, 1), 0.5),
		# slivers gathering at the summit and at the poles
		(cone(2*Z, O, 1), 0.15),
		(uvsphere(O, 1), 0.15),
		# flat facets and curved surface mixed
		(cylinder(O, 3*Z, 1), 0.5),
		# an open curved patch, its border must only be subdivided
		(submesh(uvsphere(O, 1), selection(uvsphere(O, 1), lambda normal, center: center.z > 0)), 0.3),
		]
	results = []
	# each case is a column, each subdivision size is a row
	rows = max(obj.box().width.y  for obj, size in cases) + 1
	place = 0
	for obj, size in cases:
		npoints = len(obj.points)
		for row, target in enumerate((size, size/3)):
			div = subdivide_to(obj, target)
			div.check()
			assert div.issurface()
			# the input must be left untouched
			assert len(obj.points) == npoints
			# no quad side above the target size, the diagonal a quad is triangulated with
			# being an artifact of the triangulation, it can reach `target*sqrt(2)`
			assert max(distance2(div.points[a], div.points[b])  for a,b in div.edges()) <= 2*target**2
			# no face lost nor overlapping
			assert abs(div.surface() - obj.surface()) <= 1e-9 * obj.surface()
			# the outline is only subdivided, so there is no T-junction
			assert div.isenvelope() == obj.isenvelope()
			assert abs(div.outlines().length() - obj.outlines().length()) <= 1e-9 * (obj.outlines().length() or 1)
			# both faces sharing an edge reused the same midpoint, so there is nothing to merge
			# (points unused by the input are simply propagated, hence the strip)
			stripped = deepcopy(div)
			stripped.strippoints()
			assert not stripped.mergeclose()
			# groups are propagated to the new faces
			assert div.groups is obj.groups
			assert set(div.tracks) == set(obj.tracks)

			# spread the results along X, whatever their own dimensions
			box = obj.box()
			results.append(div.transform((place - box.min.x + 0.5)*X + row*rows*Y))
		place += box.width.x + 1

	return results

def test_subdivide_to_flat():
	# an elongated flat quad must be subdivided across its length, dropping its diagonal instead
	# of subdividing it, otherwise the diagonal degenerates into slivers
	for width, size in [(Y, 1.1), (0.3*Y, 0.5)]:
		flat = parallelogram(4*X, width)
		div = subdivide_to(flat, size)
		div.check()
		assert abs(div.surface() - flat.surface()) <= 1e-9 * flat.surface()
		for face in div.faces:
			lengths = sorted(distance(div.points[face[t-2]], div.points[face[t-1]])  for t in range(3))
			assert lengths[2] <= 2*lengths[0], 'sliver face {} in {}'.format(face, div)

def test_subdivide_to_limits():
	obj = icosphere(O, 1)
	# nothing to subdivide, the mesh must be returned as is
	same = subdivide_to(obj, 10)
	assert same.faces == obj.faces
	assert same.tracks == obj.tracks
	# a needle triangle must converge instead of subdividing forever
	needle = Mesh(
		[vec3(0), vec3(10,0,0), vec3(0,0.01,0)],
		[uvec3(0,1,2)],
		)
	div = subdivide_to(needle, 0.5)
	div.check()
	assert max(distance2(div.points[a], div.points[b])  for a,b in div.edges()) <= 2*0.5**2
	assert abs(div.surface() - needle.surface()) <= 1e-9 * needle.surface()
	# a size must be given
	for size in (0, -1, nan):
		try:
			subdivide_to(obj, size)
		except ValueError:
			pass
		else:
			raise AssertionError('subdivide_to accepted size {}'.format(size))

def test_subdivide_to_templates():
	''' whatever the sides marked for subdivision, the template must fill its face exactly '''
	random_seed(0)
	for i in range(300):
		# a random convex quad in a random plane, given as its 2 triangles, and the triangle of
		# its 3 first corners. taking the corner angles sorted makes the contour convex
		place = mat4(quat(2*pi*random(), normalize(vec3(random(), random(), random()) - 0.5)))
		place[3] = vec4(vec3(random(), random(), random()) - 0.5, 1)
		angles = sorted(2*pi*random()  for t in range(4))
		corners = typedlist((
			vec3(place * vec4((0.3 + random())*vec3(cos(angle), sin(angle), 0), 1))
			for angle in angles), vec3)
		for faces in ([uvec3(0,1,2), uvec3(0,2,3)], [uvec3(0,1,2)]):
			obj = Mesh(corners, typedlist(faces, uvec3))
			size = (0.1 + random()) * max(obj.box().width)
			div = subdivide_to(obj, size)
			div.check()
			assert div.issurface()
			# the face is exactly covered, without overlap nor hole
			assert abs(div.surface() - obj.surface()) <= 1e-9 * obj.surface()
			# its border is only bisected, so a neighbor face would match whatever it decides
			assert abs(div.outlines().length() - obj.outlines().length()) <= 1e-9 * obj.outlines().length()
			assert max(distance2(div.points[a], div.points[b])  for a,b in div.edges()) <= 2*size**2
			stripped = deepcopy(div)
			stripped.strippoints()
			assert not stripped.mergeclose()

def test_subdivide_to_locality():
	# a coarse and a fine surface in the same mesh: the faces already fine enough must be left
	# untouched, not even retriangulated by a merge into quads
	obj = icosahedron(O, 1) + icosphere(4*X, 1)
	size = 0.5
	fine = {tuple(face)  for face in obj.faces
		if max(distance2(obj.points[face[t-2]], obj.points[face[t-1]])  for t in range(3)) <= size**2}
	assert fine
	assert fine <= {tuple(face)  for face in subdivide_to(obj, size).faces}

def test_subdivide_to_slivers():
	# subdividing quads rather than triangles preserves the aspect ratio of the faces: the
	# diagonals are dropped instead of subdivided, so they do not pile up in one direction
	for obj, size, expected in [
			(cylinder(O, 3*Z, 1), 0.167, 0.1),
			(cone(2*Z, O, 1), 0.05, 0.1),
			(uvsphere(O, 1), 0.05, 0.1),
			]:
		div = subdivide_to(obj, size)
		slivers = sum(1  for face in div.faces
			if max(lengths := [distance(div.points[face[t-2]], div.points[face[t-1]])  for t in range(3)])
				> 2*min(lengths))
		assert slivers <= expected * len(div.faces), '{} slivers in {} faces'.format(slivers, len(div.faces))

@visualcheck
def test_subdivide_to_selection():
	cases = [
		# a flat patch of a cube, its border faces are coarse and must follow
		(brick(size=vec3(2)), 0.3, lambda obj: selection(obj, lambda normal, center: dot(normal, Z) > 0.5)),
		# a band all around the surface, so the untouched parts are on both sides of the patch
		(brick(size=vec3(2)), 0.3, lambda obj: selection(obj, lambda normal, center: abs(dot(normal, Z)) < 0.5)),
		# a curved patch, its border faces are not coplanar with it
		(uvsphere(O, 1), 0.2, lambda obj: selection(obj, lambda normal, center: center.z > 0.5)),
		(icosphere(O, 1, resolution=('div', 1)), 0.2, lambda obj: selection(obj, lambda normal, center: center.z > 0.5)),
		# a single face in the middle of a coarse surface
		(icosahedron(O, 1), 0.2, lambda obj: [0]),
		]
	results = []
	place = 0
	propagated = 0
	for obj, size, criterion in cases:
		patch = criterion(obj)
		assert patch and len(patch) < len(obj.faces)
		# move the patch to its own group, to tell its faces apart in the result
		obj = regroup(obj, patch)
		group = len(obj.groups)-1
		npoints = len(obj.points)
		div = subdivide_to(obj, size, patch)
		div.check()
		assert div.issurface()
		# the input must be left untouched
		assert len(obj.points) == npoints
		# the border faces are only cut along the patch, so the surface is still closed
		assert div.isenvelope() == obj.isenvelope()
		assert abs(div.outlines().length() - obj.outlines().length()) <= 1e-9 * (obj.outlines().length() or 1)
		# no face lost nor overlapping
		assert abs(div.surface() - obj.surface()) <= 1e-9 * obj.surface()
		# both sides of a border edge reused the same midpoints, so there is nothing to merge
		stripped = deepcopy(div)
		stripped.strippoints()
		assert not stripped.mergeclose()
		# the target size is reached on the patch, and only there
		assert max(distance2(div.points[face[t-2]], div.points[face[t-1]])
			for face, track in zip(div.faces, div.tracks)  if track == group
			for t in range(3)) <= 2*size**2
		# a face neither selected nor sharing an edge with the selection is propagated as is
		sides = {edgekey(obj.faces[i][t-2], obj.faces[i][t-1])  for i in patch  for t in range(3)}
		untouched = {tuple(face)  for i, face in enumerate(obj.faces)  if i not in set(patch)
			and not any(edgekey(face[t-2], face[t-1]) in sides  for t in range(3))}
		assert untouched <= {tuple(face)  for face in div.faces}
		propagated += len(untouched)
		# groups are propagated to the new faces
		assert div.groups is obj.groups
		assert set(div.tracks) == set(obj.tracks)

		box = obj.box()
		results.append(div.transform((place - box.min.x + 0.5)*X))
		place += box.width.x + 1

	assert propagated
	return results

def test_subdivide_to_selection_limits():
	obj = icosphere(O, 1, resolution=('div', 1))
	size = 0.3
	# a selection of everything is the same as no selection at all
	whole = subdivide_to(obj, size, range(len(obj.faces)))
	assert whole.faces == subdivide_to(obj, size).faces
	# an empty selection leaves the mesh untouched
	same = subdivide_to(obj, size, [])
	assert same.faces == obj.faces
	assert same.tracks == obj.tracks
	# the faces must be designated by valid indices
	for faces in ([len(obj.faces)], [-1]):
		try:
			subdivide_to(obj, size, faces)
		except IndexError:
			pass
		else:
			raise AssertionError('subdivide_to accepted faces {}'.format(faces))

def test_subdivide_to_selection_templates():
	''' whatever the shape of the pair, the border face must exactly follow the subdivided one '''
	random_seed(0)
	for i in range(300):
		# a random convex quad in a random plane, given as its 2 triangles, only one subdivided
		place = mat4(quat(2*pi*random(), normalize(vec3(random(), random(), random()) - 0.5)))
		place[3] = vec4(vec3(random(), random(), random()) - 0.5, 1)
		angles = sorted(2*pi*random()  for t in range(4))
		corners = typedlist((
			vec3(place * vec4((0.3 + random())*vec3(cos(angle), sin(angle), 0), 1))
			for angle in angles), vec3)
		obj = Mesh(corners, typedlist([uvec3(0,1,2), uvec3(0,2,3)], uvec3), typedlist([0,1], 'I'), [{}, {}])
		size = (0.1 + random()) * max(obj.box().width)
		div = subdivide_to(obj, size, [0])
		div.check()
		assert div.issurface()
		# the pair is exactly covered, without overlap nor hole, and its border is left in place
		assert abs(div.surface() - obj.surface()) <= 1e-9 * obj.surface()
		assert abs(div.outlines().length() - obj.outlines().length()) <= 1e-9 * obj.outlines().length()
		# no T-junction between the subdivided face and the one following it
		stripped = deepcopy(div)
		stripped.strippoints()
		assert not stripped.mergeclose()
		# the target size is reached on the subdivided face only, the other one keeps its own sides
		assert max(distance2(div.points[face[t-2]], div.points[face[t-1]])
			for face, track in zip(div.faces, div.tracks)  if track == 0
			for t in range(3)) <= 2*size**2

def test_subdivide_to_nonmanifold():
	# 3 faces on the same edge: no pair can be merged there without opening a crack, so the
	# subdivision must fall back to triangles instead of dropping that edge
	obj = Mesh(
		[vec3(0), vec3(3,0,0), vec3(1,2,0), vec3(1,-2,0), vec3(1,0,2)],
		[uvec3(0,1,2), uvec3(1,0,3), uvec3(0,1,4)],
		)
	div = subdivide_to(obj, 0.5)
	div.check()
	assert abs(div.surface() - obj.surface()) <= 1e-9 * obj.surface()
	assert max(distance2(div.points[a], div.points[b])  for a,b in div.edges()) <= 2*0.5**2
