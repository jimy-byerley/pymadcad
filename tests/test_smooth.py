import numpy as np

from madcad import *
from madcad.mesh import typedlist_to_numpy
from madcad.smooth import smooth, laplacian_matrix
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
		dense = source.subdivide_to(size)
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
	dense = brick(size=vec3(2)).subdivide_to(0.2)
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
	dense = cone(3*Z, O, 2).subdivide_to(0.4)
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
