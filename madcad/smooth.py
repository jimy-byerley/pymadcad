# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module provides surface smoothing operations: moving a selection of points of a mesh to
	minimize the curvature variations of the surface, leaving the remaining points untouched.

	This is meant to relax a patch obtained from a coarse or irregular process (subdivision,
	boolean, reconstruction, ...) without moving the surface it is connected to.
'''

from __future__ import annotations

import numpy as np
from scipy import sparse

from .mathutils import vec3, typedlist
from .mesh import Mesh, numpy_to_typedlist, typedlist_to_numpy

__all__ = ['smooth', 'laplacian_matrix']


def smooth(mesh: Mesh, points: typedlist['I'], tangent: bool = True, weight: str = 'cotan') -> Mesh:
	''' move the given points of the mesh to minimize the curvature variations of the surface

		the points of the mesh not designated are left at their position, so the smoothed patch
		keeps matching the surrounding surface, in position as well as in tangent.

		Parameters:
			mesh:    the surface to smooth
			points:  indices of the points allowed to move
			tangent:
				if True, the patch matches the tangent of the surrounding surface at its border,
				canceling the curvature variations inside.

				if False, the tangency term is dropped and the curvature itself is canceled: the
				patch becomes a minimal surface, matching its border in position only, hence
				creasing there. a wide patch then collapses toward its border instead of keeping
				the shape of the surroundings
			weight:  weighting of the neighbors in the laplacian, see `laplacian_matrix`

		Example:
			>>> # round the corner of a cube, keeping its bottom untouched
			>>> cube = brick(size=vec3(2)).subdivide_to(0.2)
			>>> patch = cube.group({0,1,2})
			>>> border = {i  for edge in patch.outlines().edges  for i in edge}
			>>> smooth(cube, [i  for face in patch.faces  for i in face  if i not in border])
			<Mesh ...>

		Note:
			with the default cotan weights the operator depends on the shape of the input surface,
			so this solves for the surface the input describes and not for its own result: calling
			it again on its result gives a different surface. this is meant to be called once,
			repeating it does not refine the result but slowly deflates the patch and slides its
			points along the surface.
			with `weight='uniform'` the operator only depends on the topology, so it solves for
			its own result as well and a second call changes nothing
	'''
	moving = np.unique(np.asarray(points, dtype='u4'))
	if not len(moving):
		return mesh.own(points=True)
	if moving[-1] >= len(mesh.points):
		raise IndexError('point {} is out of the mesh'.format(moving[-1]))

	laplacian = laplacian_matrix(mesh, weight)
	# such a point has a null row, hence no equation to solve it
	orphans = moving[laplacian.diagonal()[moving] == 0]
	if len(orphans):
		raise ValueError('point {} has no laplacian: it belongs to no face, or its neighborhood '
			'is degenerated'.format(orphans[0]))

	# enforce laplacian * laplacian * surface = 0, which is the convergence limit of the laplacian
	# smooth. the second laplacian is what propagates the tangent of the surroundings into the
	# patch, so it must be applied to the whole mesh and not only to the patch: its rows for the
	# patch points reach the fixed points 2 edges away
	# without it, the equation is only laplacian * surface = 0, giving a minimal surface
	operator = (laplacian @ laplacian)  if tangent else  laplacian
	rows = operator.tocsr()[moving]
	system = rows[:, moving]
	# the remaining columns are the contribution of the fixed points, they form the constant term
	positions = typedlist_to_numpy(mesh.points, 'f8')
	constant = rows @ positions - system @ positions[moving]
	smoothed = sparse.linalg.splu(system.tocsc()).solve(-constant)

	# rebuild a mesh with the moved points, the other buffers are shared with the input mesh
	new = mesh.own(points=True)
	for index, moved in zip(moving, numpy_to_typedlist(smoothed, vec3)):
		new.points[int(index)] = moved
	return new


def laplacian_matrix(mesh: Mesh, weight: str = 'cotan') -> sparse.csr_matrix:
	''' laplacian operator of the surface

		for a mesh of `n` points, this is the `(n,n)` sparse matrix such as, `x` being the `(n,3)`
		array of the mesh point positions, `laplacian_matrix(mesh) @ x` is the laplacian of the
		surface at each point

		whatever the weighting, each row sums to zero, so the operator is insensitive to a
		translation of the surface. the row of a point belonging to no face is left null

		Parameters:
			weight:
				'cotan':
					discretization of the laplace-beltrami operator: each neighbor is weighted by
					the angles the surface opposes to their common edge, and each row is divided
					by the area the surface gives to its point. this only depends on the shape of
					the surface, not on the way it is triangulated, and its magnitude is a
					curvature, homogeneous to the inverse of a squared length

				'uniform':
					all the neighbors weight the same and each row is divided by their number, so
					the result is the difference between the point and the mean of its neighbors
					(umbrella operator). cheaper, dimensionless, and immune to the degenerated
					triangles a cotan weight would give a negative weight to, but biased by the
					triangulation: on a grid of quads split into triangles, each point gets 2 more
					neighbors in the direction of the diagonals, and the result is dragged that way
	'''
	n = len(mesh.points)
	faces = typedlist_to_numpy(mesh.faces, 'u4')
	points = typedlist_to_numpy(mesh.points, 'f8')
	# the 2 vectors leaving each corner of each face
	first = points[faces[:, [1,2,0]]] - points[faces]
	second = points[faces[:, [2,0,1]]] - points[faces]
	# the norm of the cross product is twice the area of the face, the same at its 3 corners
	doubled = np.linalg.norm(np.cross(first[:,0], second[:,0]), axis=-1)[:,None]

	if weight == 'uniform':
		weights = np.ones((len(faces), 3))
	elif weight == 'cotan':
		# the contribution of a face to the weight of the edge it opposes to each corner, so that
		# an inner edge ends with the usual `(cot(a) + cot(b))/2`.  `cotan = cos/sin = dot/cross`
		weights = 0.5 * np.divide(
			np.einsum('fkc,fkc->fk', first, second), doubled,
			out = np.zeros((len(faces), 3)),
			where = doubled != 0,
			)
	else:
		raise ValueError("weight must be 'cotan' or 'uniform', got {}".format(repr(weight)))

	# `weights[:,k]` is the weight of the edge opposite to the corner `k` of each face, deposited
	# in both directions so the matrix stays symmetric. an edge shared by 2 faces gets the
	# contribution of both, duplicate entries being summed by the matrix constructor
	rows, columns, values = [], [], []
	for k in range(3):
		rows.extend((faces[:,k-1], faces[:,k-2]))
		columns.extend((faces[:,k-2], faces[:,k-1]))
		values.extend((weights[:,k], weights[:,k]))
	stiffness = sparse.csr_matrix(
		(np.concatenate(values), (np.concatenate(rows), np.concatenate(columns))),
		shape = (n, n),
		)
	# the diagonal must cancel the neighbors weights whatever the topology, including at the
	# border of an open mesh where an edge belongs to one face only
	degrees = np.asarray(stiffness.sum(axis=1)).ravel()
	stiffness -= sparse.diags(degrees)

	if weight == 'uniform':
		# no geometry involved, the neighbors count is the only sensible scale
		masses = degrees
	else:
		# lumped mass matrix: the area a point is responsible for is a third of the area of each
		# of its faces
		masses = np.bincount(faces.ravel(), np.repeat(doubled.ravel()/6, 3), minlength=n)
	scale = np.divide(1, masses, out=np.zeros(n), where=masses != 0)
	return sparse.diags(scale) @ stiffness
