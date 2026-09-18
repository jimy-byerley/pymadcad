# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module provides surface smoothing operations: moving a selection of points of a mesh to
	minimize the curvature variations of the surface, leaving the remaining points untouched.

	This is meant to relax a patch obtained from a coarse or irregular process (subdivision,
	boolean, reconstruction, ...) without moving the surface it is connected to.
'''

from __future__ import annotations

from collections import Counter

import numpy as np
from scipy import sparse

from .mathutils import (
		vec3, uvec2, uvec3, uvec4, typedlist, NUMPREC,
		dot, cross, mix, length, length2, distance2,
	)
from .hashing import edgekey, arrangeface, connef
from .mesh import Mesh, mktri, mkquad, numpy_to_typedlist, typedlist_to_numpy

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


def subdivide(mesh: Mesh, div=1) -> Mesh:
	''' Subdivide all edges by the number of cuts '''
	n = div+2
	pts = typedlist(dtype=vec3)
	faces = typedlist(dtype=uvec3)
	tracks = typedlist(dtype='I')
	c = 0
	for f,t in enumerate(mesh.tracks):
		# place the points
		o,p0,p1 = mesh.facepoints(f)
		x = p0-o
		y = p1-o
		for i in range(n):
			u = i/(n-1)
			for j in range(n-i):
				v = j/(n-1)
				p = o + u*x + v*y
				pts.append(p)
		# create the faces
		for i in reversed(range(1,n+1)):
			for j in range(i-1):
				s = c+j
				faces.append(uvec3(s, s+i, s+1))
			for j in range(1,i-1):
				s = c+j
				faces.append(uvec3(s, s+i-1, s+i))
			c += i
		tracks.extend([t] * (len(faces)-len(tracks)))

	new = Mesh(pts, faces, tracks, mesh.groups)
	new.mergeclose()
	return new

	
	
	
def subdivide_to(mesh: Mesh, size: float, faces=None) -> Mesh:
	''' subdivide the faces of the mesh until no edge is bigger than the given size

		Parameters:
			size:  maximum length of an edge of the result
			faces:
				indices of the faces to subdivide, `None` meaning all of them

				the faces not designated are left untouched, except those sharing an edge with the
				subdivided ones: those are cut along that edge only, following the points inserted
				there, so the surface stays an envelope instead of getting T-junctions

		Example:
			>>> # refine only the top of a cube, the surrounding faces follow
			>>> cube = brick(size=vec3(2))
			>>> subdivide_to(cube, 0.2, [i  for i, f in enumerate(cube.faces)
			...                          if cube.facenormal(f).z > 0.5])
			<Mesh ...>
	'''
	if not size > 0:
		raise ValueError('size must be positive')
	size2 = size**2
	prec = NUMPREC*8
	points = mesh.points[:]

	if faces is None:
		selected = set(range(len(mesh.faces)))
	else:
		selected = set(map(int, faces))
		if selected and not (0 <= min(selected) and max(selected) < len(mesh.faces)):
			raise IndexError('face index out of the mesh')
	# the sides of the region to subdivide
	inside = {edgekey(mesh.faces[i][t-1], mesh.faces[i][t])  for i in selected  for t in range(3)}
	# the sides a non selected face shares with it, and the faces to cut along
	border = set()
	frontier = []
	kept = []
	for i, face in enumerate(mesh.faces):
		if i in selected:
			continue
		shared = {edgekey(face[t-1], face[t])  for t in range(3)} & inside
		if shared:
			border |= shared
			frontier.append(i)
		# untouched: `border` only grows by splitting a side it already holds, so such a face can
		# never become concerned later
		else:
			kept.append(i)

	midpoints = {}
	def midpoint(a: int, b: int) -> int:
		key = edgekey(a, b)
		if key not in midpoints:
			midpoints[key] = len(points)
			points.append(mix(points[a], points[b], 0.5))
		# cutting a border side gives 2 border sides. this must happen even when the point already
		# exists, since the subdivided side usually creates it first
		if key in border:
			border.add(edgekey(a, midpoints[key]))
			border.add(edgekey(midpoints[key], b))
		return midpoints[key]

	# make a quad mesh of the region to subdivide, the faces at its border joining as triangles
	current = triangles_to_quads(mesh, selected)
	# whether each quad of `current` can be subdivided on any of its sides, or only on its border
	# sides. this is subdivision state, hence kept aside of the surface
	current_free = [True] * len(current.quads)
	for i in frontier:
		face = mesh.faces[i]
		current.add_quad(uvec4(face[0], face[1], face[2], face[0]), mesh.tracks[i])
		current_free.append(False)
	final = QuadSurface(points, groups=mesh.groups)
	# subdivide as possible
	while current.quads:
		new = QuadSurface(points, groups=mesh.groups)
		new_free = []
		def add_new(quad: uvec4, track: int, free: bool):
			new.add_quad(quad, track)
			new_free.append(free)
		for quad, track, free in zip(current.quads, current.tracks, current_free):
			lengths = [distance2(points[quad[i]], points[quad[i-3]])  for i in range(len(quad))]
			if not free:
				# a non selected face is only cut on the sides it shares with the subdivided
				# region, so it follows the points inserted there and keeps its interior untouched
				lengths = [l  if edgekey(quad[i-3], quad[i]) in border else 0
					for i, l in enumerate(lengths)]
			order = sorted(range(len(quad)), key=lengths.__getitem__)

			# size reached, remove face from processing
			if lengths[order[3]] <= size2:
				final.add_quad(quad, track)

			# one cut
			elif lengths[order[3]] > size2 and (lengths[order[3]] > 2*lengths[order[2]]  or lengths[order[2]] <= size2):
			# elif lengths[order[2]] <= size2:
				k = order[3]
				split = midpoint(quad[k-3], quad[k-0])
				# choose which side is a triangle and which one is a quad
				# a null side must end in the piece keeping 3 corners, so both pieces stay
				# degenerated quads. else that piece has 3 aligned corners instead of a null side,
				# and further cuts tear it apart into null area triangles
				if quad[k-3] == quad[k-2]:
					triangle = True
				elif quad[k-2] == quad[k-1] or quad[k-1] == quad[k-0]:
					triangle = False
				else:
					triangle = distance2(points[split], points[quad[k-1]]) < distance2(points[split], points[quad[k-2]])
				if triangle:
					add_new(uvec4(split, quad[k-3], quad[k-2], quad[k-1]), track, free)
					add_new(uvec4(split, quad[k-1], quad[k-0], split), track, free)
				else:
					add_new(uvec4(split, quad[k-2], quad[k-1], quad[k-0]), track, free)
					add_new(uvec4(split, quad[k-3], quad[k-2], split), track, free)
			
			# two cuts
			elif lengths[order[2]] > size2:
				# adjacent sides
				if abs(order[2] - order[3]) % 2 == 1:
					k = order[3] if (order[3]-1) % 4 == order[2] else order[2]
					split1 = midpoint(quad[k-0], quad[k-3])
					split2 = midpoint(quad[k-1], quad[k-0])
					add_new(uvec4(quad[k-0], split1, split2, quad[k-0]), track, free)
					add_new(uvec4(quad[k-1], split2, split1, quad[k-3]), track, free)
					add_new(uvec4(quad[k-3], quad[k-2], quad[k-2], quad[k-1]), track, free)
				# opposite sides
				else:
					k = order[3]
					split1 = midpoint(quad[k-2], quad[k-1])
					split2 = midpoint(quad[k-0], quad[k-3])
					add_new(uvec4(split1, split2, quad[k-3], quad[k-2]), track, free)
					add_new(uvec4(split2, split1, quad[k-1], quad[k-0]), track, free)
			
			else:
				raise AssertionError("should not be reached, internal error")
		
		# prepare next iteration
		current = new
		current_free = new_free

	result = quads_to_triangles(final)
	# the faces neither subdivided nor at the border of the subdivided region are propagated as is
	for i in kept:
		result.faces.append(mesh.faces[i])
		result.tracks.append(mesh.tracks[i])
	return result

class QuadSurface:
	points: typedlist[vec3]
	quads: typedlist[uvec4]
	tracks: typedlist['I']
	groups: list[dict]

	def __init__(self, points=None, quads=None, tracks=None, groups=None):
		if points is None:
			points = typedlist(dtype=vec3)
		if quads is None:
			quads = typedlist(dtype=uvec4)
		if tracks is None:
			tracks = typedlist(dtype='I')
		if groups is None:
			groups = []
		self.points = points
		self.quads = quads
		self.tracks = tracks
		self.groups = groups

	def add_quad(self, quad: uvec4, track: int):
		self.quads.append(quad)
		self.tracks.append(track)


def triangles_to_quads(triangles: Mesh, selected: set = None) -> QuadSurface:
	''' pair the coplanar faces of the mesh into quads, the lone ones becoming degenerated quads

		only the designated faces are considered, the others are simply ignored
	'''
	prec = NUMPREC*8
	points = triangles.points
	quadrangles = QuadSurface(points, groups=triangles.groups)
	adjacency = connef(triangles.faces)
	paired = [False]*len(triangles.faces)
	for itri, (tri, track) in enumerate(zip(triangles.faces, triangles.tracks)):
		# this one has already been taken by a previous face, or is not to subdivide
		if paired[itri] or (selected is not None and itri not in selected):
			continue
		for i in range(len(tri)):
			# the adjacent face crosses the shared edge the other way around
			iadjacent = adjacency.get((tri[i-1], tri[i-2]))
			# triangles must not be paired, and must both be subdivided
			if iadjacent is None or paired[iadjacent] or triangles.tracks[iadjacent] != track:
				continue
			if selected is not None and iadjacent not in selected:
				continue
			adjacent = triangles.faces[iadjacent]
			# triangles must be coplanar
			normal_face = triangles.facenormal(tri)
			normal_adjacent = triangles.facenormal(adjacent)
			if dot(normal_face, normal_adjacent) < 1 - prec:
				continue
			# contour of the pair of faces, its diagonal is `(quad[0], quad[2])`
			quad = uvec4(tri[i-2], arrangeface(adjacent, tri[i-1])[2], tri[i-1], tri[i])
			normal = normal_face + normal_adjacent
			# the diagonal can only be moved in a convex contour
			if any(dot(normal, cross(
						points[quad[t-1]] - points[quad[t-2]],
						points[quad[t]] - points[quad[t-1]] )) <= 0
					for t in range(4)):
				continue
			quadrangles.add_quad(quad, track)
			paired[itri] = True
			paired[iadjacent] = True
			break
	for itri, (tri, track) in enumerate(zip(triangles.faces, triangles.tracks)):
		if paired[itri] or (selected is not None and itri not in selected):
			continue
		quadrangles.add_quad(uvec4(tri[0], tri[1], tri[2], tri[0]), track)
	return quadrangles

def quads_to_triangles(quads: QuadSurface) -> Mesh:
	prec = NUMPREC*8
	points = quads.points
	triangles = Mesh(points=points, groups=quads.groups)
	def add_tri(tri: uvec3, track: int):
		if any(tri[i-1] == tri[i]  for i in range(len(tri))):
			return
		triangles.faces.append(tri)
		triangles.tracks.append(track)
	for quad, track in zip(quads.quads, quads.tracks):
		# a degenerated quad is already a triangle, it must be propagated as is and not rotated
		degenerated = next((i  for i in range(len(quad))  if quad[i-1] == quad[i]), None)
		if degenerated is not None:
			add_tri(uvec3(quad[degenerated], quad[degenerated-3], quad[degenerated-2]), track)
		elif distance2(points[quad[0]], points[quad[2]]) < distance2(points[quad[1]], points[quad[3]]):
			add_tri(uvec3(quad[0], quad[1], quad[2]), track)
			add_tri(uvec3(quad[2], quad[3], quad[0]), track)
		else:
			add_tri(uvec3(quad[1], quad[2], quad[3]), track)
			add_tri(uvec3(quad[3], quad[0], quad[1]), track)
	return triangles
