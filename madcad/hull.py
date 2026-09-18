# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module provides functions to compute convex hulls
	(See https://en.wikipedia.org/wiki/Convex_hull)
	and other hull-related operations for `Mesh` and `Web`

	Those can be very helpful to complete sketches or parts by adding the missing surface portions. Also very helpful to sort a set of directions.
'''

from __future__ import annotations
from . import core
from .mathutils import (
		vec3, vec2, vec4, mat4, dirbase, noproject, NUMPREC, uvec3, uvec2, transpose,
		dmat2x3, determinant, dot, length2, cross, normalize, isnan, glm, typedlist
	)
from .mesh import Mesh, Web, Wire
from .hashing import Asso, edgekey, facekeyo, connef
from .smooth import smooth, subdivide_to

from copy import copy
from collections import Counter, deque
from math import cos, sqrt



def simple_convexhull(points: typedlist[vec3]) -> typedlist[uvec3]:
	''' Just like `convexhull()` but minimalist.
		It does not take care of groups crossing. It doesn't return a new Mesh but a buffer of triangles indices
	'''
	return core.convexhull_3d(points)

def simple_convexoutline(points: typedlist[vec2]) -> typedlist[uvec2]:
	''' Just like `convexoutline()` but minimalist.
		It does not take care of groups crossing. It doesn't return a new Web but a buffer of edges indices
	'''
	return core.convexhull_2d(points)


def convexhull(source: typedlist[vec3]|Mesh|Web|Wire) -> Mesh:
	''' compute the convex hull of the input container

	![convexhull result](../screenshots/hull-convexhull.png)

	Parameters:
		source (typedlist/Web/Mesh):	the input container, if it is a Web or Mesh, their groups are kept in the output data

	Examples:
		>>> m = convexhull(mesh([
		... 	uvsphere(O, 1, alignment=X),
		... 	uvsphere(4*X, 2, alignment=X),
		... 	]))
	'''
	if isinstance(source, (Mesh,Web,Wire)):
		source = copy(source)
		source.strippoints()
		points = source.points
	else:
		points = typedlist(source, dtype=vec3)
		source = None

	# no orient() needed, simple_convexhull() already returns outward oriented triangles
	return restore_groups(source, simple_convexhull(points))


def convexoutline(source: typedlist[vec3]|Mesh|Web|Wire, normal: vec3=None, flatten: bool=False) -> Web:
	''' based on `convexhull()` but will extract the loop formed by the edges in the biggest planar projection of the convex hull

	![convexoutline result](../screenshots/hull-convexoutline.png)

	Parameters:
		source (typedlist/Web/Mesh):     the input container, if it is a Web or Mesh, their groups are kept in the output data
		normal:                          the projection normal to retreive the outline using `horizon()`, if None is given it will default to the direction in which the outlien surface is the biggest
		flatten:                         whether to project the outline points in its mean plane

	Examples:
		>>> convexoutline(web([
		... 	Circle(Axis(O,Z), 1),
		... 	Circle(Axis(4*X,Z), 2),
		... 	]))
	'''
	if isinstance(source, (Mesh,Web,Wire)):
		source = copy(source)
		source.strippoints()
		points = source.points
	else:
		points = typedlist(source, dtype=vec3)
		source = None

	direction = normal or widest_surface_direction(Mesh(source.points, simple_convexhull(points)))
	x, y, z = dirbase(direction)
	proj = transpose(dmat2x3(x, y))
	# no orient() needed, simple_convexoutline() already returns a counterclockwise loop in the (x,y) base
	outline = restore_groups(source, simple_convexoutline(typedlist(proj * p  for p in points)))
	if flatten:
		outline.strippoints()
		center = outline.barycenter()
		outline.points = typedlist(center + noproject(p-center, direction)   for p in outline.points)
	return outline


def restore_groups(source, indices) -> Mesh:
	'''
	Create a mesh contaning the given simplices, associated with groups created from group crossing from the source mesh.
	The simplices must refer to points in `source`.
	The created groups are tuples of indices of the groups touched by each simplex.

   	Parameters:
  		source (Mesh/Web):    the mesh containing reference triangles and groups
  		indices:              a buffer of simplices indices (depending on the dimensionnality of `source`) we want to find groups for
	'''
	groups = source.groups
	tracks = typedlist.full(len(groups), len(indices), dtype='I')

	if isinstance(source, Mesh):
		former = { facekeyo(*face): track
						for face, track in zip(source.faces, source.tracks)}
	elif isinstance(source, Web):
		former = { edge: track
						for edge, track in zip(source.edges, source.tracks)}
	else:
		raise TypeError('expected a Mesh or Web')

	# create an associative dictionnary where associated tracks only appears one per point
	combiner = Asso()
	for simplex, track in former.items():
		for p in simplex:
			if track not in combiner[p]:
				combiner.add(p, track)
	# search for original group or new group at each simplex
	groupindex = {}
	for i, simplex in enumerate(indices):
		belong = Counter()
		for p in simplex:
			belong.update(combiner[p])
		# get a shortlist of the groups reaching this simplex through its points
		couple = sorted(belong, key=lambda i: belong[i], reverse=True)
		shortlist = []
		amount = 0
		while amount < len(simplex):
			n = couple[len(shortlist)]
			shortlist.append(n)
			amount += belong[n]
		# one group only
		if len(shortlist) == 1:
			tracks[i] = couple[0]
		else:
			combine = tuple(sorted(shortlist))
			if combine not in groupindex:
				groupindex[combine] = len(groups)
				groups.append(dict())
			tracks[i] = groupindex[combine]

	outdim = len(simplex)
	if outdim == 3:
		return Mesh(source.points, indices, tracks, groups)
	elif outdim == 2:
		return Web(source.points, indices, tracks, groups)
	else:
		raise TypeError('outer simplex dimension {} is not supported'.format(outdim))


def horizon(mesh, direction: vec3) -> Web:
	'''
	Return a Web of the ORIENTED edges of the given mesh that lay between triangles that are oriented either sides of `direction`

    ![horizon result](../screenshots/hull-horizon.png)
	'''
	horizon = Web(points=mesh.points, groups=mesh.groups)
	signs = {}	# dictionnary for crossing face directions around edges
	groups = {} # track of the positive face connected for each edge
	for face, track in zip(mesh.faces, mesh.tracks):
		# pick the sign of the projected surface
		s = dot(mesh.facenormal(face), direction)
		if abs(s) <= NUMPREC:	s = 0
		elif s > 0:				s = 1
		else:					s = -1

		for i in range(3):
			edge = (face[i], face[i-2])
			key = edgekey(*edge)
			# pick the group of the positive face neighboring the edge
			if s > 0 or (s >= 0 and key not in groups):
				groups[key] = track
			# the edge belong to the horizon if the neigboring faces are each appart the projection plane
			if edge in signs and signs[edge] * s <= 0 and max(signs[edge], s) == 1:
				horizon.edges.append(edge if s > 0 else flipped(edge))
				horizon.tracks.append(groups[key])
				del signs[edge]
				del groups[key]
			# if matching edge not found yet, index the current face
			else:
				signs[flipped(edge)] = s
	return horizon

def summit(mesh, direction: vec3) -> vec3:
	''' max point along the given direction '''
	index = max(
        (point  for face in mesh.faces for point in face),
        key = lambda point:  dot(mesh.points[point], direction)
        )
	return mesh.points[index]



def flipped(simplex):
	if len(simplex) == 2:
		return (simplex[1], simplex[0])
	elif len(simplex) == 3:
		return (simplex[2], simplex[1], simplex[2])
	else:
		raise TypeError('expected and edge or a triangle')


def widest_surface_direction(mesh) -> vec3:
	# find a direction not orthogonal to any of the mesh faces
	normals = mesh.facenormals()
	directionmap = simple_convexhull(typedlist(p  for p in normals if not glm.any(isnan(p))))
	score = 0
	proj = None
	for face in directionmap:
		surf = length2(cross(normals[face[1]]-normals[face[0]], normals[face[2]]-normals[face[0]]))
		if surf >= score:
			score, proj = surf, normals[face[0]] + normals[face[1]] + normals[face[2]]

	# project half of the mesh surface
	direction = vec3(0)
	for face in mesh.faces:
		surf = cross(mesh.points[face[1]]-mesh.points[face[0]], mesh.points[face[2]]-mesh.points[face[0]])
		if dot(surf, proj) > 0:
			direction += surf

	# select the average surface direction
	return normalize(direction)
	
	

def orient3d(a: vec3, b: vec3, c: vec3, d: vec3) -> float:
	''' orientation predicate: signed volume of the tetrahedron `(a,b,c,d)`.
		positive when `d` is on the positive side of the oriented plane `(a,b,c)`
	'''
	return dot(cross(b-a, c-a), d-a)

def insphere(a: vec3, b: vec3, c: vec3, d: vec3, e: vec3) -> float:
	''' in-sphere predicate: tells whether `e` is inside the circumsphere of `(a,b,c,d)`.
		its sign is meaningful relative to the tet orientation: `e` is strictly inside when
		`insphere` and `orient3d` have opposite signs (see `is_delaunay`)
	'''
	# lift each point onto the paraboloid w = |.|², relative to e, and test the hyperplane side.
	# the 4 lifted points are given as columns instead of rows, the determinant being insensitive
	# to a transposition
	a, b, c, d = (p-e  for p in (a, b, c, d))
	return determinant(mat4(
		vec4(a, length2(a)),
		vec4(b, length2(b)),
		vec4(c, length2(c)),
		vec4(d, length2(d)),
		))

def is_delaunay(tet, points: typedlist[vec3]) -> bool:
	''' tell whether the tetrahedron `tet` (4 indices into `points`) satisfies the Delaunay
		empty-sphere criterion: no other point of `points` lies strictly inside its circumsphere.

		points exactly on the sphere (`insphere == 0`) are tolerated as valid, so cospherical
		degeneracies do not reject the tet. flat tets (`orient3d == 0`) are tolerated too.

		Note:
			this tests every point, so it is O(n) per call. it can be accelerated by indexing
			`points` in a `PositionMap` (see `hashing`) and querying only those near the
			circumsphere, the empty-sphere test being local.
	'''
	a, b, c, d = (points[i]  for i in tet)
	orient = orient3d(a, b, c, d)
	if orient == 0:
		return True   # flat tet: no circumsphere, defer to degeneracy handling
	inside = orient > 0
	tetset = set(tet)
	for i in range(len(points)):
		if i in tetset:
			continue
		s = insphere(a, b, c, d, points[i])
		# strictly inside <=> opposite sign to the orientation (and not exactly on the sphere)
		if s != 0 and (s > 0) != inside:
			return False
	return True


def _facenormal(points, f) -> vec3:
	return normalize(cross(points[f[1]]-points[f[0]], points[f[2]]-points[f[0]]))

def concavehull(mesh: Mesh, angle=0.2) -> Mesh:
	''' concave hull of a mesh: the convex hull dug into the concavities of the input.

		starting from the convex hull, the faces bridging over the concavities (the ones that are not
		faces of the input) are pushed inward one tetrahedron at a time. digging a bridge face reveals
		the input face sharing its inner edge and re-fans two new bridges from the same remote point
		(the bridge apex), so the angle measured against the bridge does not drift as the front
		advances. a face is revealed only when it stays within `angle` of the bridge it is dug from.
		the input faces are kept untouched, only the bridging faces are reworked, so the result can be
		refined and relaxed afterwards without moving the input surface.

		Parameters:
			angle:  maximum angle (radians) between a revealed concave face and the local bridge face
				it is dug from. small `angle` keeps the result close to the convex hull, large `angle`
				lets it follow deeper concavities. the revealed concave surface deviates from the
				bridging surface by at most `angle`.

		Note:
			remaining limitations:
				- a carve is refused when it would reuse a directed edge (wind two faces the same way or
				  pinch), which keeps the result a consistent 2-manifold in the common case. highly
				  symmetric inputs (e.g. two identical spheres) can still leave a few inconsistent edges
				  where fronts meet ambiguously, for want of a symbolic-perturbation tie-break
	'''
	# work in a single point indexing shared by the mesh and its hull
	mesh = copy(mesh)
	mesh.strippoints()
	hull = convexhull(mesh)
	points = hull.points

	orig = connef(mesh.faces)                                          # directed edge -> input face index
	origtrack = {facekeyo(*f): int(t)  for f, t in zip(mesh.faces, mesh.tracks)}
	bridge_group = len(mesh.groups)                                    # dedicated group for the bridging faces

	# working surface, carved in place: oriented faces + directed-edge adjacency
	faceor = {}        # facekeyo -> oriented face
	facetrack = {}     # facekeyo -> group track
	bridges = set()    # facekeyo of the bridging faces
	edgeface = {}      # directed edge -> facekeyo owning it (one owner per directed edge on a manifold)

	def edges(f):
		return (f[0], f[1]), (f[1], f[2]), (f[2], f[0])

	def register(f, track, bridge):
		''' add an oriented face, or cancel it against an already present opposite bridge face '''
		fk = facekeyo(*f)
		if fk in faceor:
			# same 3 points already form a face: opposite orientation, two fronts meeting
			if fk in bridges:
				unregister(faceor[fk])   # internal face, they annihilate
			return
		faceor[fk] = tuple(f)
		facetrack[fk] = track
		if bridge:
			bridges.add(fk)
		for e in edges(f):
			edgeface[e] = fk

	def unregister(f):
		fk = facekeyo(*f)
		faceor.pop(fk, None)
		facetrack.pop(fk, None)
		bridges.discard(fk)
		for e in edges(f):
			if edgeface.get(e) == fk:
				del edgeface[e]

	inputfaces = set(origtrack)

	# normals of the input faces around each vertex, to tell a face bridging a concavity from one
	# merely retriangulating a flat or convex region of the input surface (which must be left alone)
	vfaces = {}
	for f in mesh.faces:
		n = _facenormal(points, f)
		for v in f:
			vfaces.setdefault(int(v), []).append(n)
	flush = cos(0.6)
	# typical input edge length, sets the scale under which a face is considered to lie on the surface
	edge = 0.0
	for f in mesh.faces:
		for u, v in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
			edge += sqrt(length2(points[u]-points[v]))
	edge /= max(1, 3*len(mesh.faces))
	gap2 = (1.5*edge)**2
	def spans_concavity(f):
		''' whether the hull face `f` bridges a concavity instead of lying flush on the input surface.
			two ways to bridge: the surface turns away (some vertex has no input face aligned with `f`),
			or the face floats over a gap (its centroid stands off every input vertex) '''
		n = _facenormal(points, f)
		if any(all(dot(n, g) <= flush  for g in vfaces.get(v, [n]))  for v in f):
			return True
		c = (points[f[0]] + points[f[1]] + points[f[2]]) / 3
		skip = set(f)
		return min((length2(c - points[i])  for i in range(len(points)) if i not in skip), default=0) > gap2

	# seed the surface with the convex hull, a face bridges a concavity when it is neither a face of
	# the input nor lying flush on the input surface. only bridging faces are candidates to dig
	for f in hull.faces:
		f = tuple(int(i) for i in f)
		fk = facekeyo(*f)
		bridge = fk not in inputfaces and spans_concavity(f)
		register(f, bridge_group if bridge else origtrack.get(fk, 0), bridge)

	# advancing front of directed bridge edges. a bridge face (e0,e1,apex) is dug by revealing the
	# input face sharing edge (e0,e1) on its inner side, keeping `apex` as the fixed remote point the
	# bridge fans from, so the angle measured against the bridge does not drift as the front advances
	front = deque(e  for fk in bridges  for e in edges(faceor[fk]))
	while front:
		e = front.popleft()
		bfk = edgeface.get(e)
		if bfk is None or bfk not in bridges:
			continue                       # e is no longer a live bridge edge
		e0, e1 = e
		apex = (set(faceor[bfk]) - {e0, e1}).pop()
		oi = orig.get(e)                   # input face on the inner side of this edge
		if oi is None:
			continue                       # not an input edge, nothing to reveal here
		reveal = tuple(int(i) for i in mesh.faces[oi])
		rk = facekeyo(*reveal)
		if rk in faceor:
			continue                       # already part of the surface (revealed, or a convex face)
		y = (set(reveal) - {e0, e1}).pop()
		if len({e0, e1, apex, y}) < 4:
			continue                       # degenerate, the bridge and the input face share their apex

		# reveal only when the input face stays within `angle` of the bridge face it is dug from.
		# small angle keeps the hull near convex, large angle lets it follow deeper concavities
		if dot(_facenormal(points, reveal), _facenormal(points, faceor[bfk])) < cos(angle):
			continue

		# carve the tetrahedron (e0,e1,apex,y): drop the bridge, reveal the input face, and re-fan two
		# bridges from the apex to the revealed edge. orientations keep every shared edge consistent
		b1 = (y, e1, apex)
		b2 = (apex, e0, y)
		# manifold guard: refuse the carve if any new directed edge is already owned by another live
		# face (would wind two faces the same way or pinch). meeting fronts still cancel in register()
		if any(edgeface.get(de) not in (None, bfk)  for nf in (reveal, b1, b2)  for de in edges(nf)):
			continue
		unregister(faceor[bfk])
		register(reveal, origtrack.get(rk, 0), False)
		for nb in (b1, b2):
			register(nb, bridge_group, True)
		# advance along the new bridge edges bordering an input face not yet revealed
		for nb in (b1, b2):
			nk = facekeyo(*nb)
			if nk not in faceor:
				continue                   # cancelled against a meeting front
			for de in edges(nb):
				if edgeface.get(de) == nk and orig.get(de) is not None:
					if facekeyo(*(int(i) for i in mesh.faces[orig[de]])) not in faceor:
						front.append(de)

	# rebuild the carved surface
	faces = typedlist((uvec3(*f) for f in faceor.values()), dtype=uvec3)
	tracks = typedlist((facetrack[facekeyo(*f)] for f in faceor.values()), dtype='I')
	groups = list(mesh.groups) + [dict()]
	hull = Mesh(points, faces, tracks, groups)

	# nothing was dug (already convex): return as is
	surf = hull.group(bridge_group)
	if not len(surf.faces):
		return hull

	# adaptative subdivision of the bridging faces only (originals are already fine, so left untouched)
	area = surf.surface()
	size = sqrt(2 * area / len(surf.faces))  if area > 0 else  0
	if size > 0:
		hull = subdivide_to(hull, size)
	# relax the interior of the bridging patch, keeping its border on the original surface fixed
	fixed = {i  for f, t in zip(hull.faces, hull.tracks) if t != bridge_group  for i in f}
	moving = [i  for f, t in zip(hull.faces, hull.tracks) if t == bridge_group  for i in f  if i not in fixed]
	if moving:
		# a carved patch may still hold slivers with no valid laplacian: keep the unsmoothed carve
		# rather than aborting (see the manifold/degeneracy limitations in the docstring)
		try:
			hull = smooth(hull, moving)
		except ValueError:
			pass
	return hull
	
	
from .hashing import flipedge
from collections import deque

def outlines_oriented(faces: typedlist[uvec3]) -> set[uvec2]:
	''' just like Mesh.outlines_oriented but only takes a buffer of faces '''
	edges = set()
	for face in faces:
		for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
			if e in edges:	edges.remove(e)
			else:			edges.add((e[1], e[0]))
	return edges

def restore_groups_ngons(original: Mesh, new_faces: typedlist[uvec3]) -> Mesh:
	''' 
		find faces of new mesh that were present in a flat ngon of the original mesh and associate them to the original group. 
		all other faces are associated to a new group
	'''
	def ngon_key(ngon, mesh):
		return tuple(sorted(outlines_oriented(typedlist(mesh.faces[i]  for i in ngon))))
	
	created = len(original.groups)
	original.groups.append({})
	new = Mesh(original.points, new_faces, [created]*len(new_faces), original.groups)
	
	original = {ngon_key(ngon, original): original.tracks[ngon[0]]  for ngon in original.ngons()}
	for ngon in new.ngons():
		key = ngon_key(ngon, new)
		if key in original:
			found = original[key]
			for face in ngon:
				new.tracks[face] = found
				
	return new
	
	
def concavehull(mesh: Mesh, angle=0.5) -> Mesh:
	threshold = cos(angle)
	mesh.strippoints()
	
	convex = restore_groups_ngons(mesh, core.convexhull_3d(mesh.points))
	bridge = convex.group(len(convex.groups)-1)
	assert convex.points is mesh.points
	
	from .rendering import show
	# show(convex, display_wire=True, display_points=True)
	
	# collect bridge outlines
	adjacency = connef(bridge.faces)
	horizon = set()
	front = deque() 
	for edge in adjacency:
		if flipedge(edge) not in adjacency:
			front.append((edge, adjacency[edge]))
			horizon.add(flipedge(edge))
	
	# propagate on original mesh, giving a closest bridge face to faces of orginal mesh until angle is reached
	# since it operates on normals on a convex, there is a unique closest per face and closests are connex
	adjacency = connef(mesh.faces)
	reached = {}
	# propagation is rankwise thanks to the deque
	while front:
		edge, bridger = front.popleft()
		current = adjacency.get(edge)
		if current is None:
			continue
		if edge in horizon:
			continue
		cosangle = dot(bridge.facenormal(bridger), mesh.facenormal(current))
		if cosangle < threshold:
			continue
		previousbridge, previouscosangle = reached.get(current, (bridge, 0))
		if previouscosangle >= cosangle:
			continue
		# we found a new closest
		reached[current] = (bridger, cosangle)
		# propagate from here
		face = mesh.faces[current]
		for i in range(len(face)):
			front.append(((face[i-1], face[i-2]), bridger))
	
	# TODO remove this: for debug
	return Mesh(mesh.points, (mesh.faces[i]  for i in reached), [0]*len(reached), [None])
	
	# get angle frontier
	todo()
		
	# bridges are triangles, after propagation when a bridge is propagated to serveral connex faces, the bridge becomes one new point on a side and many on an other
	# collect new single points
	singles = {}
	todo()
	
	# link to many new points on other side
	todo()
	
	# make a smooth bridge surface
	hull = subdivide_to(hull, selection_faces)
	hull = smooth(hull, selection_points)
	return hull


"""
def hull(bounds: Web) -> Web:
	conn = connpe(bounds)
	bridges = line_bridges(bounds, conn)
	# select a start point the most outward
	center = bounds.barycenter()
	start = min( (i  for e in bounds.edges for i in e),
				key=lambda i: distance2(bounds.points[i], center))
	indev

def hull(bounds: Mesh) -> Mesh:
	indev

def minkowski(a: Mesh, b: Mesh) -> Mesh:
	''' minkowski sum of the input meshes '''
	indev

def brush(brush: Mesh, path, orient=None) -> Mesh:
	''' almost the same as a minkowski sum, but allows for orientation changes of the brush mesh, for each vertex of the path mesh

Parameters:
	brush (Mesh/Web):	the input mesh to move along `path` and being reoriented by `orient`
	path (Mesh/Web):	the input set to move the `brush` along, can be a Web or a Mesh, independently of the dimension of `brush`
	orient (typedlist):
		list of vectors matching the point buffer of `path`, and giving the orientation that `brush` is taking in each of the `path` points
		If set to `None`, it defaults to the path normals
'''
indev
"""
