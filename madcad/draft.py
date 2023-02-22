# std
from typing import Tuple, Optional
from copy import copy

# third party
import numpy as np
from arrex import typedlist as tlist
from glm import cross, dot, normalize

# local
from madcad.mesh.container import edgekey
from madcad.mesh.web import Web
from madcad.mesh.wire import Wire
from madcad.mesh import Mesh
from madcad.mathutils import vec3, uvec3, uvec2, normalize
from madcad.primitives import Axis


def draft_angles(mesh: Mesh, draft_direction: vec3, **kwargs):
	"""
	returns:
	a list of draft angles for each face when compared to the draft direction
	"""
	return [
		angle_vectors(draft_direction, face_n, **kwargs)
		for face_n in mesh.facenormals()
	]


def angle_vectors(v1: vec3, v2: vec3, degrees=False) -> float:
	"""
	returns angle between two vectors
	"""
	n1 = normalize(v1)
	n2 = normalize(v2)
	angle = np.arccos(dot(n1, n2))
	if not degrees:
		return angle
	return np.rad2deg(angle)


def offset_vector(n1: vec3, n2: vec3):
	"""
	if edge1 and edge2 is offset a distance(t) along their normals
	returns the intersection offset/t
	"""
	n3 = normalize(n1 + n2)
	cos = dot(n1, n3)
	scale = 1 / cos
	return n3 * scale


def edges_with_points(edges, pids, only=True):
	"""
	args:
		edges: list[vec2]
		pids: list[int], white list of point indices
		only: bool, toggles if edges must have *only* white listed points

	returns:
		a subset of edges that have specifed points
	"""
	if only:
		bool_reduce = all
	else:
		bool_reduce = any

	edges = tlist(
		[e for e in edges if bool_reduce((e[0] in pids, e[1] in pids))],
		uvec2,
	)

	return edges


def draft_offset(
	mesh: Mesh,
	angle: float,
	neutral=Axis(vec3(0), vec3(0, 0, 1)),
	inplace=False,
) -> Optional[Mesh]:
	"""
	Offsets faces in mesh to set draft angle of orthogonal faces.
	Parallel faces are not effected. The adjustment of intermediate faces are interpolated.

	This funcion perserves topology and is suitable for adding drafts on mesh with bevels.

	args:
		mesh: Mesh to draft
		angle: Degrees
		neutral: Axis that defines the neutral plane
		inplace:
			if true:
				the argument "mesh" will be mutated which is useful for partial drafts
			else:
				return a new mesh

	examples:

	"""
	if not inplace:
		mesh = Mesh(copy(mesh.points), mesh.faces, mesh.faces, mesh.groups)

	dists = [p - neutral[0] for p in mesh.points]

	edge_vectors = {e: [] for e in mesh.edges()}
	for face in mesh.faces:
		normal = mesh.facenormal(face)
		# ortho_scale = np.linalg.norm(cross(normal, draft_normal))
		edge_vectors[edgekey(face[0], face[1])].append(normal)
		edge_vectors[edgekey(face[1], face[2])].append(normal)
		edge_vectors[edgekey(face[2], face[0])].append(normal)

	vertex_vectors = {i: [] for i in range(len(mesh.points))}
	for e, vecs in edge_vectors.items():
		if len(vecs) > 1:
			v = offset_vector(*vecs[:2])
		else:
			v = vecs[0]
		vertex_vectors[e[0]].append(v)
		vertex_vectors[e[1]].append(v)

	for i, vecs in vertex_vectors.items():
		if vecs == []:
			# the point is not used by any face, do nothing with it
			continue

		v_arr = np.array(vecs)
		# p' = p - (n ⋅ (p - o)) × n
		projected = (
			v_arr - (v_arr).dot(neutral.direction)[:, np.newaxis] * neutral.direction
		)
		if len(projected) > 1:
			l = np.linalg.norm(projected, axis=1)
			imax = np.argmax(l)
			v = projected[imax]
		else:
			v = projected[0]

		pdist = dot(neutral.direction, mesh.points[i] - neutral.origin)
		offset_scale = -np.tan(np.deg2rad(angle)) * pdist
		mesh.points[i] += v * offset_scale

	if not inplace:
		return mesh


def draft_edges(edges: tlist, points: tlist, trans: vec3, angle: float):
	"""
	adds draft angles by moving the points in the edges
	"""
	edge_arr = np.array([edge.to_tuple() for edge in edges])
	moved_inds = np.unique(edge_arr)
	moved = Web(points, edges)

	trans_normal = normalize(trans)
	vertex_normals = {i: [] for i in moved_inds}
	for e in edges:
		edir = moved.edgedirection(e)
		normal = normalize(cross(edir, trans_normal))
		vertex_normals[e[0]].append(normal)
		vertex_normals[e[1]].append(normal)

	exl = np.linalg.norm(trans)
	for i, normals in vertex_normals.items():
		v_arr = np.array(normals)
		# p' = p - (n ⋅ (p - o)) × n
		projected = v_arr - (v_arr).dot(trans_normal)[:, np.newaxis] * trans_normal
		if len(normals) > 1:
			l = np.linalg.norm(projected, axis=1)
			sorti = np.argsort(-l)
			vector = offset_vector(*projected[sorti[:2]])
		else:
			vector = projected[0]

		scale = np.tan(np.deg2rad(angle)) * exl
		points[i] += vector * scale


def _extrude(base, trans: vec3) -> Tuple[Mesh, list]:
	from_mesh = isinstance(base, Mesh)
	if from_mesh:
		base: Mesh
		base_outline = base.outlines()
		if not len(base_outline.edges):
			raise ValueError("base of type Mesh must have an outline, ie not be closed")
	elif isinstance(base, Web):
		base: Web
		base_outline = base
	elif isinstance(base, Wire):
		base: Wire
		base_outline = Web(base.points, base.edges())
	else:
		raise ValueError(
			"base is not instance a of Mesh, Wire, Web or a subclass of those"
		)

	# prepare
	base.strippoints()
	n_base_points = len(base.points)

	# translate and copy points
	points = base.points[:]
	points.extend(p + trans for p in base.points)

	# create translated edges
	edges = base_outline.edges
	t_edges = [e + n_base_points for e in edges]

	# create faces bridging the translation
	faces = []
	for e, et in zip(edges, t_edges):
		faces.append(uvec3(e[0], e[1], et[0]))
		faces.append(uvec3(et[1], et[0], e[1]))

	if not from_mesh:
		return Mesh(points, faces), t_edges

	base: Mesh
	n_base_faces = len(base.faces)
	faces.extend(base.faces)
	faces.extend(face + n_base_points for face in base.flip().faces)

	return Mesh(points, faces), t_edges


def draft_side(base, trans: vec3, angle: float) -> Mesh:
	ex, edges = _extrude(base, trans)
	draft_edges(edges, ex.points, trans, angle)
	return ex
