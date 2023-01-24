import numpy as np

from madcad.prelude import *
import madcad as cad


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
	if only:
		bool_reduce = all
	else:
		bool_reduce = any

	edges = tlist(
		[e for e in edges if bool_reduce((e[0] in pids, e[1] in pids))],
		uvec2,
	)

	return edges
	

def draft_edges(edges, points, trans, angle):
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
		if len(normals) > 1:
			vector = offset_vector(*normals)
		else:
			vector = normals[0]

		scale = np.tan(np.deg2rad(angle)) * exl
		points[i] += vector * scale


def draft_by_groups(extruded: Mesh, angle: float, free=2, fixed=1):
	"""
	Mutates an extruded Mesh to give at a draft angle.

	# Args:
	- extruded(Mesh): the mesh to give draft angle,
	- angle(float): degrees
	- free(int): group of faces with outline to offset
	"""
	moved = extruded.group(free).outlines()
	trans = moved.barycenter() - extruded.group(fixed).outlines().barycenter()
	draft_edges(moved.edges, moved.points, trans, angle)
	return extruded

def draft_by_slice(ex: Mesh, angle: float, index=None, reverse=False):
	lp = len(ex.points)
	if index is None:
		index = lp // 2

	if reverse:
		fixed_slice = slice(index, lp)
		free_slice = slice(0, index)
		free_inds = list(range(0, index))
	else:
		fixed_slice = slice(0, index)
		free_slice = slice(index, lp)
		free_inds = list(range(index, lp))
	
	edges = edges_with_points(ex.edges(), free_inds)
	trans = (sum(ex.points[index:]) - sum(ex.points[:index]))/index
	draft_edges(edges, ex.points, trans, angle)
	return ex


def draft_extrusion(trans: vec3, base: Mesh | Web | Wire,  angle: float) -> Mesh:
	ex = cad.extrusion(trans, base)
	ex.finish()
	draft_by_slice(ex, angle)
	return ex

