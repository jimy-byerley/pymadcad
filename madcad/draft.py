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


def add_draft(extruded: Mesh, trans: vec3, angle: float, free=2):
	"""
	Mutates an extruded Mesh to give at a draft angle.

	# Args:
	- extruded(Mesh): the mesh to give draft angle,
	- trans(vec3): vector of extrusion used
	- angle(float): degrees
	"""
	moved = extruded.group(free).outlines()
	edge_arr = np.array([edge.to_tuple() for edge in moved.edges])
	moved_inds = np.unique(edge_arr)

	trans_normal = normalize(trans)
	vertex_normals = {}
	for e in moved.edges:
		edir = moved.edgedirection(e)
		normal = normalize(cross(edir, trans_normal))
		vertex_normals[e[0], 0] = normal
		vertex_normals[e[1], 1] = normal

	exl = np.linalg.norm(trans)
	for i in moved_inds:
		vector = offset_vector(vertex_normals[i, 0], vertex_normals[i, 1])
		scale = np.tan(np.deg2rad(angle)) * exl
		extruded.points[i] += vector * scale


def draft_extrusion(base: Mesh, trans: vec3, angle: float) -> Mesh:
	ex = cad.extrusion(trans, base)
	add_draft(ex, trans, angle)
	return ex


