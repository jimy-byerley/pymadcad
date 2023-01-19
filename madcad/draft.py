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


def triangle() -> Mesh:
	points = tlist(
		[
			[0, 0, 0],
			[0, 1, 0],
			[1, 1, 0],
		],
		vec3,
	)
	faces = [[0, 1, 2]]
	return Mesh(points, faces)


def angle_vectors(v1: vec3, v2: vec3, degrees=False) -> float:
	n1 = normalize(v1)
	n2 = normalize(v2)
	angle = np.arccos(dot(n1, n2))
	if not degrees:
		return angle
	return np.rad2deg(angle)


def test_draft(base: Mesh, plot=False):
	trans = vec3(0, 0, 4)
	ex = cad.extrusion(trans, base)

	draft_angle = 5
	print(f"adding draft angle {draft_angle}")
	add_draft(ex, trans, -draft_angle)
	ex.finish()

	# measure the resulting face angles
	tran_n = normalize(trans)
	angles = []
	print("resulting face angles compared to extrusion axis")
	for normal in ex.facenormals():
		angle = angle_vectors(normal, tran_n, degrees=True)
		print(angle)
		angles.append(np.round(angle, decimals=3))

	if plot:
		show([ex])
	assert set(angles) == {95.0, 180.0, 0.0}


if __name__ == "__main__":
	print("testing drafting on extruded triangle")
	tribase = triangle()
	test_draft(tribase, True)

	print("testing drafting on extruded square")
	sq_base = cad.square(Axis(vec3(), Z), 1)
	test_draft(sq_base, True)
