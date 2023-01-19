import numpy as np

from madcad.prelude import *
import madcad as cad

from madcad.draft import add_draft

def test_draft(base: Mesh, plot=False):
	trans = vec3(0, 0, 4)
	ex = cad.extrusion(trans, base)

	draft_angle = 5
	print(f"adding draft angle: {draft_angle}")
	add_draft(ex, trans, -draft_angle)
	ex.finish()

	# measure the resulting face angles
	tran_n = normalize(trans)
	angles = []
	print("resulting face angles compared to extrusion axis:")
	for normal in ex.facenormals():
		angle = angle_vectors(normal, tran_n, degrees=True)
		print("  ",angle)
		angles.append(np.round(angle, decimals=3))

	if plot:
		show([ex])
	assert set(angles) == {95.0, 180.0, 0.0}


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


if __name__ == "__main__":
	print("testing drafting on extruded triangle")
	tribase = triangle()
	test_draft(tribase, True)

	print("\ntesting drafting on extruded square")
	sq_base = cad.square(Axis(vec3(), Z), 1)
	test_draft(sq_base, True)
