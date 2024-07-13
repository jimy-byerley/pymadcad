from pytest import approx
from math import pi, sqrt

from madcad import (
	# Axis,
	vec3,
	Wire,
	# Web,
	Mesh,
	show,
	extrusion,
	ArcThrough,
	wire,
	)
from madcad.cut import (
	bevel,
	chamfer,
	)


def arc_wire() -> Wire:
	P0 = vec3(0.08051, -2.024e-08, 0.1698)
	w = wire(
		[
			ArcThrough(
				vec3(-0.1988, 1.5e-10, -0.001258), vec3(-0.06793, -1.35e-08, 0.1132), P0
				),
			ArcThrough(
				P0, vec3(0.08932, -3.899e-09, 0.03271), vec3(0.2327, 1.335e-08, -0.112)
				),
			]
		)
	# w.finish()
	return w


def test_cut_curve(plot=False):
	w = arc_wire()
	bevel(w, [4], ("width", 0.15))
	w.check()
	if plot:
		show([w])


def square_wire(b=2.0) -> Wire:
	"""
	A closed wire, shaped like a square
	"""
	points = [
		vec3(0, 0, 0),
		vec3(b, 0, 0),
		vec3(b, b, 0),
		vec3(0, b, 0),
		]
	inds = [0, 1, 2, 3, 0]
	return Wire(points, inds)


def chamfer_square(b, d) -> Wire:
	"""
	A square profile with rounded corners
	"""
	# low_res = resolution=("div", 1)
	wire = square_wire(b)
	chamfer(wire, [0, 1, 2, 3], ("distance", d))
	return wire


def test_chamfer(plot=False):
	profile = chamfer_square(
		b := 2.0,
		d := 0.5,
	)
	expected_len = 4 * (b - d * 2) + 4 * sqrt(d)
	profile.finish()
	for p in profile.points:
		print(p)

	if plot:
		show([profile])

	profile.check()
	assert profile.length() == approx(expected_len)


def bevel_square(b, r) -> Wire:
	"""
	A square profile with rounded corners
	"""
	res = ("div", 100)
	wire = square_wire(b)
	bevel(wire, [0, 1, 2, 3], ("distance", r), resolution=res)
	return wire


def tube(length, b, r) -> Mesh:
	v = vec3(0, 0, length)
	profile = bevel_square(b, r)
	ex = extrusion(profile, v)
	print(ex)
	return ex


def test_bevel(plot=False):
	profile = bevel_square(b := 2.0, r := 0.5)
	d = r * 2
	expected_len = 4 * (b - d) + pi * d
	profile.finish()

	if plot:
		show([profile])

	profile.check()
	assert profile.length() == approx(expected_len, abs=0.05)  # large tolerance because the bevel isn't an actual arc, and is discretized


if __name__ == "__main__":
	test_cut_curve(True)
	test_chamfer(True)
	test_bevel(True)
	tube = tube(6, 2, 0.5)
	show([tube])
