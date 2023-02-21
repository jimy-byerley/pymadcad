import sys
from typing import Tuple, List


import numpy as np
from pytest import fixture, mark, approx

from madcad.prelude import *
import madcad as cad

from madcad.draft import (
	draft_angles,
	draft_side,
	draft,
	_extrude,
)


PLOT_DEFUALT = "--trace" in sys.argv or __name__ == "__main__" or "-v" in sys.argv


def make_bases() -> List:
	points = tlist(
		[
			[0, 0, 0],
			[1, 0, 0],
			[1, 1, 0],
			[0, 1, 0],
		],
		dtype=vec3,
	)
	bases = [
		Wire(points).own(points=True),
		Mesh(points[:3], [[1, 0, 2]]).own(points=True),
		Web(points, [(0, 1), (1, 2), (2, 0)]).own(points=True),
	]
	return bases


@fixture
def bases() -> List:
	"for extrusions"
	return make_bases()


@fixture(params=[0, 1, 2])
def base(bases, request) -> Mesh:
	"for extrusions"
	yield bases[request.param]


@fixture()
def ex(base) -> Mesh:
	trans = Z * 2
	yield cad.extrusion(trans, base).finish()


@fixture
def meshes(bases) -> List[Mesh]:
	trans = Z * 2
	extrudes = [cad.extrusion(trans, base).finish() for base in bases]
	cube = cad.brick(width=3)
	beveled = bevel_cube()
	cylinder = cad.cylinder(vec3(0, 0, 0), vec3(0, 0, 3), 1).finish()
	yield [cube, cylinder] + extrudes


@fixture(params=range(5))
def mesh(meshes, request) -> Mesh:
	yield meshes[request.param]


@fixture
def dirty(mesh: Mesh) -> Mesh:
	"""
	Test case of mesh with points that are not used by any face
	"""
	mesh.points.extend((X, Y, Z))
	yield mesh


def bevel_cube():
	cube = cad.brick(width=3)
	edges = cube.groupoutlines().edges
	cad.bevel(cube, edges, ("radius", 0.3))
	cube.finish()
	return cube


def make_curvy() -> List[Mesh]:
	orb = cad.icosphere(vec3(0), 1, ("div", 2))
	cube = bevel_cube()
	return [orb, cube]


@fixture(params=[0, 1])
def curvy(request):
	curvs = make_curvy()
	yield curvs[request.param]


@mark.skip
def test_mesh(mesh):
	"For checking that input to test functions is sound"
	plot_normals(mesh)


def check_draft(drafted, angle, normal, plot):
	result_angles = draft_angles(drafted, normal, degrees=True)

	for a in result_angles:
		print(a)

	if plot:
		show([drafted], projection=cad.rendering.Orthographic())
		# plot_normals(drafted)

	angle_set = set(result_angles)
	expected_angles = {0.0, 90.0 - angle, 90.0 + angle, 180.0}
	assert angle_set <= expected_angles


def test_draft_extrusion(base, plot=PLOT_DEFUALT):
	angle = 5
	drafted = draft_side(base, Z * 2, angle)
	check_draft(drafted, angle, Z, plot)


def test_draft_axis(mesh: Mesh, plot=PLOT_DEFUALT):
	axis = Axis(vec3(0, 0, 1), Z)
	angle = 5
	drafted = draft(mesh, axis, angle)
	check_draft(drafted, angle, Z, plot)


def test_extrude(base, plot=PLOT_DEFUALT):
	ex, edges = _extrude(base, Z)
	if plot:
		show([ex])
		plot_normals(ex)


def test_draft_curvy(curvy: Mesh, plot=PLOT_DEFUALT):
	def inspect(mesh, plot):
		result_angles = draft_angles(mesh, Z, degrees=True)
		if plot:
			cad.show([mesh])
			mesh.finish()
		return result_angles

	# orb = cad.icosphere(vec3(0), 1, ("div", 2))

	angle = 5
	angles0 = inspect(curvy, plot)
	drafted = draft(curvy, Axis(vec3(0), Z), angle)
	angles1 = inspect(drafted, plot)

	tol = 0.02
	for a0, a1 in zip(angles0, angles1):
		if a0 == approx(90, abs=tol):
			assert a1 == approx(85, abs=tol)


def test_draft_dirty(dirty: Mesh, plot=PLOT_DEFUALT):
	mesh = dirty
	axis = Axis(vec3(0, 0, 1), Z)
	angle = 5
	drafted = draft(mesh, axis, angle)
	check_draft(drafted, angle, Z, plot)


def plot_normals(mesh: Mesh):
	arrows = []
	for face in mesh.faces:
		center = sum(mesh.facepoints(face)) / 3
		n = mesh.facenormal(face)
		cyl = cad.cylinder(center, center + n * 0.2, 0.05)
		arrows.append(cyl)

	show([mesh] + arrows)


if __name__ == "__main__":
	mesh = bevel_cube()
	show([mesh])
