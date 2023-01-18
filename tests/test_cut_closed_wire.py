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
)
from madcad.cut import (
    bevel,
    chamfer,
)


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

    assert profile.length() == approx(expected_len)


def bevel_square(b, r) -> Wire:
    """
    A square profile with rounded corners
    """
    res = ("div", 100)
    wire = square_wire(b)
    bevel(wire, [0, 1, 2, 3], ("distance", r), resolution=res)
    return wire

def bevel_tube(l, b, r) -> Mesh:
    v = vec3(0, 0, l)
    profile = bevel_square(b, r)
    ex = extrusion(v, profile)
    return ex
    

def test_bevel(plot=False):
    profile = bevel_square(b := 2.0, r := 0.5)
    d = r * 2
    expected_len = 4 * (b - d) + pi * d

    if plot:
        profile.finish()
        show([profile])

    assert profile.length() == approx(expected_len)


if __name__ == "__main__":

    test_chamfer(True)
    test_bevel(True)
