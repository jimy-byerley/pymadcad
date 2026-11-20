from madcad import *
from madcad.offseting import thicken, inflate, inflate_offsets
from . import visualcheck

@visualcheck
def test_inflate():
	results = []

	corner = Mesh(
				[vec3(0), vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)], 
				[(0,1,2), (0,2,3), (0,3,1)],
				[0,1,2]
				).flip().transform(vec3(0,-1,0))
	corner.options['color'] = (1, 0.5, 0.1)

	thick = thicken(corner, 0.1, 0.5,collapse=True)
	inflated = inflate(corner, 0.1,collapse=True)
	thick.check()
	assert thick.isenvelope()
	results.append([thick, inflated])

	sphere = pierce(
				icosphere(O, 1), 
				brick(min=vec3(0), max=vec3(2)) .flip(),
				)
	thick = thicken(sphere, 0.1)
	inflated = inflate(sphere, 0.1)
	thick.check()
	assert thick.isenvelope()
	results.append([thick, inflated])

	return results

def test_inflate_collapse_limits_edges():
    sphere = pierce(icosphere(O, 1),brick(min=vec3(0), max=vec3(2)).flip(),)
    original = sphere.points
    displacements = inflate_offsets(sphere, 2.0, collapse=True)

    min_ratio = 0.05

    for face in sphere.faces:
        for i in range(len(face)):
            a = face[i]
            b = face[(i + 1) % len(face)]

            edge = original[b] - original[a]
            new_edge = edge + displacements[b] - displacements[a]

            if length2(edge) <= NUMPREC**2:
                continue

            ratio = dot(new_edge, edge) / length2(edge)

            assert ratio >= min_ratio - 1e-6

def test_inflate_collapse_methods():
    sphere = pierce(
        icosphere(O, 1),
        brick(min=vec3(0), max=vec3(2)).flip(),
    )

    for method in ('face', 'edge', 'point'):
        displacements = inflate_offsets(
            sphere, 2.0, method=method, collapse=True
        )

        assert len(displacements) == len(sphere.points)

        for displacement in displacements:
            assert isfinite(displacement)

def test_inflate_collapse_negative_offset():
    sphere = pierce(
        icosphere(O, 1),
        brick(min=vec3(0), max=vec3(2)).flip(),
    )

    displacements = inflate_offsets(
        sphere, -2.0, collapse=True
    )

    assert len(displacements) == len(sphere.points)

    for displacement in displacements:
        assert isfinite(displacement)


def test_thicken_collapse():
    sphere = pierce(
        icosphere(O, 1),
        brick(min=vec3(0), max=vec3(2)).flip(),
    )

    thick = thicken(
        sphere,
        1.0,
        alignment=0.5,
        collapse=True,
    )

    thick.check()
    assert thick.issurface()
    assert thick.isenvelope()


def test_inflate_collapse_reduces_displacement():
    sphere = pierce(
        icosphere(O, 1),
        brick(min=vec3(0), max=vec3(2)).flip(),
    )

    normal = inflate_offsets(sphere, -2.0, collapse=False)
    collapsed = inflate_offsets(sphere, -2.0, collapse=True)


    assert any(
        length2(a - b) > NUMPREC**2
        for a, b in zip(normal, collapsed)
    )

def test_inflate_collapse_preserves_input():
    sphere = pierce(
        icosphere(O, 1),
        brick(min=vec3(0), max=vec3(2)).flip(),
    )

    original_points = sphere.points[:]
    original_faces = sphere.faces[:]

    inflate(sphere, 2.0, collapse=True)
    thicken(sphere, 2.0, collapse=True)

    assert sphere.points == original_points
    assert sphere.faces == original_faces
