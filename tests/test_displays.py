import numpy as np
from madcad import *
from madcad.rendering import *
from madcad.rendering.d3.marker import *
from . import visualcheck, Hidden

@visualcheck
def test_displays():
	scn = []
	m = Mesh(
		[
			vec3(1.0, -1.0, -1.0),
			vec3(1.0, -1.0, 1.0),
			vec3(-1.0, -1.0, 1.0),
			vec3(-1.0, -1.0, -1.0),
			vec3(1.0, 1.0, -1.0),
			vec3(1.0, 1.0, 1.0),
			vec3(-1.0, 1.0, 1.0),
			vec3(-1.0, 1.0, -1.0)],
		[
			(0, 1, 2),
			(0, 2, 3),
			(4, 7, 6),
			(4, 6, 5),
			(0, 4, 5),
			(0, 5, 1),
			(1, 5, 6),
			(1, 6, 2),
			(2, 6, 7),
			(2, 7, 3),
			(4, 0, 3),
			(4, 3, 7)],
		list(range(12)),
		)
	scn.append(m)

	w = Web(
		[	
			vec3(0,0,0),
			vec3(1,0,0),vec3(0,1,0),vec3(0,0,1),
			vec3(0,1,1),vec3(1,0,1),vec3(1,1,0),
			vec3(1,1,1)],
		[	
			(0,1),(0,2),(0,3),
			(1,6),(2,6),
			(1,5),(3,5),
			(2,4),(3,4),
			(4,7),(5,7),(6,7),
			],
		list(range(12)),
		)
	w.transform(vec3(0,0,2))
	scn.append(w)

	scn.append(Displayable(PointDisplay, vec3(0,0,0) ))
	scn.append(Displayable(PointDisplay, vec3(1,1,1) ))
	scn.append(Displayable(PointDisplay, vec3(1,0,1) ))
	scn.append(Displayable(PointDisplay, vec3(1,0,0) ))
	scn.append(Displayable(AxisDisplay, (vec3(2,0,0), vec3(0,0,1)) ))
	scn.append(Displayable(BoxDisplay, Box(vec3(-3), vec3(-1,-2,-1.5)) ))
	scn.append(Displayable(GridDisplay, vec3(0)))

	return [scn, Hidden(render(scn))]

def test_transparent_background():
	scene = Scene({'part': brick(width=vec3(1))})
	size = uvec2(200, 150)
	view = Offscreen3D(scene, size, view=fmat4(1), proj=fmat4(1))

	# opaque by default
	view.render(size)
	h, w = view.color.shape[:2]
	assert view.color.shape[2] == 3

	# enable alpha
	view.enable_alpha = True
	view.render(size)
	assert view.color.shape == (h, w, 4)

	alpha = view.color[:, :, 3]
	rgb = view.color[:, :, :3]
	has_color = np.any(rgb > 0, axis=2)

	# background pixels must be fully transparent
	assert np.all(alpha[~has_color] == 0), "background pixels should have alpha=0"
	# geometry pixels must have non-zero alpha (edges may be anti-aliased, so not always 255)
	assert np.all(alpha[has_color] > 0), "geometry pixels should have alpha>0"
	# there should be both transparent and opaque pixels
	assert np.any(alpha == 0), "expected some transparent pixels"
	assert np.any(alpha == 255), "expected some opaque pixels"

	# toggle back to opaque
	view.enable_alpha = False
	view.render(size)
	assert view.color.shape == (h, w, 3)

	# rapid toggles without crash
	for i in range(5):
		view.enable_alpha = (i % 2 == 0)
		view.render(size)
