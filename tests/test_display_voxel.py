from madcad import *
from madcad.rendering.d3.dense import VoxelsDisplay
from madcad.rendering import Displayable, show
import numpy as np
from . import visualcheck, Hidden

np.random.seed(7)

@visualcheck
def test_display_voxel():
	voxel = np.random.random((30,20,10))
	disp = Displayable(VoxelsDisplay, voxel, fmat4(3,2,1,1), (0.4, 0.9))

	scene = [
		disp, 
		Box(min=vec3(0), max=vec3(1)), 
		Circle((O,Z),1),
		Circle((0.5*Z,Z),1),
		uvsphere(-2*Z-X,1),
		brick(center=-Z+X, width=vec3(1.99)),
		]
	return [scene, Hidden(render(scene))]
