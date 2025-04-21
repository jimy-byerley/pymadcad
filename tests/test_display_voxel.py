from madcad import *
from madcad.rendering.d3.dense import VoxelsDisplay
from madcad.rendering import Displayable, show
import numpy as np

voxel = np.random.random((30,20,10))
disp = Displayable(VoxelsDisplay, voxel, fmat4(3,2,1,1), (0.4, 0.9))

show([
	disp, 
	Box(min=vec3(0), max=vec3(1)), 
	Circle((O,Z),1),
	Circle((0.5*Z,Z),1),
	uvsphere(-2*Z-X,1),
	brick(center=-Z+X, width=vec3(1.99)),
	])
