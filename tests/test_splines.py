from madcad import *

curve = [vec3(0), vec3(1), vec3(2,0,1), vec3(3,3,1)]

show([Softened(curve), Interpolated(curve)])
