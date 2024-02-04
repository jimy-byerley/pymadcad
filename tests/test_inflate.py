from madcad import *
from madcad.generation import *

results = []

corner = Mesh(
			[vec3(0), vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)], 
			[(0,1,2), (0,2,3), (0,3,1)],
			[0,1,2]
			).flip().transform(vec3(0,-1,0))
corner.options['color'] = (1, 0.5, 0.1)

thick = thicken(corner, 0.1, 0.5)
inflated = inflate(corner, 0.1)
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

show(results)
