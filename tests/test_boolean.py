from copy import deepcopy
from madcad import *
from madcad.boolean import pierce, boolean
from madcad.generation import brick
from madcad.nprint import nprint

m1 = brick(width=vec3(2))
m2 = (deepcopy(m1)
		.transform(vec3(0.5, 0.3, 0.4))
		.transform(quat(0.7*vec3(1,1,0)))
		)
		
#from madcad.boolean import cut_mesh
#res, frontier = cut_mesh(m1, m2)
#show([res, frontier, m2], options={'display_wire':True, 'display_faces':False})

results = []

for side in (False, True):
	nprint('* pierce(side={})'.format(side))
	
	r = pierce(m1, m2, side)
	r.check()
	assert r.issurface()
	results.append(r)
	
	r = pierce(m2, m1, side)
	r.check()
	assert r.issurface()
	results.append(r)

for sidea in (False, True):
	for sideb in (False, True):
		nprint('* boolean(sides=({}, {}))'.format(sidea, sideb))
		r = boolean(m1, m2, (sidea, sideb))
		r.check()
		assert r.isenvelope()
		results.append(r)

# display
for i in range(len(results)):
	results[i] = Solid(content=results[i])
	results[i].position += 4*i*Y

show(results)

