from madcad import *
from madcad.boolean import cut_web, pierce_web, boolean_web

wa = web(Circle((O,Z), 1))

others = {
	0: web(wire([vec3(-1, 0.5, 0), vec3(1, 0.2, 0)])),
	1: web(Softened([vec3(-2, 0, 0), vec3(-1, 0.5, 0), vec3(1, 0.2, 0), vec3(2, 3, 0)])),
	2: web(wire([vec3(-2,-0.5,0), vec3(2,-0.5,0), vec3(2,0.5,0), vec3(-2,0.5,0)]).close()),
	#3: web(wire([vec3(-1,-0.5,0), vec3(1,-0.5,0), vec3(1,0.5,0), vec3(-1,0.5,0)]).close()),
	4: web([
		Segment(vec3(-1,-0.5,0), vec3(1,-0.5,0)), 
		Segment(vec3(1,0.5,0), vec3(-1,0.5,0)),
		]),
	}

results = []
for i, wb in others.items():
	print('*', i, repr(wb))
	results.extend([
		Solid(content=pierce_web(wa, wb), wb=wb),
		Solid(content=boolean_web(wa, wb, (True, False))),
		Solid(content=boolean_web(wa, wb, (False, True))),
		])

for i, result in enumerate(results):
	result.position += i*Z

show([results], options={'display_points':True})
