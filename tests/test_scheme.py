from madcad import *
from madcad.scheme import *

part = extrusion(vec3(-1,-3,-5), flatsurface(wire(Circle((vec3(1), vec3(0,0,1)), 1))))
cube = brick(center=vec3(-4,0,0), width=vec3(1))
part.mergeclose()

show([
	part,
	cube,
	#note_leading(vec3(1), vec3(2), text="This is a note\n  with\n  several lines"),
	note_floating(vec3(-2,2,-2), text="break all sharp edges\n\nsurfaces must be Ra <= 1.6 unless otherwise indicated"),
	#note_distance(vec3(0,0,1), vec3(0,0,-5), vec3(0,-3,0), tol=0.05),
	#note_angle((vec3(0),normalize(vec3(1,0,0))), (vec3(0),normalize(vec3(-1,2,0))), offset=2, d=4),
	#note_label(vec3(2,1,1), vec3(0,0,1), 'A'),
	
	note_leading(part.group(0), text="ø5 ± 0.05"),
	note_label(part.group(2), text='A'),
	
	#note_cyclinder(part.group(0), tol=0.01),
	note_distance_planes(part.group(1), part.group(2), tol='H7'),
	#note_distance_set(part.group(1), part.group(2)),
	#note_distance_set(part.frontier(0,1), part.frontier(0,2)),
	note_angle_edge(part, (49,50)),
	#note_angle_edge(part.group(0), part.group(1)),
	note_angle_planes(cube.group(0), cube.group(2), offset=-1),
	#note_angle_planes(cube.group(0), cube.group(1)),
	#note_angle_planes((O,x), (O,y+x)),
	])
