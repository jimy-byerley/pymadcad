from madcad import *
from madcad.cut import bevel, chamfer, multicut

if True:
	line = wire([
		vec3(0,0,0),
		vec3(1,0,0),
		vec3(1,1,0),
		vec3(3,3,0),
		vec3(2,4,1),
		vec3(0,2,2),
		vec3(0,0,0),
		])
		
	bevel(line, [1,2,5], ('width',0.6))
	line.check()

	show([line], options={'display_points':True})

if True:
	line = web([
		vec3(0,0,0),
		vec3(1,0,0),
		vec3(1,1,0),
		vec3(3,3,0),
		vec3(2,4,1),
		vec3(0,2,2),
		vec3(0,0,0),
		])
	
	bevel(line, [1,2,5], ('width',0.6))
	line.check()
	
	show([line])
	
