from madcad import *
from madcad.standard import *

d = 3
show([
	screw(d,12,10, head='BH'),
	nut(d) .transform(vec3(0,0,-7)),
	washer(d) .transform(vec3(0,0,-5)),
	
	screw(5,12, head='SH') .transform(vec3(10,0,0)),
	nut(5) .transform(vec3(10,0,-5)),
	
	screw(4,12, head='VH', drive='slot') .transform(vec3(-10,0,0)),
	nut(4) .transform(vec3(-10,0,-5)),
	
	coilspring_compression(20) .transform(vec3(0,10,0)),
	coilspring_tension(20) .transform(vec3(10,10,0)),
	])
