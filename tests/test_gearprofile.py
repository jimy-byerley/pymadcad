from madcad import *
from madcad import gear
from . import visualcheck

@visualcheck
def test_gearprofile():
	settings.resolution = ('rad', 0.05)

	axis = (vec3(0),vec3(0,0,1))

	teeth = 8
	step = 1
	pressure_angle = radians(20)
	height = 0.5 * 2.25 * step/pi * cos(pressure_angle)/cos(radians(20))
	offset = 0.5*height
	primitive = step*teeth / (2*pi)
	prof = gear.gearprofile(step, teeth, height, offset, pressure_angle=pressure_angle)

	return [
		vec3(0),
		vec3(1,0,0),
		(vec3(primitive,0,0), vec3(1,0,0)),
		
		web(prof) + web(prof).transform(angleAxis(2*pi/teeth, vec3(0,0,1))),
		
		Circle(axis, primitive),
		Circle(axis, primitive * cos(pressure_angle)),
		Circle(axis, primitive +height +offset),
		Circle(axis, primitive -height +offset),
		]
