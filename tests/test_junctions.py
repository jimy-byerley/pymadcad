from random import random, randint
from madcad import vec3, Mesh, show, Circle, extrusion, web
from madcad.junctions import *

if False:
	interfaces = [
		Circle((vec3(0,0,3),vec3(0,0,1)), 1),
		Circle((vec3(-1,-1,-1),normalize(vec3(-1,-1,-1))), 1),
		Circle((vec3(1,-1,-1),normalize(vec3(1,-1,-1))), 1),
		Circle((vec3(0,1,0),normalize(vec3(0,1,-1))), 1),
		]

	m = junction(
			interface[0],
			interface[1],
			interface[2],
			(interface[3], 'normal'),
			tangents='tangent',
			)
	for c in interface:
		m += extrusion(c.axis[1]*3, web(c))


if True:
	z = vec3(0,0,1)
	x = normalize(vec3(1,1,0))
	interfaces = [
			extrusion(2*z, web(Circle((vec3(0),z), 1))),
			extrusion(5*z, web(Circle((vec3(2,0,-2),z), 0.6))),
			]
	m = junction(
			interfaces[0],
			interfaces[1],
			tangents='tangent',
			weight=-1,
			)

m.check()
show([m, interfaces], {'display_points':False, 'display_wire':True})
