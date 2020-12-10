from random import random, randint
from madcad import vec3, Mesh, show, Circle, extrusion, web
from madcad.junctions import *

if True:
	m = junction(
			Circle((vec3(0,0,3),vec3(0,0,1)), 1).mesh(),
			Circle((vec3(-1,-1,-1),normalize(vec3(-1,-1,-1))), 1).mesh(),
			Circle((vec3(1,-1,-1),normalize(vec3(1,-1,-1))), 1).mesh(),
			Interface(Circle((vec3(0,1,0),normalize(vec3(0,1,-1))), 1).mesh(), tangents='tangent'),
			generate='normal',
			)
	m += extrusion(vec3(0,1,-1), web(Circle((vec3(0,1,0),normalize(vec3(0,1,-1))), 1)))

if False:
	z = vec3(0,0,1)
	x = normalize(vec3(1,1,0))
	m = junction(
			extrusion(2*z, web(Circle((vec3(0),z), 1))),
			extrusion(5*z, web(Circle((vec3(2,0,-2),z), 0.6))),
			#Interface(extrusion(5*z, web(Circle((vec3(2,0,-2),z), 0.6))), tangents='tangent'),
			generate='normal',
			)

m.check()
show([m], {'display_points':False, 'display_wire':True})
