from random import random, randint
from madcad import vec3, Mesh, show, Circle
from madcad.junctions import *

if False:
	m = junction(
			Circle((vec3(0,0,3),vec3(0,0,1)), 1).mesh(),
			Circle((vec3(-1,-1,-1),normalize(vec3(-1,-1,-1))), 1).mesh(),
			Circle((vec3(1,-1,-1),normalize(vec3(1,-1,-1))), 1).mesh(),
			Circle((vec3(0,1,0),normalize(vec3(0,1,-1))), 1).mesh(),
			generate='normal',
			resolution=('div',5),
			)
	from madcad.generation import extrusion
	from madcad.mesh import web
	m += extrusion(vec3(0,1,-1), web(Circle((vec3(0,1,0),normalize(vec3(0,1,-1))), 1)))

if True:
	from madcad import extrusion, web
	z = vec3(0,0,1)
	x = normalize(vec3(1,1,0))
	m = junction(
		extrusion(2*z, web(Circle((vec3(0),z), 1))),
		extrusion(5*z, web(Circle((vec3(2,0,-2),z), 1))),
		generate='normal',
		)

show([m], {'display_points':False, 'display_wire':False})
