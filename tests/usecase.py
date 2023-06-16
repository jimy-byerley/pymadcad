import sys

from PyQt6.QtWidgets import QApplication

from madcad import *
from madcad import view
from madcad.constraints import *
from madcad.mesh import suites
from madcad.selection import *

app = QApplication(sys.argv)
main = scn3D = view.Scene()
from madcad import boolean

boolean.scn3D = scn3D


O = vec3(0,0,0)
X = vec3(1,0,0)
Y = vec3(0,1,0)
Z = vec3(0,0,1)

dint = 20
dext = 100
h = 30
rvis = dext/3
dvis = 3
hvis = 3

B = vec3(dint/2, 0, 0)
S = vec3(dint/2, 0, h)
E = vec3(dext/2, 0, 0)
Eh = E+2*Z
Se = S+5*X

line = [
	Segment(B,S),
	ArcCentered(((Se+S)/2, Y), S, Se),
	Segment(Se, Eh),
	Segment(Eh, E),
	Segment(E, B),
	]

solve([
		Tangent(line[0], line[1], S),
		Tangent(line[1], line[2], Se),
		Radius(line[1], 5),
		OnPlane((O,Y), (B,S,E,Eh,Se)),
		],
	fixed=[O,X,Y,Z,B,S,E],
	precision=1e-12
	)
print('points', B,S,E,Eh,Se)
cone = revolution(radians(360), (O,Z), web(line))
print('alignment', cross(cone.facenormal(28), cone.facenormal(29)))

cone.mergeclose()
chamfer(cone, 		# NOTE cette operation est responsable des points a l'interieur du trou central
	suites(select(cone, edgenear(cone, B), crossover).edges)[0], 
	('depth', 3))
#bevel(cone, cone.select(B, crossover), ('depth', 2))
cone.mergeclose()

rplace = dvis*3+2
C = vec3(rvis,0,hvis)
A = vec3(rvis, rplace,hvis)
B = vec3(rvis,-rplace,hvis)
Ae = A+vec3(dext/2, 3,0)
Be = B+vec3(dext/2,-3,0)
line = [
	Segment(Ae, A),
	ArcCentered((C,Z), A, B),
	Segment(B, Be),
	]
solve([
		Tangent(line[0], line[1], A),
		Tangent(line[2], line[1], B),
		Radius(line[1], 3*dvis),
		Distance(Ae, O, dext),
		Distance(Be, O, dext),
		Angle(Segment(Ae,A), Segment(Be,B), radians(60)),
		OnPlane((C,Z), (A,B,Ae,Be)),	# TODO trouver pourquoi ca foire la derniere operation booleenne
		],
	fixed=[O,X,Y,Z,C],
	precision=1e-12,
	)
w = web(line)
w.strippoints()
loop = suites(w.edges)[0]
loop.reverse()
place = extrusion(vec3(0,0,h), w) + flatsurface(Wire(w.points, loop))
place.mergeclose()
sel = select(place, edgenear(place, C-vec3(rplace,0,0)), straight | stopangle(radians(20)))
bevel(place, 
	suites(sel.edges)[0],
	('depth', 2))
place.mergeclose()

#part = difference(cone, place)
#vis = extrusion(vec3(0,0,-2*h), web(Circle((C+vec3(0,0,h),-Z), dvis)))
#part = difference(part, vis)
vis = extrusion(vec3(0,0,-2*h), web(Circle((C+vec3(0,0,h),-Z), dvis)))
place = union(place, vis)
big = Mesh()
for i in range(6):
	pl = deepcopy(place)
	pl.transform(angleAxis(i/6*2*pi, Z))
	big += pl
#place.transform(angleAxis(2*pi/3, Z))
part = difference(cone, big)
#from madcad import boolean
#boolean.debug_propagation = True
#boolean.intersectwith(cone, big)
#boolean.booleanwith(big, cone, True)
#part = big

#part.finish()
#part.groups = [None]*len(part.faces)
#part.tracks = list(range(len(part.faces)))
#part.options = {'debug_display':True, 'debug_points':True, 'debug_faces':False}
#place.options = {'debug_display':True, 'debug_points':False, 'debug_faces':False}
#vis.options.update({'debug_display':True, 'debug_points':False, 'debug_faces':False})
scn3D.add(part)
#scn3D.add(place)
#scn3D.add(vis)
#scn3D.add(big)
scn3D.look(part.box())

main.show()
sys.exit(app.exec())
