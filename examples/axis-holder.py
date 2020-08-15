from madcad import *
from madcad.selection import *

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
		Distance(E, Eh, 10),
		OnPlane((O,Y), [B,S,E,Eh,Se]),
		],
	fixed=[O,X,Y,Z,B,S,E],
	precision=1e-12
	)

cone = revolution(
			radians(360), 
			(O,Z), 
			web(line))
cone.mergeclose()
chamfer(cone, 
	suites(select(cone, edgenear(cone, B), crossover).edges)[0], 
	('depth', 3)) 
#bevel(cone, cone.select(B, crossover), ('depth', 2))

print('mergeclose', cone.mergeclose())

rplace = dvis*3+2
C = vec3(rvis,0,hvis)
A = vec3(rvis, rplace,hvis)
B = vec3(rvis,-rplace,hvis)
Ae = A+vec3(dext/2, 60,0)
Be = B+vec3(dext/2,-60,0)
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
		OnPlane((C,Z), [A,B,Ae,Be]),
		],
	fixed=[O,X,Y,Z,C],
	precision=1e-12,
	)
w = web(line)
place = extrusion(vec3(0,0,h), w) + flatsurface(wire(w).flip())
place.mergeclose()
sel = select(place, edgenear(place, C-vec3(rplace,0,0)), straight | stopangle(radians(20)))
bevel(place, 
	suites(sel.edges)[0],
	('depth', 2))
place.mergeclose()

vis = extrusion(vec3(0,0,-2*h), web(Circle((C+vec3(0,0,h),-Z), dvis)))
place = union(place, vis)
big = multiple(place, 6, angle=radians(60), axis=(O,Z))

part = difference(cone, big)
part.finish()

#write(part, 'tests/demo1.ply')

quickdisplay([part])
