from madcad import vec3, Point, Segment, ArcCentered, Distance, Tangent, Angle, solve
from math import radians
from madcad.rendering import show

print('\nequilateral triangle')
A = Point(0, 0, 0)
B = Point(1, 0, 0)
C = Point(0, 1, 0)
AB = Segment(A,B)
AC = Segment(A,C)
BC = Segment(B,C)

csts = [
	Distance(A,B, 1),
	Distance(A,C, 1),
	Distance(B,C, 1),
	]
solve(csts, fixed=[A], afterset=lambda x: print(repr((A,B,C))))
print('final', repr((A,B,C)))
show([AB,AC,BC, csts])

print('\ntangent arc')
A = Point(0, 0, 0)
B = Point(1, 0, 0)
C = Point(0, 1, 0)
AB = Segment(A,B)
AC = Segment(A,C)
O = Point(0.8,0.8,0)
Z = vec3(0,0,1)
BC = ArcCentered((O,Z), B,C)

csts = [
	Distance(A,B, 1),
	Distance(A,C, 1),
	Distance(B,C, 1),	# resolution en cas de probleme sous-contraint: enlever cette ligne
	Tangent(AB,BC,B),
	Tangent(AC,BC,C),
	]
solve(csts, fixed=[A,Z], afterset=lambda x: print(repr((A,B,C,O))))
print('final', repr((A,B,C,O)))
show([AB, AC, BC, csts])

print('\nangle')
A = Point(0, 0, 0)
B = Point(1, 0, 0)
C = Point(0, 1, 0)
D = Point(0.9, 0.9, 0)
AB = Segment(A,B)
AC = Segment(A,C)
CD = Segment(C,D)
BD = Segment(B,D)

csts = [
	Distance(A,B, 1),
	Distance(A,C, 1),
	Angle(AB,AC, radians(120)),
	Angle(CD,BD, radians(90)),
	#Parallel(AB,CD),
	]
solve(csts, fixed=[A], afterset=lambda x: print(repr((A,B,C,D))))
print('final', repr((A,B,C,D)))
show([AB, AC, CD, BD, csts])
