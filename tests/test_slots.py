from madcad import *
from itertools import accumulate
from madcad.standard import *

a = O
b = 16*X

b1 = bolt(a, b, 3)
s1 = bolt_slot(a, b, 3)

o = -15*Z
b2 = screw(3, 11).transform(o)
s2 = screw_slot(Axis(o,Z), 3, hole=11)

b3 = bearing(16, 35, 11).transform(30*Z)
s3 = bearing_slot_exterior(Axis(30*Z,Z), 35, 11, shouldering=True, circlip=True, evade=True)
s4 = bearing_slot_interior(Axis(30*Z,Z), 16, 11, shouldering=True, circlip=False, evade=False)

show([
	a, b,
	b1, s1,
	b2, s2,
	b3, s3, s4,
	])