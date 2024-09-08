'''
	This is an example to generate a hexnut from scratch. Of course it's even better to use the provided function in module `madcad.standard`
'''

from madcad import *

# create a base, for convenience in the code
O = vec3(0,0,0)
X = vec3(1,0,0)
Y = vec3(0,1,0)
Z = vec3(0,0,1)

# part parameters
d = 8
w = 6
h = 6

# build the revolution profile with approximate positions
axis = Axis(O,Z)
P0 = vec3(   0.5*d,          0,      0.5*h )
P1 = vec3(      3.11142,          0.5,      3.39084 )
P2 = vec3(      w,          0.5,      1.38932 )
P3 = vec3(      w,          0.5,     -1.23665 )
P4 = vec3(      3.40459,          0.5,     -3.2627 )
P5 = vec3(   0.5*d,          0,     -0.5*h )
profile = [
	Segment(P0, P1),
	Segment(P1, P2),
	Segment(P2, P3),
	Segment(P3, P4),
	Segment(P4, P5),
	Segment(P5, P0),
	]
# mutate the profile to fit some constraints
csts = [
	OnPlane((O,Y), [P1,P2,P3,P4]),
	Parallel(profile[2], axis),
	Angle(profile[0], profile[1], 		radians(45)),
	Angle(profile[4], profile[3],		radians(45)),
	Angle(profile[0], axis,		radians(90)),
	Angle(profile[4], axis,		radians(90)),
	Distance(profile[2], axis,		1.01 * w/cos(radians(30))),
	Distance(P1, axis, 		0.9*w),
	Distance(P4, axis, 		0.9*w),
	]
solve(csts, fixed=[O,X,Y,Z,P0,P5], precision=1e-6, method='trf')

# generate a revolution surface from the profile
base = revolution(profile)
base.mergeclose()  # merge revolution ends

# make an haxagon to cut the nut sides
hexagon = regon(
			Axis(-h*Z,Z), 
			w/cos(radians(30)),
			6,
			)
# trick to separate edges in different groups
ext = extrusion(hexagon, 2*h*Z)

# cut the revolution by the hexagon
nut = intersection(base, ext)
# put chamfers and roundings at top and bottom
filet(nut, nut.frontiers(5,4), width=0.7)
chamfer(nut, nut.frontiers(0,5), width=0.6)

# place some dimensinal notes (also to check that our model is good)
notes = [
	note_leading(nut.group(5), text='M{}'.format(d)),
	note_distance_planes(nut.group(6), nut.group(9)),
	note_distance_planes(nut.group(0), nut.group(4)),
	]

# display what we want
show([nut, notes])
