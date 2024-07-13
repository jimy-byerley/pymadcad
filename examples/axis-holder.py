from madcad import *

# define a base of vectors: origin and directions
O = vec3(0,0,0)
X = vec3(1,0,0)
Y = vec3(0,1,0)
Z = vec3(0,0,1)

# part parameters
dint = 20
dext = 100
h = 30
rvis = dext/3
dvis = 3
hvis = 3


# create the revolution profile, called 'cone'
# --------------------------------------------

B = vec3(dint/2, 0, 0)
S = vec3(dint/2, 0, h)
E = vec3(dext/2, 0, 0)
Eh = E+2*Z
Se = S+5*X
# sketch a profile
line = [
	Segment(B,S),
	ArcCentered(((Se+S)/2, Y), S, Se),
	Segment(Se, Eh),
	Segment(Eh, E),
	Segment(E, B),
	]
# mutate the profile to fit some geometrical constraints
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
# generate the cone
cone = revolution(line)
# merge the start and end sections of the revolution (because its a 360° revolution)
cone.mergeclose()
# chamfer the lower edge: this is a chamfer over a circular edge
chamfer(cone, cone.frontiers((0,4)), ('depth', 3))



# create the slots for screws
# ---------------------------
# we remove a partially defined volume

rplace = dvis*3+2
C = vec3(rvis,0,hvis)
A = vec3(rvis, rplace,hvis)
B = vec3(rvis,-rplace,hvis)
Ae = A+vec3(dext/2, 60,0)
Be = B+vec3(dext/2,-60,0)
# sketch its line
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
# extrude that base line and add a bottom face
place = (	extrusion(line, vec3(0,0,h)) 
		+	flatsurface(wire(line).flip())
		)
# merge outlines of both generated faces
place.mergeclose()
# round the cutting edge to have smooth transition
bevel(
	place, 	
	(   place.frontiers(0,3) 	# this is the frontier line between group 0 and group 3
	  + place.frontiers(1,3) 	# this is the frontier line between group 1 and group 3
	  + place.frontiers(2,3) ), 
	('depth', 2))

# make the screw holes:
# a cylinder (not necessarily closed on its ends as we don't care of that surfaces)
vis = extrusion(Circle((C+vec3(0,0,h),-Z), dvis), vec3(0,0,-2*h))


# assemble everything
# -------------------
# get 6 shapes with the slot and the hole for the scren
big = repeat(
		union(place, vis), 	# this union cuts the slot to add the hole
		6, rotatearound(radians(60), (O,Z)))
# cut the cone to put the slots and holes
part = difference(cone, big)
# this is the final touch for parts: optimize the buffers and check mesh validity
part.finish()




# if we want we can at any moment place some fancy notes
notes = [
	note_leading(part.group(2), text="conic surface"),
	note_leading(part.group(11), vec3(-5,0,-10), text='ø'+str(dvis)),
	]

# write the part to a ply file
write(part, 'tests/axis-holder.ply')

# display what we want
show([part, notes])
