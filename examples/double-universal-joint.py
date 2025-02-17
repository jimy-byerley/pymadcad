from madcad import *
from madcad.joints import *
from itertools import accumulate
settings.resolution = ('rad', 0.1)

# profile separating the bottom and top part allowing them to move around each other and using the maximum volume for the part robustness
def cardan_sphereprofile(maxangle=0.5):
	s = icosphere(O, 1)
	d = vec3(cos(maxangle), 0, sin(maxangle))
	a = normalize(d+1*Y)
	return wire([
		ArcCentered(Axis(O, d*vec3(-1,1,1)), a*vec3(-1,-1,1), a*vec3(-1,1,1)),
		ArcCentered(Axis(O, -Y), a*vec3(-1,1,1), a),
		ArcCentered(Axis(O, d), a, a*vec3(1,-1,1)),
		ArcCentered(Axis(O, Y), a*vec3(1,-1,1), a*vec3(-1,-1,1)),
		])

# generate a side of the universal joint
def cardan_side(rint, rext, rtop, thickness, height):
	margin = 0.1*rint
	profile = cardan_sphereprofile(0.5) .transform(rext*thickness*0.95)
	
	body = union(
			icosphere(O, rext),
			revolution(wire([
				Softened([
					vec3(rext*0.9, 0, height*0.2),
					vec3(rext*0.6, 0, height*0.4),
					vec3(rtop*1.5, 0, height*0.7),
					vec3(rtop*1.5, 0, height-0.5*rtop)]),
				Wire([ 
					vec3(rtop*1.5, 0, height-0.5*rtop), 
					vec3(rtop*1.1, 0, height), 
					vec3(rtop, 0, height), 
					vec3(rtop, 0, height*0.55),
					vec3(0, 0, height*0.55),
					]).segmented(),
				])) .flip() .qualify('junction'),
			)
	body.mergeclose()

	shape = intersection(
		inflate(extrusion(profile.flip(), mat3(5)), -margin),
		body + icosphere(O, thickness*rext).flip(),
		)
	
	rscrew = stfloor(0.6*rtop)/2
	
	pocket = extrusion(flatsurface(convexoutline(web([
		Circle(Axis(vec3(0, rtop*1.8, rext*thickness + 2*rscrew), Y), 2.7*rscrew),
		Segment(
			vec3(+rtop, rtop*1.5, height),
			vec3(-rtop, rtop*1.5, height)),
		]))), rext*Y).orient()
	pocket = union(pocket, 
				extrusion(
					flatsurface(Circle(Axis(vec3(0, rtop*1.35, height - 1.5*rtop), -Y), 2.7*rscrew)),
					rtop*Y))
	removal = union(
				pocket + pocket.transform(scaledir(Y, -1)).flip(),
				(	  cylinder(
						vec3(0, rext, rext*thickness + 2*rscrew), 
						vec3(0, -rext, rext*thickness + 2*rscrew), 
						rscrew*1.2)
					+ cylinder(
						vec3(0, rext, height - 1.5*rtop), 
						vec3(0, -rext, height - 1.5*rtop), 
						rscrew*1.2)
					).qualify('hole'),
				)
	shape = difference(shape, removal)
	
	hole = revolution(wire([
		vec3(rint+0.1*rext, rext, 0),
		vec3(rint, rext-rint*0.3, 0),
		vec3(rint, 0, 0),
		]).segmented().flip(), Axis(O,Y)) .qualify('axis', 'joint')
	result = intersection(
			shape, 
			hole + hole.transform(scaledir(Y,-1)).flip(),
			)
	result = result.finish()

	sep = square((O+1e-5*Y,Y), rext*5)
	return Solid(
		right = difference(result, sep),
		left = intersection(result, sep),
		b1 = bolt(
				vec3(0, rtop*1.8, rext*thickness + 2*rscrew), 
				vec3(0, -rtop*1.8, rext*thickness + 2*rscrew), 
				rscrew),
		b3 = bolt(
				vec3(0, rtop*1.3, height - 1.5*rtop), 
				vec3(0, -rtop*1.3, height - 1.5*rtop), 
				rscrew),
		annotations = [
			note_radius(result.group('axis')),
			note_radius(result.group('hole')),
			],
		)

# just like madcad.standard.bolt but with annotations added
def bolt(a, b, radius, washera=False, washerb=False):
	dir = normalize(b-a)
	rwasher = washer(2*radius)
	thickness = rwasher['part'].box().width.z
	rscrew = screw(radius*2, distance(a,b) + 3*radius)
	rnut = nut(radius*2)
	return Solid(
			screw = rscrew.place((Revolute, rscrew['axis'], Axis(a-thickness*dir, -dir))), 
			nut = rnut.place((Revolute, rnut['top'], Axis(b+thickness*dir, -dir))),
			w1 = rwasher.place((Revolute, rwasher['top'], Axis(b, -dir))),
			w2 = rwasher.place((Revolute, rwasher['top'], Axis(a, dir))),
			)

# central part of the universal joint
def moyeu(brext, rext, thickness, brint=None, slot=False):
	if not brint:
		brint = brext - stceil(1/8*brext)
	
	xint = sqrt((thickness*rext)**2 - brext**2)
	profile = Wire([ 
		(rext - 0.2*brext)*X, 
		(rext - 0.2*brext)*X + 0.99*brint*Z, 
		xint*X - 0.2*brext*X + 0.99*brint*Z,
		xint*X - 0.2*brext*X + 1.1*brext*Z,
		xint*X - 0.3*brext*X + 1.1*brext*Z
		]) .segmented()
	profile.qualify('axis', select=1)
	chamfer(profile, [1], radius=0.1*brext)
	tip = revolution(profile, Axis(O,X))
	tip.finish()

	if slot:
		tip = difference(tip, union(
			brick(vec3(xint-0.15*rint, -0.1*rint, 0.1*rint), vec3(rext, 0.1*rint, 2*rint)),
			brick(vec3(xint-0.1*rint, -rint, 0.5*rint), vec3(rext*2, rint, 2*rint)),
			))

	tubes = repeat(tip, 4, rotatearound(pi/2, Axis(O,Z))) 
	moyeu = tubes + junction(
		tubes.flip(),
		tangents='tangent', weight=-1,
		)
	moyeu

	bearing = slidebearing(
					dint = 2*brint, 
					thickness = brext-brint,
					h = stceil((1-thickness)*rext + 0.2*rint),
					) .transform(translate((rext - 0.2*rint)*X) * rotate(pi/2,Y))

	return Solid(
		part = moyeu,
		bearings = [
			bearing.transform(rotate(i*pi/2, Z))
			for i in range(4)
			],
		annotations = [
			note_radius(moyeu.group('axis').islands()[0]),
			],
		)


# generate the parts
rint = 16/2
rext = 40/2
thickness = 0.7

side = cardan_side(rint*1.02, rext, rext*0.25, thickness, rext*2)
lower = side.transform(rotate(pi/2,Z) * rotate(pi,Y) * rotate(0.5,X))
upper = side.transform(translate(2*rext*Z) * rotate(pi/2,Z) * rotate(0.5,X))
central = moyeu(rint, rext, thickness)
central_lower = central.transform(rotate(-0.5,Y))
central_upper = central.transform(translate(2*rext*Z) * rotate(0.5,Y))

# TODO allow it to move with a kinematic

# add visual annotations
part = central['bearings'][0]['part']
islands = part.group(2).islands()
central['bearings'][0]['annotations'] = [
	note_radius(part.group(1)),
	note_radius(part.group(0)),
	note_distance_planes(islands[0], islands[1]),
	]


# display the result
#show([upper, lower, central])



# generate a side of the universal joint
def cardan_middle(rint, rext, rtop, thickness, height):
	margin = 0.1*rint
	profile = cardan_sphereprofile(0.45) .transform(rext*thickness*0.95)
	
	body = union(
			icosphere(O, rext) + icosphere(O + 2*rext*Z, rext),
			revolution(wire([
				Softened([
					vec3(rext*0.9, 0, height*0.2),
					vec3(rext*0.6, 0, height*0.4),
					vec3(rext*0.6, 0, 2*rext - height*0.4),
					vec3(rext*0.9, 0, 2*rext - height*0.2)]),
				])) .flip() .qualify('junction'),
			)
	body.mergeclose()

	shape = intersection(
		intersection(
			inflate(extrusion(profile.flip(), mat3(5)), -margin),
			inflate(extrusion(profile, mat3(5)).transform(scaledir(Z, -1)).transform(2*rext*Z), -margin),
			),
		body + icosphere(O, thickness*rext).flip() + icosphere(O + 2*rext*Z, thickness*rext).flip(),
		)
	
	rscrew = stfloor(0.6*rtop)/2
	
	removal = bolt_slot(
		vec3(0, 0.5*rext, rext),
		vec3(0, -0.5*rext, rext),
		2*rscrew*1.2).flip()
	shape = difference(shape, removal)
	
	hole = revolution(wire([
		vec3(rint+0.1*rext, rext, 0),
		vec3(rint, rext-rint*0.3, 0),
		vec3(rint, 0, 0),
		]).segmented().flip(), Axis(O,Y)) .qualify('axis', 'joint')
	result = intersection(
			shape, 
			hole + hole.transform(scaledir(Y,-1)).flip() 
             + hole.transform(translate(2*rext*Z)) + hole.transform(translate(2*rext*Z) * mat4(scaledir(Y,-1))).flip(),
			)
	result = result.finish()

	sep = square((O+1e-5*Y,Y), rext*5)
	return Solid(
		right = difference(result, sep),
		left = intersection(result, sep),
		b1 = bolt(
				vec3(0, 0.5*rext, rext), 
				vec3(0, -0.5*rext, rext), 
				rscrew),
		)

middle = cardan_middle(rint*1.02, rext, rext*0.25, thickness, rext*2)
