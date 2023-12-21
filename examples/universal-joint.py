from madcad import *
from itertools import accumulate
settings.primitives['curve_resolution'] = ('rad', 0.1)

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
			revolution(2*pi, Axis(O,Z), wire([
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
		inflate(extrusion(mat3(5), profile.flip()), -margin),
		body + icosphere(O, thickness*rext).flip(),
		)
	
	rscrew = stfloor(0.6*rtop)/2
	
	pocket = extrusion(rext*Y, flatsurface(convexoutline(web([
		Circle(Axis(vec3(0, rtop*1.8, rext*thickness + 2*rscrew), Y), 2.7*rscrew),
		Segment(
			vec3(+rtop, rtop*1.5, height),
			vec3(-rtop, rtop*1.5, height)),
		])))).orient()
	pocket = union(pocket, 
				extrusion(rtop*Y, flatsurface(
					Circle(Axis(vec3(0, rtop*1.35, height - 1.5*rtop), -Y), 2.7*rscrew)
				)))
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
	
	hole = revolution(2*pi, Axis(O,Y), wire([
		vec3(rint+0.1*rext, rext, 0),
		vec3(rint, rext-rint*0.3, 0),
		vec3(rint, 0, 0),
		]).segmented().flip()) .qualify('axis', 'joint')
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
			screw = rscrew.place((Pivot, rscrew['axis'], Axis(a-thickness*dir, -dir))), 
			nut = rnut.place((Pivot, rnut['bottom'], Axis(b+thickness*dir, -dir))),
			w1 = rwasher.place((Pivot, rwasher['top'], Axis(b-0.5*thickness*dir, -dir))),
			w2 = rwasher.place((Pivot, rwasher['top'], Axis(a+0.5*thickness*dir, dir))),
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
	chamfer(profile, [1], ('radius', 0.1*brext))
	tip = revolution(2*pi, Axis(O,X), profile)
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

upper = cardan_side(rint*1.02, rext, rext*0.25, thickness, rext*2)
lower = upper.transform(rotate(pi/2,Z) * rotate(pi,Y))
central = moyeu(rint, rext, thickness)

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
show([upper, lower, central])
