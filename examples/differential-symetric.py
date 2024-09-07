from madcad import *
from madcad.gear import *
from madcad.joints import *

def sbevelgear(step, z, pitch_cone_angle, **kwargs):
	part = bevelgear(step, z, pitch_cone_angle, **kwargs)
	top = project(part.group(4).barycenter(), Z)
	bot = project(part.group(1).barycenter(), Z)
	return Solid(
		part = part .option(color=gear_color),
		summit = O,
		axis = Axis(top, -Z, interval=(0, length(top))),
		bot = Axis(bot, Z, interval=(0, length(top-bot))),
		)

def bolt(a, b, dscrew, washera=False, washerb=False):
	dir = normalize(b-a)
	rwasher = washer(dscrew)
	thickness = rwasher['part'].box().width.z
	rscrew = screw(dscrew, stceil(distance(a,b) + 1.2*dscrew, precision=0.2))
	rnut = nut(dscrew)
	rscrew['annotations'] = [
				note_distance(O, -stceil(distance(a,b) + 1.2*dscrew)*Z, offset=2*dscrew*X),
				note_radius(rscrew['part'].group(0)),
				]
	return Solid(
			screw = rscrew.place((Revolute, rscrew['axis'], Axis(a-thickness*dir, -dir))), 
			nut = rnut.place((Revolute, rnut['top'], Axis(b+thickness*dir, -dir))),
			w1 = rwasher.place((Revolute, rwasher['top'], Axis(b, -dir))),
			w2 = rwasher.place((Revolute, rwasher['top'], Axis(a, dir))),
			)

# the discretisation paremeter can be set high for exportation, or small for quick computations
#settings.primitives['curve_resolution'] = ('rad', 0.105)
#settings.primitives['curve_resolution'] = ('rad', 0.19456)
settings.primitives['curve_resolution'] = ('sqradm', 0.5)

# parameters of the mechanism to design
transmiter_angle = pi/6
transmiter_z = 8
gear_step = 6
output_radius = 5
gear_color = vec3(0.2, 0.3, 0.4)
dscrew = 3
neighscrew = 1.3*dscrew
shell_thickness = 1

# dedices parameters
axis_z = round(transmiter_z/tan(transmiter_angle))
transmiter_rint = stceil((transmiter_z * gear_step / (2*pi) - 0.6*gear_step) * (0.6 + 0.2*sin(transmiter_angle)))
transmiter_axis_thickness = stceil(transmiter_rint*0.2)
transmiter_washer_thickness = stceil(transmiter_rint*0.4)
# interior area radius
space_radius = transmiter_z*gear_step/(2*pi) / sin(transmiter_angle) + transmiter_washer_thickness * 1.6

# output bearing dimensions
bearing_height = stceil(output_radius)
bearing_radius = stceil(output_radius*2.5)
bearing_bore = stceil(output_radius*1.5*2)
# overides for the output bearing
bearing_height = 5
bearing_radius = 24/2
bearing_bore = 15/2

# output interface shafts construction
out_gear = sbevelgear(gear_step, axis_z, pi/2-transmiter_angle, 
				bore_radius=output_radius, 
				bore_height=1.2*bearing_height,
				)
output = Solid(
	gear = out_gear,
	bearing = bearing(bearing_bore*2, bearing_radius*2, bearing_height)
				.transform(out_gear['axis'].origin - 0.5*bearing_height*Z),
	)
# add a shouldering to the gear part to support the bearing
bearing_support = revolution(wire([
		out_gear['part'].group(4).barycenter() - bearing_height*Z + output_radius*1.5*0.9*X,
		out_gear['part'].group(4).barycenter() - bearing_height*Z + output_radius*1.5*1.2*X,
		out_gear['part'].group(7).barycenter() + output_radius*1.5*1.2*X - output_radius*0.01*Z,
		]))
out_gear['part'] = union(out_gear['part'], bearing_support).option(color=out_gear['part'].options['color'])

# output interface shafts
output1 = output
output2 = deepcopy(output).transform(rotate(pi,Y))

# intermediate shafts, responsible for synchronization of output shafts
transmiter_gear = sbevelgear(gear_step, transmiter_z, transmiter_angle, 
				bore_height=0, 
				bore_radius=transmiter_rint * 1.05,
				)
transmiter = Solid(
		gear = transmiter_gear,
		bearing = slidebearing(
				(transmiter_rint-transmiter_axis_thickness)*2, 
				stfloor(space_radius + dscrew + neighscrew - length(transmiter_gear['bot'].origin)), 
				transmiter_axis_thickness,
				) .transform(translate(transmiter_gear['bot'].origin) * rotate(pi,X)),
		washer = washer(
				stceil(transmiter_rint*2), 
				stceil(transmiter_rint*1.8*2), 
				transmiter_washer_thickness,
				) .transform(transmiter_gear['axis'].origin),
		).transform(rotate(pi/2,Y))
# add annotations not (yet ?) present in the standard assemblies
transmiter['bearing']['annotations'] = [
	note_radius(transmiter['bearing']['part'].group(1)),
	note_distance_planes(*transmiter['bearing']['part'].group(2).islands()),
	]
transmiter['washer']['annotations'] = [
	note_distance_planes(transmiter['washer']['part'].group(1), transmiter['washer']['part'].group(0)),
	note_radius(transmiter['washer']['part'].group(2).islands()[1]),
	]
# generate the maximum possible number of transmiters, so that the total force that can be hold is high
transmiter_amount = ceil(axis_z / (1.5*transmiter_z/pi))
transmiters = [deepcopy(transmiter).transform(rotate(i*2*pi/transmiter_amount,Z))  for i in range(transmiter_amount)]

# build the interior shell based on the interior area bounds
# the top part is bound to the ouput gear
interior_top = revolution(Wire([
	out_gear['axis'].origin + bearing_radius*X - bearing_radius*0.15*X,
	out_gear['axis'].origin + bearing_radius*X,
	out_gear['axis'].origin + bearing_radius*X - bearing_height*Z,
	out_gear['axis'].origin + bearing_radius*X - bearing_height*1.2*Z + bearing_height*0.2*X,
	out_gear['axis'].origin + space_radius*X  - bearing_height*1.2*Z,
	]).flip().segmented())
interior_out = (
			interior_top 
			+ interior_top.transform(scaledir(Z,-1)).flip()
			).finish()

# the side part is bound to the transmiters
r = length(transmiter['gear']['axis'].origin)
h = transmiter_rint
w = r + transmiter_washer_thickness*1.5
interior_transmision = revolution(Wire([
	2*r*X + h*Z,
	w*X + h*Z,
	w*X + 2.5*h*Z,
	w*X + 2.5*h*Z + h*(X+Z),
	]).flip().segmented(), Axis(O,X))
interior_transmisions = repeat(interior_transmision, transmiter_amount, rotate(2*pi/transmiter_amount, Z))

# the rest is bound to the bevel gearing
interior_space = union(
				icosphere(O, space_radius),
				cylinder(out_gear['axis'].origin, out_gear['axis'].origin*vec3(1,1,-1), bearing_radius*1.05, fill=False),
				).flip()
# mix all together in the interior shell
interior_shell = union(interior_space, interior_out)
interior = union(interior_shell, interior_transmisions.group({0,1})) + interior_transmisions.group({2,3,4,5})
interior.mergeclose()

## symetrical exterior
# the exterior shell is based on an thickened version of the interior shell
exterior_shell = inflate(interior_shell.flip(), shell_thickness)

# place screws for assembly and torque transmission to the input ring
a = 2*dscrew*Z + stceil(space_radius + 0.8*dscrew, precision=0.05)*X
b = a*vec3(1,1,-1)
bolt = bolt(a, b, dscrew)
bolts = [bolt.transform(rotate((i+0.5)*2*pi/8, Z))  for i in range(8)]

# build a shape to support the screws
screw_support = web([
	project(a, Z) + dscrew*X,
	a + neighscrew*X,
	b + neighscrew*X,
	project(b, Z) + dscrew*X,
	]).segmented()
screw_supports = revolution(screw_support)

exterior_shape = union(exterior_shell, screw_supports)

# put holes for screws
hole = cylinder(a+1*Z, a*vec3(1,1,-1)-1*Z, dscrew*0.55).flip()
holes = mesh.mesh([hole.transform(b.pose)  for b in bolts])
exterior = intersection(exterior_shape, holes)
sep = square(Axis(1.5*transmiter_rint*Z, Z), space_radius*4)

# join the interior and the exterior
interior = interior + extrusion(interior.frontiers(5,None), shell_thickness*Z).orient().flip()
interior.mergeclose()

part = intersection(exterior, interior).finish()

# the part constructed above is not feasible nor mountable, so we split it into mountable subsets
part_mid = intersection(part, sep + sep.flip().transform(mat3(1,1,-1))) .finish()
part_top = intersection(part, sep.flip()) .finish()
part_bot = part_top.flip().transform(mat3(1,1,-1))

# annotations to check the dimensions of what we generated
output1['bearing']['annotations'] = [
	note_radius(output1['bearing']['part'].group(2)),
	note_radius(output1['bearing']['part'].group(9)),
	note_distance_planes(output1['bearing']['part'].group(0), output1['bearing']['part'].group(3)),
	]
annotations = [
	note_distance(O, a, project=X, offset=20*Z),
	]



# the rest is only prospective from now

#option1 = (sbevelgear(10, 24, radians(70), helix_angle=radians(20), bore_height=0, bore_radius=bearing_radius)
#			.transform(translate(5*Z) * rotate(pi,X)))
top_stuff = Solid(part=part_top).transform(30*Z)


def sbevelgear(step, z, pitch_cone_angle, **kwargs):
	part = helical_bevel_gear(step, z, pitch_cone_angle, **kwargs)
	top = project(part.group(4).barycenter(), Z)
	bot = project(part.group(1).barycenter(), Z)
	return Solid(
		part = part .option(color=gear_color),
		summit = O,
		axis = Axis(top, -Z, interval=(0, length(top))),
		bot = Axis(bot, Z, interval=(0, length(top-bot))),
		)

option2 = (sbevelgear(8, 20, radians(70),  helix_angle=radians(20), bore_radius=bearing_radius, bore_height=0, resolution=('sqradm', 0.5))
			.transform(translate(50*Z) * rotate(pi,X)))

#option3 = Solid(part=gear(8, 22, 10, helix_angle=radians(20), bore_radius=space_radius, hub_height=0)).transform(-8*Z)

## export the parts to print
#io.write(part_top, '/tmp/differentiel-part-top.stl')
#io.write(part_mid, '/tmp/differentiel-part-mid.stl')
#io.write(output['bearing']['part'], '/tmp/bearing-15-24-5-shape.stl')
#io.write(output['gear']['part'], '/tmp/differentiel-output-gear.stl')
#io.write(transmiter['gear']['part'], '/tmp/differentiel-transmiter-gear.stl')

show([
	part_top, part_bot, part_mid,
	option2,
	bolts,
	output1, output2, transmiters,
	annotations,
	])
