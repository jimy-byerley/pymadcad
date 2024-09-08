from madcad import *
from madcad.gear import *
from madcad.joints import *

# bevelgear with annotations
def sbevelgear(step, z, pitch_cone_angle, **kwargs):
	part = bevelgear(step, z, pitch_cone_angle, **kwargs)
	top = project(part.group(4).barycenter(), Z)
	bot = project(part.group(1).barycenter(), Z)
	return Solid(
		part = part .option(color=gear_color),
		summit = O,
		axis = Axis(top, -Z, interval=(0, length(top))),
		bot = Axis(bot, Z, interval=(0, length(top-bot))),
		annotations = [
			note_radius(part.group(part.groupnear(top+z*step*X))),
			note_radius(part.group(part.groupnear(mix(top, bot, 0.5)))),
			],
		)

# bolt with annotations
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
			screw = rscrew.place((Revolute, rscrew['axis'], Axis(a-thickness*dir*int(washera), -dir))), 
			nut = rnut.place((Revolute, rnut['top'], Axis(b+thickness*dir*int(washerb), -dir))),
			w1 = rwasher.place((Revolute, rwasher['top'], Axis(b, -dir))),
			w2 = rwasher.place((Revolute, rwasher['top'], Axis(a, dir))),
			)

		

settings.resolution = ('sqradm', 0.5)

transmiter_angle = pi/6
transmiter_z = 8
gear_step = 6
output_radius = 5
gear_color = vec3(0.2, 0.3, 0.4)
shell_thickness = 1
dscrew = 3
helix_angle = 0 #radians(-20)

axis_z = round(transmiter_z/tan(transmiter_angle))
transmiter_rint = stceil((transmiter_z * gear_step / (2*pi) - 0.6*gear_step) * (0.5 + 0.2*sin(transmiter_angle)))

# mechanism
# interface shafts
bearing_height = stceil(output_radius)
bearing_radius = stceil(output_radius*2.5)
out_gear = sbevelgear(gear_step, axis_z, pi/2-transmiter_angle, 
				bore_radius=output_radius, 
				bore_height=1.2*bearing_height,
				helix_angle = helix_angle,
				)
output = Solid(
	gear = out_gear,
	bearing = bearing(stceil(output_radius*1.5*2), bearing_radius*2, bearing_height)
				.transform(out_gear['axis'].origin - 0.5*bearing_height*Z),
	)
output1 = output
output2 = output.transform(rotate(pi,Y))

# internal transmission shafts
transmiter_axis_thickness = stceil(transmiter_rint*0.4)
transmiter_washer_thickness = stceil(transmiter_rint*0.2)
transmiter_gear = sbevelgear(gear_step, transmiter_z, transmiter_angle, 
				bore_height=0, 
				bore_radius=transmiter_rint*1.05,
				helix_angle = -helix_angle,
				)
transmiter = Solid(
		gear = transmiter_gear,
		bearing = slidebearing(
				(transmiter_rint-transmiter_axis_thickness)*2, 
				stceil(distance(transmiter_gear['axis'].origin, transmiter_gear['bot'].origin) + 2*transmiter_rint), 
				transmiter_axis_thickness,
				) .transform(translate(transmiter_gear['bot'].origin) * rotate(pi,X)),
		washer = washer(
				stceil(transmiter_rint*2), 
				stceil(transmiter_rint*1.8*2), 
				transmiter_washer_thickness,
				) .transform(transmiter_gear['axis'].origin),
		).transform(rotate(pi/2,Y))

transmiter_amount = ceil(axis_z / (1.5*transmiter_z/pi))
transmiters = [transmiter.transform(rotate(i*2*pi/transmiter_amount,Z))  for i in range(transmiter_amount)]

space_radius = transmiter_z*gear_step/(2*pi) / sin(transmiter_angle) + transmiter_washer_thickness

# interior shell
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

r = length(transmiter['gear']['axis'].origin)
h = transmiter_rint
w = r + transmiter_washer_thickness*1.5
interior_transmision = revolution(Wire([
		2*r*X + h*Z,
		w*X + h*Z,
		w*X + 2.5*h*Z,
		w*X + 2.5*h*Z + h*(X+Z),
		]).flip().segmented(),
	Axis(O,X))
interior_transmisions = repeat(interior_transmision, transmiter_amount, rotate(2*pi/transmiter_amount, Z))

interior_space = union(
				icosphere(O, space_radius),
				cylinder(
					out_gear['axis'].origin, 
					out_gear['axis'].origin*vec3(1,1,-1), 
					bearing_radius*1.05, 
					fill=False),
				).flip()

interior_shell = union(interior_space, interior_out)
interior = union(interior_shell, interior_transmisions.group({0,1})) + interior_transmisions.group({2,3,4,5})

# asymetrical exterior shell
exterior_shell = inflate(interior_shell.flip(), shell_thickness)

# interface shape and botls
mount_thickness = 4*shell_thickness
neighscrew = 1.4*dscrew
rscrew = stceil(max(bearing_radius + 1.2*neighscrew,  space_radius + dscrew*0.5 + shell_thickness*0.5), precision=0.1)
a = out_gear['axis'].origin + rscrew*X + shell_thickness*Z
b = a - mount_thickness*Z
bolt = bolt(a, b, dscrew, False, False)
bolts = [bolt.transform(rotate((i+0.5)*2*pi/8, Z))  for i in range(8)]

screw_support = revolution(wire(
	Wire([
		project(a, Z),
		a + 1.3*dscrew*X,
		]).segmented(),
	Softened([
		b + 1.3*dscrew*X,
		project(b, Z) + space_radius*X,
		space_radius*X,
		]),
	))
slot = cylinder(b+1e-2*Z, noproject(b, Z), neighscrew).flip()
slots = mesh.mesh([slot.transform(b.pose)  for b in bolts])
head = intersection(screw_support, slots)

l = length(transmiter['gear']['axis'].origin) + 2*transmiter_rint
transmiter_back = revolution(Wire([
	l*Z,
	l*Z + transmiter_rint*1.5*X,
	rotate(pi/6, Y) * space_radius*Z,
	]).segmented()) .transform(rotate(pi/2, Y))

exterior_shell = union(
			exterior_shell, 
			mesh.mesh([ 
				transmiter_back.transform(rotate(i*2*pi/transmiter_amount, Z))  
				for i in range(transmiter_amount) ]))
exterior = union(head, exterior_shell)

interior.mergeclose()
part = intersection(interior, exterior).finish()

# part decomposition of the shell
split = revolution(Wire([
	a + 2*neighscrew*X - 2*shell_thickness*Z,
	project(a, Z) + bearing_radius*X + 2*shell_thickness*X - 2*shell_thickness*Z,
	project(a, Z) + bearing_radius*X - mount_thickness*Z,
	project(a, Z) + bearing_radius*X - mount_thickness*Z - shell_thickness*(X+Z),
	]).segmented()).finish()
part_top = intersection(part, split)
part_bot = intersection(part, split.flip())


# annotations
transmiters[0]['bearing']['annotations'] = [
	note_radius(transmiter['bearing']['part'].group(1)),
	note_distance_planes(*transmiter['bearing']['part'].group(2).islands()),
	]
output1['bearing']['annotations'] = [
	note_radius(output1['bearing']['part'].group(2)),
	note_radius(output1['bearing']['part'].group(9)),
	note_distance_planes(output1['bearing']['part'].group(0), output1['bearing']['part'].group(3)),
	]
annotations = [
	note_distance(O, a, project=X, offset=15*Z),
	]

show([
	part_top, part_bot,
	output1, output2, transmiters,
	bolts,
	])
