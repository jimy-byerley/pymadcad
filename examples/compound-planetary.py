from madcad import *
from madcad.gear import *
from madcad.joints import *
from madcad.text import *

def repeataround(pattern, repetitions=None, axis=Axis(O,Z), angle=2*pi):
	if repetitions is None:
		if isinstance(pattern, Mesh):	indices = (i for f in pattern.faces for i in f )
		elif isinstance(pattern, Web):	indices = (i for e in pattern.edges for i in e)
		elif isinstance(pattern, Wire): indices = iter(pattern.indices)

		x,y,z = dirbase(axis[1], align=pattern.points[next(indices)] - axis[0])
		lower, upper = 0, 0
		for p in indices:
			t = atan2(dot(pattern.points[p]-axis[0], y), dot(pattern.points[p]-axis[0], x))
			lower = min(lower, t)
			upper = max(upper, t)
		repetitions = round(angle / (upper - lower))
	return repeat(pattern, repetitions, rotatearound(angle/repetitions, axis))

def revolution(shape, axis=Axis(O,Z), angle=2*pi, alignment=0., resolution=None):
	return generation.revolution(angle, axis, shape, alignment, resolution)

def extrusion(shape, direction=Z, alignment=0.):
	return generation.extrusion(direction, shape, alignment)

def vprofile(radius, helix=radians(30)):
	helix = tan(helix)
	return [
		translate(t*height*Z) * rotate(sin(t*pi)/pi/radius*height*helix, Z)  
		for t in linrange(0, 1, div=10)]

def helix(height, radius, angle=radians(20)):
	helix = tan(angle)
	return [
		translate(t*height*Z) * rotate(t*height*helix/radius, Z)  
		for t in linrange(0, 1, div=4)]

def bearing(dint, dext=None, h=None, **kwargs):
	return Solid(
		rint = dint/2,
		rext = dext/2,
		height = h,
		placeholder = standard.bearing(dint, dext, h, **kwargs),
		annotations = [
			note_distance(O, dint/2*X),
			note_distance(dext/2*X - h/2*Z, dext/2*X + h/2*Z, offset=h*X),
			note_distance(dint/2*X + h/2*Z, dext/2*X + h/2*Z, offset=h*Z),
		])

# teeth generation functions
def vgear(step, teeth, height, angle, **kwargs):
	profile = repeataround(gearprofile(step, teeth, **kwargs))
	return mesh.mesh([
		extrans(profile, helix(height*0.5, step*teeth/(2*pi), angle)),
		extrans(profile, helix(-height*0.5, -step*teeth/(2*pi), angle)).flip(),
		]).mergegroups().finish()

def hgear(step, teeth, height, angle, **kwargs):
	print('gearprofile', step, teeth, kwargs)
	profile = repeataround(gearprofile(step, teeth, **kwargs), teeth)
	return (
		extrans(profile, helix(+0.5*height, step*teeth/(2*pi), angle))
		+ extrans(profile, helix(-0.5*height, step*teeth/(2*pi), angle)).flip()
		).mergegroups().finish()

def spurgear(step, teeth, height, **kwargs):
	return extrusion(repeataround(gearprofile(step, teeth, **kwargs)), height*Z, alignment=0.5)

def circular_screwing(axis, radius, height, dscrew, diameters=1, div=8, hold=False):
	''' standard holes and screws for screwing a circular perimeter '''
	holes = Mesh()
	bolts = []
	x,y,z = dirbase(axis.direction)
	a = axis.origin + radius*x
	b = axis.origin + radius*x + height*z
	gap = 0.1*dscrew*z
	enlarge = 1.05
#	bolts.append(bolt(a, b, dscrew))
	for i in range(diameters):
		holes += cylinder(a-gap, b+gap, enlarge*stfloor(0.8**i * dscrew)/2) .transform(rotatearound(i*1.4*dscrew/radius, axis))
	holes = repeataround(holes, 8, axis).flip()
	if hold:
		angle = 1.7*dscrew/radius
		holes += repeataround(screw_slot(Axis(a+gap,-z), dscrew, 
					screw=height-hold, 
					hole=hold, 
					flat=True), 2) .transform(rotatearound(-angle, axis))
		bolts.append(screw(dscrew, height, head='flat')
			.place((Revolute, Axis(O,Z), Axis(a,-z)))
			.transform(rotatearound(-angle, axis)))
		bolts.append(screw(dscrew, height, head='flat')
			.place((Revolute, Axis(O,Z), Axis(a,-z)))
			.transform(rotatearound(pi-angle, axis)))
	return holes, bolts

def grooves(r, n=16, align=0.5, angle=radians(40)):
	''' coupling grooves profile '''
	h = r/n / tan(angle)
	def profile(t, min=-1, max=1):
		return (r+(0.5-align)*h+h*sin(n*t)) * vec3(cos(t), sin(t), 0)
	return wire([ profile(t, -1, 0.7)
		for t in linrange(0, 2*pi, div=12*n, end=False) ]).close()

color_gear = vec3(0.2, 0.3, 0.4)

settings.primitives['curve_resolution'] = ('sqradm', 0.4)
#settings.primitives['curve_resolution'] = ('sqradm', 0.8)

## determine teeth number targeting the desired ratio
#target = 1/160
#zmin = 7
#step_min = 1
#rext = 50
#zcrown_max = floor(2*pi*rext / step_min)
#
#best = inf
#params = None
#for zplanet_out in range(zmin, zcrown_max//2):
#	for zplanet_in in range(zmin, zcrown_max//2):
#		for zcrown_out in range(2*zplanet_out, zcrown_max):
#			for zsun in range(zplanet_in, zcrown_max):
#				ratio = zsun / (2*zcrown_out) * ((zcrown_out - zplanet_out) / (zsun + zplanet_in) - zplanet_out / zplanet_in)
#				score = abs(ratio - target)
#				if score < best:
#					params = (ratio, zplanet_out, zplanet_in, zcrown_out, zsun)
#
#ratio, zplanet_out, zplanet_in, zcrown_out, zsun = params
#out_step = rext * 2*pi/zcrown_out
#in_step = out_step * (zcrown_out - zplanet_out) / (zsun + zplanet_in)

annotations = {}

#zcrown_out = 21
#zcrown_in = 13+2*8
#zplanet_in = 8
#zplanet_out = 7

#zcrown_out = 24
#zcrown_in = 25 + 2*15
#zplanet_in = 15
#zplanet_out = 7

#zcrown_out = 24
#zcrown_in = 26 + 2*14
#zplanet_in = 14
#zplanet_out = 7

#zcrown_out = 23
#zcrown_in = 26 + 2*13
#zplanet_in = 13
#zplanet_out = 7

#zcrown_out = 62
#zcrown_in = 31 + 2*15
#zplanet_in = 15
#zplanet_out = 16

zcrown_out = 51
zcrown_in = 44 + 2*9
zplanet_in = 9
zplanet_out = 8


# global parameters
rext = 50
height = stceil(0.2*rext)
dscrew_out = stceil(rext*0.1)
dscrew_in = stceil(rext*0.08)
shell_thickness = stceil(0.03 * rext)
nplanets = floor(pi*(zcrown_out - zplanet_out) / (zplanet_out + pi))
zsun = zcrown_in - 2*zplanet_in

axis = Axis(O,Z)

# the bearing will constrain the mounting of the design
bearing_height = stceil(rext*0.2)
bearing_rext = stfloor(rext-dscrew_out*0.6)
bearing_rint = bearing_rext-bearing_height
bearing_center = +0.7*bearing_height*Z
bearing = bearing(2*bearing_rint, 2*bearing_rext, bearing_height) .transform(bearing_center)

# out_step is computed such as the out crown fit in the bearing out ring
out_step = mix(bearing_rint, bearing_rext, 0.7) / (1 + zcrown_out/(2*pi))
# in_step is such as the gearbox can be assembled
in_step = out_step * (zcrown_out - zplanet_out) / (zcrown_in - zplanet_in)

# define marks in the assembly
support_height = max(stceil(0.9*min(in_step, out_step)), 0.2*height)
out_center = bearing_center + bearing_height*0.6*Z + 1*shell_thickness*Z + support_height*Z + height*0.5*Z
in_center = out_center + height*Z + support_height*Z

# place gears primitive circles
carrier = Circle(Axis(in_center, Z), in_step*(zcrown_in-zplanet_in)/(2*pi))
planet_out = Circle(Axis(out_center+carrier.radius*X,Z), out_step*zplanet_out/(2*pi))
crown_out = Circle(Axis(out_center,Z), carrier.radius+planet_out.radius)
planet_in = Circle(Axis(in_center + carrier.radius*X,Z), in_step*zplanet_in/(2*pi))
crown_in = Circle(Axis(in_center,Z), carrier.radius + planet_in.radius)
sun = Circle(Axis(in_center,Z), carrier.radius - planet_in.radius)

gears_placeholder = [carrier, planet_out, crown_out, planet_in, crown_in, sun]

gap = 0.1*support_height
hole = min(
	carrier.radius - planet_out.radius - 0.6*out_step - shell_thickness, 
	sun.radius - 0.5*in_step - shell_thickness,
	)

in_ext = Circle(
	Axis(in_center + height*0.5*Z + shell_thickness*Z + 1.2*dscrew_in*Z, Z), 
	stfloor(carrier.radius + planet_in.radius - in_step*0.6 - 1.2*dscrew_in),
	)
out_ext = Circle(axis, rext)
out_int = Circle(axis.transform(-dscrew_out*Z), stfloor(bearing_rint - 1.2*dscrew_in))
in_placeholder = [
	in_ext, Circle(in_ext.axis.transform(in_ext.radius*X), dscrew_in/2),
	Circle(in_ext.axis, hole), Circle(in_ext.axis.transform((hole-1.2*dscrew_in)*X), dscrew_in/2),
	]
out_placeholder = [
	out_ext, Circle(axis.transform(rext*X), dscrew_out/2), 
	out_int, Circle(out_int.axis.transform(out_int.radius*X), dscrew_out/2),
	]

# generate all gears teeth
vangle = radians(20)
hangle = radians(10)
sun_teeth = hgear(in_step, zsun, height+2*gap, -hangle, offset=-0.1*in_step, asymetry=-0.1) .transform(rotate(pi/zsun-pi*zplanet_in/zsun, Z)) .transform(sun.center)
crown_in_teeth = hgear(in_step, zcrown_in, height+4*gap, hangle, offset=0.1*in_step, asymetry=0.1).flip() .transform(crown_in.center)
crown_out_teeth = hgear(out_step, zcrown_out, height+2*support_height, hangle, offset=0, asymetry=0.2).transform(crown_out.center).flip()
planet_in_teeth = hgear(in_step, zplanet_in, height+2*gap, hangle, offset=0.1*in_step, asymetry=0.0).transform(planet_in.center) .option(color=color_gear)
planet_out_teeth = hgear(out_step, zplanet_out, height+2*gap, hangle, offset=0, asymetry=0.2).transform(planet_out.center) .option(color=color_gear)

# place as much planets as possible, their phasing is constrained by the sun and and crown
max_planets = floor(pi*(zcrown_out - zplanet_out) / (zplanet_out + pi))
perimeter = 0
phase = 0
planets_phases = [phase]
planet_sector = 2*pi/max_planets
for i in range(zcrown_in):
	new = pi/(zcrown_in - zplanet_in)*i
	if new > pi - planet_sector:	
		break
	if new - phase > planet_sector:
		phase = new
		planets_phases.append(phase)

planets_phases += [phase+pi  for phase in planets_phases]
planets = [(
			planet_in_teeth.transform(rotatearound(-phase*zcrown_in/zplanet_in, planet_in.axis))
			+ planet_out_teeth.transform(rotatearound(-phase*zcrown_out/zplanet_out, planet_out.axis))
		).transform(rotate(phase,Z)).option(color=color_gear)
		for phase in planets_phases]


# support prevending the planets from collapsing under the pressure put by the torque of out crown 
rsupport = carrier.radius - planet_out.radius + 0.4*out_step
planets_support = wire([
	(support_height+0.5*height-gap)*Z + hole*X,
	(support_height+0.5*height-gap)*Z + rsupport*X,
	(0.5*height+2*gap)*Z + rsupport*X,
	(0.5*height+2*gap)*Z + (carrier.radius - planet_out.radius - out_step*0.5)*X,
	])
planets_support = revolution(
			(planets_support + planets_support.transform(scaledir(Z,-1)).flip()) 
			.transform(crown_out.center).segmented().close())

# assemble the planet
planet_body = revolution(web([
	wire([
		crown_out.center - (0.5*height+support_height-gap)*Z + carrier.radius*X, 
		crown_out.center - (0.5*height+support_height-gap)*Z + rsupport*X, 
		crown_out.center - (0.5*height+gap)*Z + rsupport*X, 
		crown_out.center - (0.5*height-gap)*Z + (carrier.radius-planet_out.radius-0.5*out_step)*X,
		]),
	wire([
		crown_out.center + (0.5*height-gap)*Z + (carrier.radius-planet_out.radius-0.5*out_step)*X,
		crown_out.center + (0.5*height+gap)*Z + rsupport*X,
		crown_in.center - (0.5*height+gap)*Z + rsupport*X, 
		crown_in.center - (0.5*height-gap)*Z + (sun.radius-0.5*out_step)*X
		]),
	wire([
		crown_in.center + (0.5*height-gap)*Z + (sun.radius-0.5*out_step)*X, 
		crown_in.center + (0.5*height)*Z + rsupport*X,
		crown_in.center + (0.5*height)*Z + carrier.radius*X,
		]), 
	]).segmented().flip(), planet_in.axis)
planet = intersection(planet_body, planet_in_teeth + planet_out_teeth).option(color=color_gear)

in_coupling = extrans(grooves(stfloor(sun.radius*0.9), 24, 1), [
	translate(-height*0.6*Z),
	translate(+height*0.4*Z),
	translate(+height*0.7*Z) * scale(vec3(1.08)),
	]).flip().transform(in_center)

# shape the teeth side
crown_out_teeth = union(crown_out_teeth, revolution(wire([
	- out_step*0.6*X - 2*gap*Z,
	+ out_step*0.6*X,
	]) .transform(out_center - height*0.5*Z + crown_out.radius*X)))
crown_out_teeth = intersection(crown_out_teeth, revolution(wire([
	- out_step*0.8*X - gap*Z,
	+ out_step*0.8*X + gap*Z,
	]) .transform(out_center + height*0.5*Z + crown_out.radius*X)))

crown_in_teeth = intersection(crown_in_teeth, revolution(wire([
	+ in_step*0.8*X - gap*Z,
	- in_step*0.6*X + gap*Z,
	]) .transform(in_center - height*0.5*Z + crown_in.radius*X)))
crown_in_teeth = union(crown_in_teeth, revolution(wire([
	+ in_step*0.6*X - gap*Z,
	- in_step*0.8*X + 2*gap*Z,
	]) .transform(in_center + height*0.5*Z + crown_in.radius*X)))

sun_teeth = union(sun_teeth, revolution(wire([
	+ in_step*0.8*X +2*gap*Z,
	- in_step*0.6*X -gap*Z,
	]) .transform(in_center + height*0.5*Z + sun.radius*X)))
sun_teeth = intersection(sun_teeth, revolution(wire([
	+ in_step*0.6*X +gap*Z,
	- in_step*0.6*X -gap*Z,
	]) .transform(in_center - height*0.5*Z + sun.radius*X)))

in_coupling = union(in_coupling, revolution(wire([
	- sun.radius*0.3*X,
	+ sun.radius*0.*X,
	]) .transform(in_center - height*0.4*Z + sun.radius*X)))

# shape the main parts
input = revolution(wire([
	in_center + height*0.5*Z + sun.radius*X + in_step*0.6*X,
	in_center + height*0.5*Z + sun.radius*X + in_step*0.6*X + shell_thickness*Z,

	in_center + height*0.5*Z + min(hole, sun.radius*0.8)*X + shell_thickness*Z,
	in_center - height*0.5*Z + min(hole, sun.radius*0.8)*X,

	in_center - height*0.5*Z + sun.radius*X + in_step*X,
	]).segmented().flip())
input = intersection(input, (sun_teeth + in_coupling))

out = revolution(wire([
	crown_out.center + height*0.5*Z + (crown_out.radius + out_step*0.5 + shell_thickness)*X + gap*(Z-X),
	crown_out.center + height*0.5*Z + (crown_out.radius + out_step*0.5 + shell_thickness)*X,
	crown_out.center - height*0.5*Z - support_height*Z + (crown_out.radius + out_step*0.5)*X + shell_thickness*X,

	bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
	bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.)*X,
	bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.)*X,

	bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
	bearing_center - (bearing_height*0.7 + dscrew_out)*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
	bearing_center - (bearing_height*0.7 + dscrew_out)*Z + (bearing_rint - dscrew_out*2)*X,
	bearing_center + bearing_height*0.4*Z + hole*X,
	bearing_center + bearing_height*0.7*Z + hole*X,
	bearing_center + bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.2)*X,

	crown_out.center - height*0.5*Z - support_height*Z + crown_out.radius*X,
	crown_out.center - height*0.5*Z + crown_out.radius*X,
	crown_out.center - height*0.5*Z + gap*Z + (crown_out.radius - in_step*0.6)*X,
	]).segmented())
out = intersection(out, crown_out_teeth)

shell = wire([
	crown_in.center - height*0.5*Z + crown_in.radius*X,
	
	crown_in.center - height*0.5*Z + (crown_out.radius + out_step*0.6 + 2*shell_thickness)*X,
	bearing_center + bearing_height*0.7*Z + (crown_out.radius + out_step*0.6 + 2*shell_thickness)*X,
	
	bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
	bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 1.)*X,
	bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 1.)*X,
	bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
	bearing_center - bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.7)*X,

	bearing_center - bearing_height*0.7*Z + (rext + 1.*dscrew_out)*X,
	bearing_center + bearing_height*1*Z + (rext + 1.*dscrew_out)*X,

	bearing_center + bearing_height*1*Z + (crown_out.radius + out_step*0.5 + 3*shell_thickness)*X,
	crown_in.center - height*0.5*Z + (crown_out.radius + out_step*0.5 + 3*shell_thickness)*X,
	crown_in.center + height*0.5*Z + (crown_in.radius + in_step*0.5 + shell_thickness)*X,
	
	in_ext.center + (in_ext.radius + 1.5*dscrew_in)*X + shell_thickness*(X+Z),
	in_ext.center + (in_ext.radius - 1.2*dscrew_in)*X +shell_thickness*Z,
	in_ext.center + (in_ext.radius - 1.2*dscrew_in)*X,
	in_ext.center + (in_ext.radius + 1.5*dscrew_in)*X,
	in_center + (in_ext.radius + 1.5*dscrew_in)*X,
	]).segmented().flip()
bevel(shell, [13], ('width', 0.5*dscrew_out))
bevel(shell, [12, 11], ('width', 0.5*height))
bevel(shell, [10], ('width', 0.5*dscrew_out))
bevel(shell, [1], ('width', 0.5*height))
shell = intersection(revolution(shell), crown_in_teeth)

# put screws to hold everything
in_ext_screw = bolt(
	in_ext.center + in_ext.radius*X,
	in_ext.center + in_ext.radius*X + 1*dscrew_in*Z,
	dscrew_in,
	nutb=False)

holes, bolts_out_ext = circular_screwing(
	Axis(out_ext.axis.origin + bearing_height*1.7*Z, -Z), 
	out_ext.radius, bearing_height*1.7, dscrew_out, diameters=2, hold=bearing_height*0.8)
shell = intersection(shell, holes)
holes, bolts_out_int = circular_screwing(out_int.axis, out_int.radius, bearing_height*1.7, dscrew_out, diameters=2, hold=True)
out = intersection(out, holes)
holes, bolts_in_ext = circular_screwing(in_ext.axis, in_ext.radius, dscrew_in, dscrew_in, diameters=2)
shell = intersection(shell, holes)

# cut the main parts into assebmlable parts
shell_in = intersection(shell, square(Axis(bearing_center + bearing_height*0.7*Z, -Z), 1.5*rext))
shell_mid = intersection(shell, 
	square(Axis(bearing_center + bearing_height*0.7*Z, Z), 1.5*rext)
	+ square(Axis(bearing_center + bearing_height*0.2*Z, -Z), 1.5*rext)
	)
shell_out = intersection(shell, square(Axis(bearing_center + bearing_height*0.2*Z, Z), 1.5*rext))

out_receiver = intersection(out, square(Axis(bearing_center - bearing_height*0.499*Z, -Z), 2*bearing_rint))
out_int = intersection(out, square(Axis(bearing_center - bearing_height*0.499*Z, Z), 2*bearing_rint))

# rework planets to have the appropriate dephasing between in and out
planets = [(
			planet_in_teeth.transform(rotatearound(-phase*zcrown_in/zplanet_in, planet_in.axis))
			+ planet_out_teeth.transform(rotatearound(-phase*zcrown_out/zplanet_out, planet_out.axis))
		).transform(rotate(phase,Z)).option(color=color_gear)
		for phase in planets_phases]
planets = []
for i, phase in enumerate(planets_phases):
	in_half = pi/zplanet_in
	out_half = pi/zplanet_out
	in_transform = rotatearound((-phase*zcrown_in/zplanet_in - in_half) % (2*in_half) + in_half, planet_in.axis)
	out_transform = rotatearound((-phase*zcrown_out/zplanet_out - out_half) % (2*out_half) + out_half, planet_out.axis)
	def t(p):
		if p.z > mix(in_center.z, out_center.z, 0.5):	
			return in_transform * p
		else: 
			return out_transform * p
	
	# put an index on each planet
	label = extrusion(
			text(str(i), 
				align=('center', 'center'), 
				fill=True, 
				font='PuristaBold',
				resolution=('rad', 0.4),
				size=0.5*zplanet_in*in_step/pi,
				)
				.transform(rotate(pi/2,Z))
				.transform(planet_in.center + height*0.5*Z), 
			zplanet_in*in_step*0.02*Z,
			alignment=0.5)
	labeled = intersection(planet.transform(t), label.flip()).finish()
	planets.append(labeled.transform(rotate(phase,Z)).option(color=color_gear))

for i, planet in enumerate(planets):
	io.write(planets[i], '/tmp/compound-planetary/planet-{}.stl'.format(i))
io.write(input, '/tmp/compound-planetary/input.stl')
io.write(out_receiver, '/tmp/compound-planetary/out_receiver.stl')
io.write(planets_support, '/tmp/compound-planetary/planets_support.stl')
io.write(out_int, '/tmp/compound-planetary/out_int.stl')
io.write(shell_in, '/tmp/compound-planetary/shell_in.stl')
io.write(shell_out, '/tmp/compound-planetary/shell_out.stl')
io.write(shell_mid, '/tmp/compound-planetary/shell_mid.stl')
