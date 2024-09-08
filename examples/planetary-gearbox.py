from madcad import *
from madcad.gear import *


color_gear = vec3(0.2, 0.3, 0.4)


zsun = 10
zcrown = 30
zplanet = (zcrown - zsun)//2
rcrown = 30

axis = Axis(O,Z)
step = 2*pi*rcrown / zcrown
crown = Circle(axis, rcrown)
sun = Circle(axis, rcrown/zcrown*zsun)
orbit = mix(crown.radius, sun.radius, 0.5)
corbit = Circle(axis, orbit)
planet = Circle(Axis(orbit*X,Z), rcrown/zcrown*zplanet)
hole = stfloor(sun.radius - 0.5*step)
#nplanet = int((2*pi*orbit) / (3*planet.radius))
nplanet = 4
height = 1.2*crown.radius
helix = radians(20)
out_dscrew = int(stfloor(rcrown*0.08, 0.2)*2)

#sun = [
#	gear(step, zsun, height, helix_angle=helix),
#	gear(step, zsun, height, helix_angle=-helix) .transform(height*Z) .transform(rotate(pi/zsun,Z)),
#	]
#planet = gear(step, zplanet, height, helix_angle=-helix) .transform(planet.axis.origin)

def vprofile(radius, helix=radians(30)):
	helix = tan(helix)
	return [
		translate(t*height*Z) * rotate(sin(t*pi)/pi/radius*height*helix, Z)  
		for t in linrange(0, 1, div=10)]

def helix(height, radius, helix=radians(20)):
	helix = tan(helix)
	return [
		translate(t*height*Z) * rotate(t*height*helix/radius, Z)  
		for t in linrange(0, 1, div=4)]

gap = 0.05*crown.radius
sun_teeth = extrans(repeataround(gearprofile(step, zsun, asymetry=0.)), helix(height*0.6, -sun.radius)) .transform(rotate(pi/zsun, Z))
ext_teeth = extrans(repeataround(gearprofile(step, zcrown, asymetry=0.1)), helix(height*0.5, crown.radius)).flip()
pla_teeth = extrans(repeataround(gearprofile(step, zplanet, asymetry=0.3)), helix(height*0.5, planet.radius)) .transform(planet.axis.origin)
sun_teeth += sun_teeth.transform(scaledir(Z,-1)).flip()
pla_teeth += pla_teeth.transform(scaledir(Z,-1)).flip()
ext_teeth += ext_teeth.transform(scaledir(Z,-1)).flip()
cut = revolution(Softened([
	planet.axis.origin + gap*Z + (planet.radius+0.5*step)*X,
	planet.axis.origin + 0.5*gap*Z + (planet.radius-0.4*step)*X,
	planet.axis.origin - 0.5*gap*Z + (planet.radius-0.4*step)*X,
	planet.axis.origin - gap*Z + (planet.radius+0.5*step)*X,
	]), planet.axis)
pla_gear = intersection(pla_teeth, cut + brick(center=planet.axis.origin, width=(height-2*gap)*Z+4*planet.radius*(X+Y)))
pla_dscrew = int(stfloor(0.7*planet.radius/2, 0.4)*2)
pla_gear = intersection(pla_gear, bolt_slot(
	planet.axis.origin+(height*0.5-pla_dscrew)*Z, 
	planet.axis.origin-(height*0.5-pla_dscrew)*Z, 
	pla_dscrew,
	pla_dscrew,
	)).option(color=color_gear)

pla_bolt = bolt(
	planet.axis.origin+height*0.2*Z, 
	planet.axis.origin-height*0.2*Z, 
	pla_dscrew,
	)

plas = [Solid(gear=pla_gear, bolt=pla_bolt) .transform(rotate(x,Z)*rotatearound(x*zcrown/zplanet, planet.axis))
		for x in linrange(0, 2*pi, div=nplanet-1)]

sun_gear = intersection(sun_teeth.finish(), revolution(wire([
	-0.45*height*Z,
	-0.45*height*Z + sun.radius*0.8*X,
	-0.35*height*Z + sun.radius*2*X,
	]).segmented().flip(), sun.axis))

interlayer = stceil(height*0.4)
output_bearings = []
input_bearings = []
holes = Mesh()
for p in regon(axis, orbit, nplanet):
	thickness = 1
	output_bearings.append( slidebearing(pla_dscrew, interlayer, thickness) .transform(p-(height*0.5)*Z) )
	input_bearings.append( slidebearing(pla_dscrew, interlayer, thickness) .transform(p+(height*0.5+interlayer)*Z) )
	holes += cylinder(p+2*height*Z, p-2*height*Z, 0.5*pla_dscrew+0.95*thickness)

#output = thicken(revolution(2*pi, sun.axis, wire([
#	-0.5*height*Z + hole*X,
#	-0.5*height*Z + (orbit + 0.5*planet.radius)*X,
#	])), 1.5*out_dscrew)
#output = difference(output, output_holes)

bearing_rint = stceil(orbit + 0.5*planet.radius)
bearing_rext = stceil(bearing_rint + 1.5*out_dscrew)
bearing_height = stceil(bearing_rext - bearing_rint)
bearing_center = -(0.5*height+0.8*bearing_height)*Z
out_bearing = bearing(2*bearing_rint, 2*bearing_rext, bearing_height) .transform(bearing_center)

sealing = 0.5
shell = cylinder(+height*0.5*Z, -height*0.5*Z, crown.radius+0.5*step, fill=False)
input = thicken(flatsurface(Circle(Axis(0.5*height*Z, Z), orbit+0.5*planet.radius)), -interlayer).flip()
input = difference(
			union(input, sun_gear),
			holes + cylinder(height*Z, -height*Z, hole),
			)
output = revolution(wire([
	-0.5*height*Z + hole*X,
	-0.5*height*Z + mix(bearing_rint, bearing_rext, 0.3)*X,
	bearing_center + mix(bearing_rint, bearing_rext, 0.3)*X + 0.5*bearing_height*Z,
	bearing_center + bearing_rint*X + 0.5*bearing_height*Z,
	bearing_center + bearing_rint*X - 0.5*bearing_height*Z,
	bearing_center + hole*X - 0.5*bearing_height*Z,
	]).close().segmented())

shell = revolution(
	web([
		-(0.5*height-gap)*Z + (crown.radius-0.5*step)*X,
		-0.5*height*Z + (crown.radius+0.5*step)*X,
		bearing_center + bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,
		bearing_center + bearing_height*0.5*Z + bearing_rext*X,
		bearing_center - bearing_height*0.5*Z + bearing_rext*X,	
		bearing_center - bearing_height*0.5*Z + mix(bearing_rint, bearing_rext, 0.7)*X,	
		bearing_center - bearing_height*0.7*Z + mix(bearing_rint, bearing_rext, 0.7)*X,	
		bearing_center - bearing_height*0.7*Z + 2*bearing_rext*X,	
		]).segmented().flip()
	+web([
		(0.5*height-gap)*Z + (crown.radius-0.5*step)*X,
		0.5*height*Z + (crown.radius+0.5*step)*X,
		0.55*height*Z + (orbit+0.5*planet.radius+sealing)*X,
		(0.5*height+2*out_dscrew)*Z + (orbit+0.5*planet.radius+sealing)*X,
		(0.5*height+2*out_dscrew)*Z + 2*bearing_rext*X,
		]).segmented())

out_rscrew = mix(bearing_rint, bearing_rext, 0.8)+out_dscrew
out_bolt = bolt(
	bearing_center - bearing_height*0.6*Z + out_rscrew*X,
	0.5*height*Z + out_rscrew*X,
	out_dscrew,
	washera=True,
	)

outline = convexoutline(
	repeataround(web(Circle(Axis(out_rscrew*X, Z), 1.2*out_dscrew)), 4, axis)
#	+ repeataround(
#		web(Circle(Axis(out_rscrew*X, Z), 0.8*out_dscrew))
#			.transform(rotate(2*out_dscrew/rcrown,Z)), 
#		4, axis)
	+ web(Circle(axis, 1.05*bearing_rext)),
	)
shell = intersection(shell, 
		ext_teeth
		+ extrusion(outline, 4*height*Z, alignment=0.5).orient())

#interface = cylinder(bearing_center + bearing_height*Z, bearing_center - bearing_height*Z, bearing_rext + 2*out_dscrew)
