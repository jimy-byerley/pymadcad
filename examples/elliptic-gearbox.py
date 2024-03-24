from madcad import *
from madcad.gear import gearprofile
from functools import reduce
import operator
settings.primitives['curve_resolution'] = ('rad', 0.2)
#settings.primitives['curve_resolution'] = ('sqradm', 0.4)

def ellipsis_perimeter(a, b):
	return Circle(Axis(O,Z), a).mesh(resolution=('div', 128)).transform(scaledir(X, b/a)).length()



# ellipsis parameters based on underformed circle
rellipsis = 40    # underformed deformable radius
height = 30  # undeformed deformable height
ellipsis_teeth = 60  # number of teeth on the ellipsis
circle_teeth = ellipsis_teeth+2 # number of teeth on the circle
dscrew_ext = stceil(rellipsis*0.15, 0.3)
dscrew_int = stceil(rellipsis*0.08, 0.3)

# the deformed shape keeps the same neutral fiber length
aellipsis = rellipsis * circle_teeth/ellipsis_teeth  # ellipsis max
#bellipsis = fbisect(lambda bellipsis: ellipsis_perimeter(aellipsis, bellipsis) > 2*pi*rellipsis, aellipsis, 0.5*aellipsis)
def predicate(bellipsis):
	return ellipsis_perimeter(aellipsis, bellipsis) > 2*pi*rellipsis
bellipsis = fbisect(predicate, aellipsis, 0.5*aellipsis)  # ellipsis min

hellipsis = rellipsis*0.3   # teeth depth
thickness = rellipsis*0.02  # thickness of the deformable part
rprimitive = rellipsis*(1 + 1.2*pi/ellipsis_teeth)  # primitive radius of the teeth on the ellipsis
teeth_step = 2*pi*rprimitive / ellipsis_teeth  # common step of the teeth profile
teeth_height = teeth_step

# output bearing
rint = stceil(rellipsis*0.8)  # output bearing interior radius
rext = stceil(rellipsis*1.05)  # output bearing exterior radius
hout = stceil(rext-rint)    # output bearing height
# ellipsis bearing
rball = stceil(hellipsis*0.4)
rball = 6/2


# transform function
def deform(p):
	ratio = smoothstep(-height+hout*0.5, -hellipsis*0.5, p.z)
	return p + normalize(noproject(p, Z)) * mix(vec3(0), vec3(bellipsis, aellipsis, rellipsis) - rellipsis, ratio)



out_bearing = bearing(2*rint, 2*rext, hout) .transform(-height*Z)

# teeth
circle_teeth = extrusion(
			1.5*hellipsis*Z, 
			repeat(
				gearprofile(teeth_step, circle_teeth, asymetry=teeth_height*0.05, alpha=radians(30)) 
					.transform(rotate(pi/2, Z)),
				circle_teeth, 
				rotate(2*pi/circle_teeth, Z),
				),
			alignment=0.5)
ellipsis_teeth = extrusion(
			1.5*hellipsis*Z,
			repeat(
				gearprofile(teeth_step, ellipsis_teeth, height=teeth_step*0.25, asymetry=-teeth_height*0.05, alpha=radians(30)) 
					.transform(rotate(pi/2, Z)),
				ellipsis_teeth, 
				rotate(2*pi/ellipsis_teeth, Z),
				),
			alignment=0.5)
ellipsis_teeth.mergeclose()
ellipsis_teeth = intersection(
			ellipsis_teeth,
			revolution(2*pi, Axis(O,Z), web([
					Segment(vec3(rellipsis+thickness*0.5, 0, hellipsis*0.5-thickness*0.5), vec3(rellipsis+teeth_height, 0, hellipsis*0.4)),
					Segment(vec3(rellipsis+teeth_height, 0, -hellipsis*0.4), vec3(rellipsis+thickness*0.5, 0, -hellipsis*0.6)),
					])),
			)


# deformable part
deformable_profile = wire([
	vec3(rint - thickness, 0, -height-hout*0.5),
	vec3(rint - thickness, 0, -height+hout*0.5 + thickness),
	vec3(rellipsis, 0, -height+hout*0.5 + thickness),
	vec3(rellipsis, 0, -hellipsis),
	vec3(rellipsis, 0, -hellipsis*0.7),
	vec3(rellipsis, 0, hellipsis*0.5),
	]).segmented()
bevel(deformable_profile, [2], ('width', rint*0.15))
deformable = revolution(2*pi, Axis(O,Z), deformable_profile)
deformable = thicken(deformable, thickness)
deformable = union(deformable, ellipsis_teeth)

# output shaft interface
out_shoulder = revolution(2*pi, Axis(O,Z), wire([
	vec3(rint*1.1, 0, -height+hout*0.65),
	vec3(rint*0.6, 0, -height+hout*0.65),
	vec3(rint*0.6, 0, -height-hout*0.5),
	vec3(rint, 0, -height-hout*0.5),
	]).segmented().flip())
deformable = union(deformable, out_shoulder)
out_shape = extrusion(-0.3*hout*Z, revolution(2*pi, Axis(O,Z), wire([
	vec3(rint*0.6, 0, -height-hout*0.5),
	vec3(rint*1.1, 0, -height-hout*0.5),
	])))

out_bolts = [bolt(
				p+(-height-hout*0.8)*Z, 
				p+(-height+hout*0.65)*Z, 
				dscrew_ext,
				washera=True,
				washerb=True,
				)  for p in regon(Axis(O,Z), stfloor(rint*0.8), 4)]
out_holes = [cylinder(
				p+(-height-hout)*Z,
				p+(-height+hout)*Z,
				dscrew_ext*0.55,
				) for p in regon(Axis(O,Z), stfloor(rint*0.8), 4).points]
out_holes += [cylinder(
				p+(-height-hout)*Z,
				p+(-height+hout)*Z,
				0.55*stfloor(dscrew_ext*0.9, 0.3),
				) .transform(rotate(pi/4, Z))
			 for p in regon(Axis(O,Z), stfloor(rint*0.8), 4).points]
out_shape = difference(out_shape, mesh.mesh(out_holes))
deformable = difference(deformable, mesh.mesh(out_holes))


# ellipsis bearing with custom cage
ball = icosphere(O, rball)
nballs = int(floor(2*pi*rellipsis / (3.3*rball)))
rballs = rellipsis - 1.05*rball
balls = reduce(operator.add, [ball.transform(p*rballs)   for p in regon(Axis(O,Z), 1, nballs).points]).option(color=vec3(0.1, 0.2, 0.4))

cage_profile1 = Wire([
			vec3(cos(t), sin(t), 0) 
			* ( rballs + 0.55*rball * mix(1, -cos(t*nballs), 0.35) )    
			for t in linrange(0, 2*pi/nballs, div=20)])
cage_profile2 = Wire([
			vec3(cos(t), sin(t), 0) 
			* ( rballs - 0.55*rball * mix(1, -cos(t*nballs), 0.35) )    
			for t in linrange(0, 2*pi/nballs, div=20)]).flip()
cage_profile = extrusion(3*rball*Z, flatsurface(wire([cage_profile1, cage_profile2]).close()).flip(), alignment=0.7)
cage_profile.mergeclose()

cage = repeat(cage_profile, nballs, rotate(2*pi/nballs, Z))
cage = difference(cage, inflate(balls, rball*0.1))

ellipsis_exterior = wire([
		vec3(rellipsis+thickness*0.5, 0, -hellipsis*0.5+thickness*0.5), 
		[vec3(rballs + rball*cos(t), 0, rball*sin(t))   for t in linrange(-radians(40), radians(40), div=10)],
		vec3(rellipsis+thickness*0.5, 0, hellipsis*0.5-thickness*0.5),
		])
deformable = union(deformable, revolution(2*pi, Axis(O,Z), ellipsis_exterior))

# wave generator
rgenerator = rellipsis - 2.2*rball
generator_thickness = rellipsis*0.1
generator_profile = wire([
		vec3(teeth_height*1.5, 0, -hellipsis*0.5), 
		vec3(rgenerator, 0, -hellipsis*0.5), 
		[vec3(rballs - rball*cos(t), 0, rball*sin(t))   for t in linrange(-radians(40), radians(40), div=10)],
		vec3(rgenerator, 0, hellipsis*0.5),
		vec3(rgenerator - 0.5*generator_thickness, 0, hellipsis*0.5), 
		vec3(rgenerator - generator_thickness, 0, 0), 
		vec3(teeth_height*1.5, 0, 0), 
		]).flip()
generator = revolution(2*pi, Axis(O,Z), generator_profile) .transform(deform)
generator = difference(generator, cylinder(-hellipsis*Z, hellipsis*Z, rgenerator - generator_thickness - dscrew_int*2.5))
in_bolts = [
	bolt(p, p-0.5*hellipsis*Z, dscrew_int)
	for p in regon(Axis(O,Z), stfloor(rgenerator - generator_thickness - dscrew_int*1.2), 4, alignment=X+Y).points
	]
in_holes = mesh.mesh([
	cylinder(p+hellipsis*Z, p-hellipsis*Z, dscrew_int*0.55)
	for p in regon(Axis(O,Z), stfloor(rgenerator - generator_thickness - dscrew_int*1.2), 4, alignment=X+Y).points
	])
generator = difference(generator, in_holes)

# exterior
shoulder = hout*0.2
rexterior = stceil(aellipsis*1.3 + 1.1*dscrew_ext, 0.05)
rinner = max(aellipsis*1.08+teeth_height*1.1, rext+shoulder)
exterior_profile = wire([
	vec3(aellipsis, 0, -hellipsis*0.4),
	vec3(aellipsis+teeth_height*1.1, 0, -hellipsis*0.6),
	vec3(aellipsis+teeth_height*1.1, 0, -height+hout*0.5+2*shoulder),
	vec3(rext-shoulder, 0, -height+hout*0.5+shoulder),
	vec3(rext-shoulder, 0, -height+hout*0.5),
	vec3(rext, 0, -height+hout*0.5),
	vec3(rext, 0, -height-hout*0.5),
	vec3(rinner, 0, -height-hout*0.5),

	vec3(rinner, 0, hellipsis*0.6-dscrew_ext),
	vec3(rexterior + dscrew_ext, 0, hellipsis*0.6-dscrew_ext),
	vec3(rexterior + dscrew_ext, 0, hellipsis*0.6),
	vec3(aellipsis+teeth_height*1.2, 0, hellipsis*0.6),
	vec3(aellipsis, 0, +hellipsis*0.4),
	]).segmented().flip()
bevel(exterior_profile, [8], ('width', dscrew_ext))
chamfer(exterior_profile, [7], ('width', 0.5*(rinner - rext)))
exterior = revolution(2*pi, Axis(O,Z), exterior_profile)
exterior = intersection(exterior, circle_teeth.flip())

exterior_bolts_placement = regon(Axis(hellipsis*0.6*Z,Z), rexterior, 6).points
exterior_bolts = [bolt(
			p+dscrew_ext*0.5*Z,
			p-dscrew_ext*2*Z,
			dscrew_ext,
			washera=True,
			washerb=True,
			)
		for p in exterior_bolts_placement]
exterior_holes = mesh.mesh((
		[cylinder(
			p+dscrew_ext*Z,
			p-dscrew_ext*3*Z,
			dscrew_ext*0.55,
			) 
			for p in exterior_bolts_placement]
		+ [cylinder(
			p+dscrew_ext*Z,
			p-dscrew_ext*3*Z,
			0.55*stceil(dscrew_ext*0.9, 0.3),
			) .transform(rotate(pi/12, Z))
			for p in exterior_bolts_placement]
		))
exterior = difference(exterior, exterior_holes)

# the hat is what is fixing the motor on the in side of the gearbox
hat = thicken(revolution(2*pi, Axis(O,Z), wire([
	vec3(rexterior + dscrew_ext, 0, hellipsis*0.6),
	vec3(aellipsis*1.1, 0, hellipsis*0.6),
	vec3(aellipsis*1, 0, hellipsis*0.6 + dscrew_int),
	vec3(bellipsis*0.655, 0, hellipsis*0.6 + dscrew_int),
	]).segmented()), dscrew_int*0.5)
hat = difference(hat, exterior_holes)

hat_screws_placement = regon(
		Axis((hellipsis*0.6 + dscrew_int)*Z, Z), 
		stfloor(aellipsis - 1.2*dscrew_int, 0.2), 
		4,
		).points
hat_screws = [screw(dscrew_int, dscrew_int*2)
			.transform(translate(p) * rotate(pi,X))
			for p in hat_screws_placement]
hat_holes = mesh.mesh([cylinder(
			p+dscrew_int*Z,
			p-dscrew_int*Z,
			dscrew_int*0.55,
			) 
			for p in hat_screws_placement])
hat = difference(hat, hat_holes)

# the butt the part holding the out bearing at the bottom
butt_profile = wire([
	vec3(rext-hout*0.3, 0, -height-hout*0.5),
	vec3(rinner*1.02, 0, -height-hout*0.5),
	vec3(rinner*1.02, 0, hellipsis*0.6-dscrew_ext),
	vec3(rexterior + dscrew_ext, 0, hellipsis*0.6-dscrew_ext),
	]).segmented()
bevel(butt_profile, [1], ('width', hout*0.5))
bevel(butt_profile, [2], ('width', dscrew_ext))
butt = thicken(revolution(2*pi, Axis(O,Z), butt_profile), hout*0.2)
butt = difference(butt, exterior_holes)



# export parts to print
#io.write(deformable, '/tmp/deformable.stl')
#io.write(exterior, '/tmp/exterior.stl')
#io.write(cage, '/tmp/cage.stl')
#io.write(generator, '/tmp/generator.stl')
#io.write(out_shape, '/tmp/out_shape.stl')
#io.write(butt, '/tmp/butt.stl')
#io.write(hat, '/tmp/hat.stl')


# apply deformation to deformable parts
cage = cage.transform(deform) .option(color=vec3(0.5,0.3,0))
deformable = deformable.transform(deform) .option(color=vec3(0.2, 0.3, 0.4))
#ellipsis_teeth = ellipsis_teeth.transform(deform)
# balls are not deformed but moved
#balls = balls.transform(deform)
balls = reduce(operator.add, [
		ball.transform(p)   
		for p in regon(Axis(O,Z), rballs, nballs)
					.transform(deform)
					.points
		]) .option(color=vec3(0.1, 0.2, 0.4))

annotations = [
	note_radius(balls.islands()[0], offset=height),
	note_leading(hat.group(4), text="hat shape depends on the motor fixture\nthis hat is designed for motors with\n a front screwable shape\n\nTODO: make it the same part as exterior"),
	note_leading(O, offset=height*Z, text="gearbox is sealed by the motor\n if the motor is hollowshaft,\n it must seal it through the generator"),
	note_leading(-height*Z, offset=-height*Z, text="gearbox is sealed by closing this hole,\n or linking it to the generator\n\nTODO: provide sealing"),
	note_distance(
		O, 
		in_bolts[0]['screw']['axis'].transform(in_bolts[0]['screw'].pose).origin,
		offset=height*Z),
	note_distance(
		O, 
		noproject(out_bolts[0]['screw']['axis'].transform(out_bolts[0]['screw'].pose).origin, Z),
		offset=-2*height*Z),
	note_distance(
		O, 
		noproject(exterior_bolts[0]['screw']['axis'].transform(exterior_bolts[0]['screw'].pose).origin, Z),
		offset=-2*height*Z),
	note_leading(out_bearing['part'].group(2).transform(out_bearing.pose), text="{:.3g}x{:.3g}x{:.3g}".format(2*rint, 2*rext, hout)),
	]

color_bolts = vec3(0.2)
out_bolts[0]['screw']['part'].option(color=color_bolts)
out_bolts[0]['nut']['part'].option(color=color_bolts)
out_bolts[0]['washera']['part'].option(color=color_bolts)
out_bolts[0]['washerb']['part'].option(color=color_bolts)
exterior_bolts[0]['screw']['part'].option(color=color_bolts)
exterior_bolts[0]['nut']['part'].option(color=color_bolts)
exterior_bolts[0]['washera']['part'].option(color=color_bolts)
exterior_bolts[0]['washerb']['part'].option(color=color_bolts)
