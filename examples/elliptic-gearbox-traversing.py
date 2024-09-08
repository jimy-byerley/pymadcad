from madcad import *
from madcad.gear import gearprofile
from madcad.joints import *
from functools import reduce
import operator
#settings.primitives['curve_resolution'] = ('rad', 0.2)
#settings.primitives['curve_resolution'] = ('sqradm', 0.4)
settings.primitives['curve_resolution'] = ('sqradm', 0.8)

def ellipsis_perimeter(a, b):
	return Circle(Axis(O,Z), a).mesh(resolution=('div', 128)).transform(scaledir(X, b/a)).length()



# ellipsis parameters based on underformed circle
rellipsis = 40    # underformed deformable radius
height = 30  # undeformed deformable height
ellipsis_teeth = 60  # number of teeth on the ellipsis
circle_teeth = ellipsis_teeth+2 # number of teeth on the circle
dscrew_ext = stceil(rellipsis*0.10, 0.3)
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
rcircle = rprimitive * circle_teeth / ellipsis_teeth

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
			repeat(
				gearprofile(teeth_step, circle_teeth, asymetry=teeth_height*0.05, alpha=radians(30)) 
					.transform(rotate(pi/2, Z)),
				circle_teeth, 
				rotate(2*pi/circle_teeth, Z),
				),
			1.5*hellipsis*Z, 
			alignment=0.5).flip()
ellipsis_teeth = extrusion(
			repeat(
				gearprofile(teeth_step, ellipsis_teeth, height=teeth_step*0.25, asymetry=-teeth_height*0.05, alpha=radians(30)) 
					.transform(rotate(pi/2, Z)),
				ellipsis_teeth, 
				rotate(2*pi/ellipsis_teeth, Z),
				),
			1.5*hellipsis*Z,
			alignment=0.5)
ellipsis_teeth.mergeclose()
ellipsis_teeth = intersection(
			ellipsis_teeth,
			revolution(web([
					Segment(vec3(rellipsis+thickness*0.5, 0, hellipsis*0.5-thickness*0.5), vec3(rellipsis+teeth_height, 0, hellipsis*0.4)),
					Segment(vec3(rellipsis+teeth_height, 0, -hellipsis*0.4), vec3(rellipsis+thickness*0.5, 0, -hellipsis*0.6)),
					])),
			)

# deformable part
deformable_profile = wire([
	vec3(rint - 2*thickness, 0, -height+hout*0.5 + thickness),
	vec3(rellipsis, 0, -height+hout*0.5 + thickness),
	vec3(rellipsis, 0, -hellipsis),
	vec3(rellipsis, 0, -hellipsis*0.7),
	vec3(rellipsis, 0, hellipsis*0.5),
	]).segmented()
filet(deformable_profile, [1], width=rint*0.15)
deformable = revolution(deformable_profile)
deformable = thicken(deformable, thickness)
deformable = union(deformable, ellipsis_teeth)

# output shaft interface
out_rint = stfloor(rint-1.6*dscrew_ext)
out_shoulder = revolution(wire([
	vec3(rint, 0, -height+hout*0.65),
	vec3(out_rint, 0, -height+hout*0.65),
	vec3(out_rint, 0, -height-hout*0.5),
	vec3(rint, 0, -height-hout*0.5),
	]).close().segmented().flip())
deformable = union(deformable, out_shoulder)
out_shape = thicken(revolution(wire([
	vec3(out_rint, 0, -height-hout*0.5),
	vec3(rint*1.1, 0, -height-hout*0.5),
	])), stceil(0.5*hout))

rbolts = stfloor(rint-0.8*dscrew_ext)
out_bolts = []
out_holes = Mesh()
for p in regon(Axis(O,Z), rbolts, 8):
	out_bolts.append(bolt(
				p+(-height-hout*1.)*Z, 
				p+(-height)*Z, 
				dscrew_ext,
				washera=True,
				nutb=False,
				))
	out_holes += cylinder(
				p+(-height-2*hout)*Z,
				p+(-height+hout)*Z,
				dscrew_ext*0.55,
				)
out_shape = difference(out_shape, out_holes)
deformable = difference(deformable, out_holes)


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
cage_profile = extrusion(
	flatsurface(wire([cage_profile1, cage_profile2]).close()).flip(), 
	3*rball*Z, 
	alignment=0.7)
cage_profile.mergeclose()

cage = repeat(cage_profile, nballs, rotate(2*pi/nballs, Z))
cage = difference(cage, inflate(balls, rball*0.1))

ellipsis_exterior = wire([
		vec3(rellipsis+thickness*0.5, 0, -hellipsis*0.5+thickness*0.5), 
		[vec3(rballs + rball*cos(t), 0, rball*sin(t))   for t in linrange(-radians(40), radians(40), div=10)],
		vec3(rellipsis+thickness*0.5, 0, hellipsis*0.5-thickness*0.5),
		])
deformable = union(deformable, revolution(ellipsis_exterior))

# wave generator
rgenerator = rellipsis - 2.2*rball
bgenerator = bellipsis - 2.2*rball
generator_thickness = rellipsis*0.1
generator_profile = wire([
		vec3(teeth_height*1.5, 0, -hellipsis*0.5), 
		vec3(rgenerator, 0, -hellipsis*0.5), 
		[vec3(rballs - rball*cos(t), 0, rball*sin(t))   for t in linrange(-radians(40), radians(40), div=10)],
		vec3(rgenerator, 0, hellipsis*0.5),
		vec3(rgenerator - generator_thickness, 0, hellipsis*0.5), 
		vec3(teeth_height*1.5, 0, 0), 
		]).flip()
generator = revolution(generator_profile) .transform(deform)
generator = difference(generator, cylinder(-hellipsis*Z, hellipsis*Z, bgenerator - generator_thickness))

in_rint = stfloor(bgenerator - generator_thickness - 2*dscrew_int, 0.05)
generator_interface_profile = wire([
	vec3(bgenerator - generator_thickness*0.2, 0, -hellipsis*0.1),
	vec3(bgenerator - generator_thickness*0.2, 0, -height+hout*0.5+3*thickness),
	vec3(out_rint, 0, -height+hout*0.5+3*thickness),
	vec3(in_rint, 0, -height+hout),
	vec3(in_rint, 0, -height+hout+2*dscrew_int),
	vec3(bgenerator - generator_thickness +thickness, 0, -height+hout+2*dscrew_int),
	vec3(bgenerator - generator_thickness +thickness, 0, -hellipsis*0.1),
	]).segmented()
filet(generator_interface_profile, [3], width=dscrew_int)
generator_interface = revolution(generator_interface_profile)
generator = union(generator, generator_interface)

in_bolts = []
in_bolts_slots = Mesh()
for p in regon(Axis((-height+hout+2*dscrew_int)*Z, Z), in_rint + dscrew_int, 4).points:
	in_bolts.append(bolt(p, p-dscrew_int*Z, dscrew_int, washera=True, nutb=False))
	in_bolts_slots += cylinder(p+0.1*dscrew_int*Z, p-2*dscrew_int*Z, 0.5*dscrew_int)
for p in regon(Axis((-height+hout+2*dscrew_int)*Z, Z), in_rint + dscrew_int, 4, alignment=(X+Y)).points:
	in_bolts_slots += cylinder(p+0.1*dscrew_int*Z, p-2*dscrew_int*Z, 0.5*stfloor(0.9*dscrew_int))
generator = difference(generator, in_bolts_slots)



# exterior
rext_in = stceil(rcircle+0.5*teeth_step + dscrew_int, 0.1)
rext_out = stceil(rext + 2*dscrew_ext, 0.05)

exterior_profile = wire([
	vec3(rext_out, 0, -height-0.7*hout),
	vec3(mix(rint, rext, 0.7), 0, -height-0.7*hout),
	vec3(mix(rint, rext, 0.7), 0, -height-0.5*hout),
	vec3(rext, 0, -height-0.5*hout),
	vec3(rext, 0, -height+0.5*hout),
	vec3(mix(rint, rext, 0.7), 0, -height+0.5*hout),
	vec3(mix(rint, rext, 0.7), 0, -height+0.7*hout),
	vec3(rcircle+0.5*teeth_step, 0, -height+0.7*hout),
	vec3(rcircle+0.5*teeth_step, 0, -0.6*hellipsis),
	vec3(rcircle-0.5*teeth_step, 0, -0.4*hellipsis),
	vec3(rcircle-0.5*teeth_step, 0, +0.4*hellipsis),
	vec3(rcircle+0.5*teeth_step, 0, +0.5*hellipsis),
	vec3(rcircle+0.5*teeth_step, 0, +0.6*hellipsis),
	vec3(rext_in, 0, +0.6*hellipsis),
	]).close().segmented()
filet(exterior_profile, [7,8], width=0.5*dscrew_ext)

exterior = revolution(exterior_profile)
exterior = intersection(exterior, circle_teeth)
exterior_in = thicken(revolution(wire([
	vec3(rext_in, 0, +0.6*hellipsis),
	vec3(in_rint, 0, +0.6*hellipsis),
	])), 0.2*hellipsis)

ext_out_bolts = []
ext_out_holes = Mesh()
for p in regon(Axis(O,Z), rext_out - dscrew_ext, 8):
	ext_out_bolts.append(bolt(
		p-0.7*hout*Z-height*Z,
		p+0.8*hellipsis*Z,
		dscrew_ext,
		washera=True,
		washerb=True))
	ext_out_holes += cylinder(p-2*height*Z, p+2*height*Z, 0.55*dscrew_ext)
for p in regon(Axis(O,Z), rext_out - dscrew_ext, 8, alignment=rotate(pi/16,Z)*X):
	ext_out_holes += cylinder(p-2*height*Z, p+2*height*Z, 0.55*stfloor(0.9*dscrew_ext))
exterior = difference(exterior, ext_out_holes)
exterior_in = difference(exterior_in, ext_out_holes)

holding_defs = [
	(Axis(rbolts*rotate(pi/8,    Z)*X + (-height-1*hout)*Z, -Z),  dscrew_ext, 1.7*hout, 0.5*hout),
	(Axis(rbolts*rotate(pi/8+pi, Z)*X + (-height-1*hout)*Z, -Z),  dscrew_ext, 1.7*hout, 0.5*hout),	
	(Axis((rext_out-dscrew_ext)*rotate(pi/8,    Z)*X + (-height-0.7*hout)*Z, -Z),  dscrew_ext, 1.7*hout, 0.7*hout),	
	(Axis((rext_out-dscrew_ext)*rotate(pi/8+pi, Z)*X + (-height-0.7*hout)*Z, -Z),  dscrew_ext, 1.7*hout, 0.7*hout),	
	(Axis((rext_out-dscrew_ext)*rotate(pi/8,    Z)*X + (0.8*hellipsis)*Z, Z),  dscrew_ext, 1.7*hout, 0),
	(Axis((rext_out-dscrew_ext)*rotate(pi/8+pi, Z)*X + (0.8*hellipsis)*Z, Z),  dscrew_ext, 1.7*hout, 0),
	]
holding_slots = Mesh()
holding_screws = []
for a, d, h, p in holding_defs:
	holding_screws.append(screw(d, 0.9*h, head='flat').place((Revolute, Axis(O,Z), a)))
	holding_slots += screw_slot(Axis(a[0]-a[1]*0.1, a[1]), d, hole=p, screw=h-p, flat=True)
deformable = intersection(deformable, holding_slots)
out_shape = intersection(out_shape, holding_slots)
exterior = intersection(exterior, holding_slots)
exterior_in = intersection(exterior_in, holding_slots)

exterior_out = difference(exterior, square(Axis(-height*Z, -Z), 2*rext_out)) .finish()
exterior_mid = difference(exterior, square(Axis(-height*Z, +Z), 2*rext_out)) .finish()


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
# balls are not deformed but moved
balls = reduce(operator.add, [
		ball.transform(p)   
		for p in regon(Axis(O,Z), rballs, nballs)
					.transform(deform)
					.points
		]) .option(color=vec3(0.1, 0.2, 0.4))


annotations = [
	note_radius(balls.islands()[0], offset=height),
	note_distance(
		O, 
		in_bolts[0]['screw']['axis'].transform(in_bolts[0]['screw'].pose).origin,
		offset=height*Z),
	note_distance(
		O, 
		noproject(out_bolts[0]['screw']['axis'].transform(out_bolts[0]['screw'].pose).origin, Z),
		offset=-2*height*Z),
	]

