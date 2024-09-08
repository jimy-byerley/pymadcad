'''
	This is an example to generate a bearing from scratch. Of course it's even better to use the provided function in module `madcad.standard`
'''

from madcad import *

# main generation dimensions
dint, dext, h = 16, 35, 11

# convenient variables
rint = dint/2
rext = dext/2
c = 0.05*h
w = 0.5*h
e = 0.15*(dext-dint)

# interior and exterior rings
axis = Axis(O,Z)
interior = Wire([
	vec3(rint+e, 0,	w), 
	vec3(rint, 0, w),
	vec3(rint,	0,	-w),
	vec3(rint+e, 0,	-w), 
	]) .segmented() .flip()
filet(interior, [1, 2], radius=c, resolution=('div',1))

exterior = Wire([
	vec3(rext-e,	0, -w),
	vec3(rext, 0, -w),
	vec3(rext, 0, w),
	vec3(rext-e,	0, w),
	]) .segmented() .flip()
filet(exterior, [1,2], radius=c, resolution=('div',1))


# detailed interior
rb = (dint + dext)/4	# balls path radius
rr = 0.75*(dext - dint)/4	# balls radius

hr = sqrt(rr**2 - (rb-rint-e)**2)	# half balls inprint width on the interior ring
interior += wire(ArcCentered((rb*X,-Y), vec3(rint+e, 0, hr), vec3(rint+e, 0, -hr)))
exterior += wire(ArcCentered((rb*X,-Y), vec3(rext-e, 0, -hr), vec3(rext-e, 0, hr)))
interior.close()
exterior.close()

nb = int(0.7 * pi*rb/rr)  # number of balls that fits in
balls = repeat(icosphere(rb*X, rr), nb, angleAxis(radians(360)/nb, Z))
balls.option(color=vec3(0,0.1,0.2))

# simple interior
#interior = (
#	  Wire([
#		exterior[-1], 
#		exterior[-1]+c*Z, 
#		interior[0]+c*Z, 
#		interior[0]]) .segmented()
#	+ interior
#	+ Wire([
#		interior[-1], 
#		interior[-1]-c*Z, 
#		exterior[0]-c*Z, 
#		exterior[0]]) .segmented()
#	)

part = revolution(web([exterior, interior]), axis, 4)
part.mergeclose()

# cage
sides = pierce(
	union(
		repeat(
			icosphere(rb*X, 2*c+rr), 
			nb, angleAxis(2*pi/nb,Z),
			), 
		square((c*Z,Z), dext),
		),
	( extrusion(Circle((-h*Z,Z), rb-rr*0.4, resolution=('rad',0.1)), 2*h*Z)
	+ extrusion(Circle((-h*Z,Z), rb+rr*0.4, resolution=('rad',0.1)), 2*h*Z) .flip()
		),
	)
cage = thicken(
			  sides 
			+ sides.transform(scaledir(Z,-1)) .flip(), 
			c) .option(color=vec3(0.5,0.3,0))



# display the results
show([axis, part, cage, balls])
