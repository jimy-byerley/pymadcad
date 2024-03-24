from madcad import *
settings.primitives['curve_resolution'] = ('rad', 0.1)

nballs = 6
rball = 6/2
rsphere = 2.7*rball
helix = 0.2
amax = radians(40)
ecage = rball*0.4

rinmax = distance(rotateY(rsphere*X, -amax/2), rotateY(rsphere*Z, amax))
routmax = rsphere * cos(amax/2)


balls = [icosphere(p, rball).option(color=vec3(0.1, 0.2, 0.4))
	for p in regon(Axis(O,Z), rsphere, nballs).points]
guide_profile = web(Circle(Axis(rsphere*X, Z), rball)) .transform(scaledir(Y, 1.02))
guide = revolution(pi, Axis(O,Y), guide_profile, alignment=0.5).flip()

interior = intersection(
		icosphere(O, rsphere-ecage), 
		brick(width=2*rsphere*vec3(1, 1, sin(amax/2))),
		)
exterior = union(
		icosphere(O, rsphere+ecage), 
		intersection(
			cylinder(-(rsphere+rball)*Z, (rsphere+rball)*Z, 2*rball, fill=False),
			icosphere(O, rsphere+rball*1.1)),
		)
for i in range(nballs):
	interior = difference(interior, guide.transform(rotate(helix*(-1)**i, X)).transform(rotate(2*pi*i/nballs, Z)))
	exterior = union(exterior, guide.transform(rotate(-helix*(-1)**i, X)).transform(rotate(2*pi*i/nballs, Z)))
exterior = exterior.flip()
interior = interior.finish()


cage = revolution(2*pi, Axis(O,Z), wire([rsphere*vec3(cos(t), 0, sin(t))  for t in linrange(-0.75*amax, 0.75*amax, div=10)]))

window = parallelogram(2.3*rball*Y, 2*rball*Z, align=vec3(0.5), fill=False) .transform(rsphere*X)
bevel(window, [0, 1, 2, 3], ('width', rball))
cage = thicken(
	pierce(cage, 
		repeat(extrusion(rball*X, window, alignment=0.5), nballs, rotate(2*pi/nballs, Z))), 
	ecage, 
	alignment=0.5,
	)
cage = cage.option(color=vec3(0.5,0.3,0))


mesh.mesh(balls).transform(rotate(-amax/2, Y))
interior.transform(rotate(-amax, Y))


rin = stfloor(rinmax*0.7)
rout = rinmax
hext = stceil(0.8*rsphere)

interior = difference(interior, cylinder(-rsphere*Z, rsphere*Z, min(rsphere-rball*1.2, rin)))

#in_shape = revolution(2*pi, Axis(O,Z), wire([
#			vec3(rinmax, 0, sqrt((rsphere+rball)**2 - rinmax**2)*1.01),
#			vec3(rinmax, 0, 0),
#			]).transform(rotate(amax, Y)).flip())
#
#exterior = union(exterior, square(Axis(-rsphere*Z,Z), rsphere))
## inside shaft interface
#out_shape = revolution(2*pi, Axis(O,Z), wire([
#	vec3(min(rout, routmax), 0, -rsphere*sin(amax/2)),
##	vec3(max(routmax, rout), 0, -1.05*sqrt((rsphere+rball)**2 - max(routmax, rout)**2)),
#	vec3(rout, 0, -rsphere*1.01),
#	vec3(rout, 0, -rsphere-hext),
#	vec3(1.2*rout, 0, -rsphere-hext),
#	]).segmented().flip())
#exterior = intersection(exterior, out_shape + in_shape)
#exterior = intersection(
#			inflate(convexhull(exterior + icosphere(O, rsphere+rball)).orient(), rsphere*0.1).mergegroups(), 
#			expand(exterior, rsphere*0.5),
#			).finish()

# exterior screws interface
in_shape = revolution(2*pi, Axis(O,Z), wire([
			vec3(rinmax, 0, sqrt((rsphere+rball)**2 - rinmax**2)*1.1),
			vec3(rinmax, 0, 0),
			]).transform(rotate(amax, Y)).flip())
exterior = intersection(
	icosphere(O, rsphere*1.1+rball) + exterior,
	in_shape + in_shape.transform(scaledir(Z, -1)).flip(),
	)

dscrew = stceil(rsphere*0.2, 0.3)
rscrews = stceil(rsphere+0.8*rball+dscrew, 0.05)
hole = bolt_slot(rscrews*Y - rball*0.99*Z, rscrews*Y + rball*0.99*Z, dscrew)
support = extrusion(2*rball*Z, flatsurface(convexoutline(
	repeat(web(Circle(Axis(rscrews*Y,Z), 1.2*dscrew)), 3, rotate(2*pi/3, Z))
	+ web(Circle(Axis(O,Z), rsphere*1.1 + rball))
	)), alignment=0.5).flip()

e = pierce(support, exterior).islands()[0].transform(0.999)
exterior = union(exterior, e)
exterior = intersection(exterior, repeat(hole, 3, rotate(2*pi/3, Z)))

annotations = [
	note_distance(O, rscrews*Y, offset=1.5*rsphere*Z),
	note_radius(hole.group(2), offset=rsphere),
	note_radius(interior.group(9), offset=2.5*rsphere),
	note_radius(balls[0], offset=1.5*rsphere),
	]

io.write(exterior, '/tmp/birfield/exterior.stl')
io.write(interior, '/tmp/birfield/interior.stl')
io.write(cage, '/tmp/birfield/cage.stl')
