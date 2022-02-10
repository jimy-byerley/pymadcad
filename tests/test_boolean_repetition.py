from madcad import *
settings.primitives['curve_resolution'] = ('rad', pi/16)

z = vec3(0,0,1)

def cylinder(axis, radius, height):
	return extrusion(
		-height*axis[1],
		flatsurface(wire(Circle(axis, radius))),
		)

# boolean on normal meshes with some crossing edges
diff = difference(
	cylinder((vec3(0),z), 2, 2),
	cylinder((vec3(2,0,1),z), 1, 4),
	)
print(repr(diff))

# weired geometry for the next test
C0 = wire(Interpolated([
		vec3(-2.132,	-1.937,	2.138),
		vec3(-1.124,	-0.983,	1.075),
		vec3(-2.323,	-0.3509,	-0.08454),
		vec3(-2.243,	0.3852,	-1.075),
		vec3(-0.6619,	1.687,	-2.476),
		vec3(-0.7226,	2.028,	-2.959)]))
C1 = wire(Softened([
		vec3(-3.794, 0.8014, 3.072),
		vec3(-3.285, -0.4081, 2.613),
		vec3(-2.182, -1.67, 3.035),
		vec3(-1.638, -3.067, 2.385),
		vec3(-2.275, -4.168, 0.4291)]))
m = tube(C1, C0)


diff, frontier = boolean.cut_mesh(diff, m)	# mark first intersections
diff, frontier = boolean.cut_mesh(diff, m)	# remark the same intersections
res = difference(diff, m)		# boolean on top of already marked intersections

res.check()
assert res.isenvelope()

show([res])
