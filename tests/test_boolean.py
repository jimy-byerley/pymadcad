from copy import deepcopy
from pnprint import nprint

from madcad import *
from madcad.boolean import pierce, boolean, difference, cut_mesh, pierce_web, boolean_web
from madcad.generation import brick
from . import visualcheck

m1 = brick(width=vec3(2))
m2 = (deepcopy(m1)
		.transform(vec3(0.5, 0.3, 0.4))
		.transform(quat(0.7*vec3(1,1,0)))
		)

@visualcheck
def test_pierce_mesh(subtests):
	results = []
	for side in (False, True):
		with subtests.test(side=side):
			r = pierce(m1, m2, side)
			r.check()
			assert r.issurface()
			results.append(Solid(body=r))
			
			r = pierce(m2, m1, side)
			r.check()
			assert r.issurface()
			results.append(Solid(body=r))
	for i, result in enumerate(results):
		results[i] = result.transform(translate(4*i*Z))
	return results

@visualcheck
def test_boolean_mesh(subtests):
	results = []
	for sidea in (False, True):
		for sideb in (False, True):
			with subtests.test(sides=(sidea, sideb)):
				r = boolean(m1, m2, (sidea, sideb))
				r.check()
				assert r.isenvelope()
				results.append(Solid(body=r))
	for i, result in enumerate(results):
		results[i] = result.transform(translate(4*i*Z))
	return results

@visualcheck
def test_boolean_web(subtests):
	wa = web(Circle((O,Z), 1))

	others = {
		0: web(wire([vec3(-1, 0.5, 0), vec3(1, 0.2, 0)])),
		1: web(Softened([vec3(-2, 0, 0), vec3(-1, 0.5, 0), vec3(1, 0.2, 0), vec3(2, 3, 0)])),
		2: web(wire([vec3(-2,-0.5,0), vec3(2,-0.5,0), vec3(2,0.5,0), vec3(-2,0.5,0)]).close()),
		#3: web(wire([vec3(-1,-0.5,0), vec3(1,-0.5,0), vec3(1,0.5,0), vec3(-1,0.5,0)]).close()),    # that one will fail until overlapping geometries are handled
		4: web([
			Segment(vec3(-1,-0.5,0), vec3(1,-0.5,0)), 
			Segment(vec3(1,0.5,0), vec3(-1,0.5,0)),
			]),
		}

	results = []
	for i, wb in others.items():
		with subtests.test(wb=i):
			r = pierce_web(wa, wb)
			r.check()
			assert r.isline()
			results.append(Solid(content=r, wb=wb))
			
			r = boolean_web(wa, wb, (True, False))
			r.check()
			assert r.isline()
			results.append(Solid(content=r))
			
			r = boolean_web(wa, wb, (False, True))
			r.check()
			assert r.isline()
			results.append(Solid(content=r))

	for i, result in enumerate(results):
		results[i] = result.transform(translate(i*Z))

	return results

@visualcheck
def test_boolean_web_mesh():
	mesh = extrusion(Circle((O,Z),1), Z, alignment=0.5)

	others = {
		0: web(wire([vec3(-1, 0.5, 0), vec3(1, 0.2, 0)])),
		1: web(Softened([vec3(-2, 0, 0), vec3(-1, 0.5, 0), vec3(1, 0.2, 0), vec3(2, 3, 0)])),
		2: web(wire([vec3(-2,-0.5,0), vec3(2,-0.5,0), vec3(2,0.5,0), vec3(-2,0.5,0)]).close()),
		#3: web(wire([vec3(-1,-0.5,0), vec3(1,-0.5,0), vec3(1,0.5,0), vec3(-1,0.5,0)]).close()),	# that one will fail until overlapping geometries are handled
		4: web([
			Segment(vec3(-1,-0.5,0), vec3(1,-0.5,0)), 
			Segment(vec3(1,0.5,0), vec3(-1,0.5,0)),
			]),
		}

	results = []
	for i, w in others.items():
		nprint('* w={} '.format(i))
		
		r = pierce(w, mesh, False)
		r.check()
		assert r.isline()
		results.append(Solid(content=r, mesh=mesh))
		
		r = pierce(w, mesh, True)
		r.check()
		assert r.isline()
		results.append(Solid(content=r, mesh=mesh))

	for i, result in enumerate(results):
		results[i] = result.transform(translate(2*i*Z))

	return results

		
@visualcheck
def test_sidecases():
	z = vec3(0,0,1)
	cut_tool = icosphere(vec3(0), 1, resolution=("div", 1))
	plane = square(Axis(vec3(0), z), width=4)
	mesh, web = cut_mesh(plane, cut_tool)
	
	web.check()
	assert web.isloop()
	mesh.check()
	assert mesh.issurface()
	
	return [mesh, web]

@visualcheck
def test_edgecrossing():
	# 2 bricks intersecting
	b1 = brick(width=vec3(2))
	b2 = b1.transform(vec3(1, -0.5, 1))

	br = difference(b1, b2)
	br.check()
	assert br.isenvelope()

	# cylinders intersecting
	o = vec3(0)
	z = vec3(0,0,1)

	def cylinder(axis, radius, height):
		return extrusion(
			flatsurface(wire(Circle(axis, radius, resolution=('rad', pi/16)))),
			-height*axis[1],
			)

	cyl = difference(
		cylinder((o,z), 2, 2),
		cylinder((vec3(2,0,1),z), 1, 4),
		)
	cyl.check()
	assert cyl.isenvelope()

	return [cyl]
	
@visualcheck
def test_repetition():
	def cylinder(axis, radius, height):
		return extrusion(
			flatsurface(wire(Circle(axis, radius))),
			-height*axis[1],
			)

	# boolean on normal meshes with some crossing edges
	z = vec3(0,0,1)
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


	diff, frontier = cut_mesh(diff, m)	# mark first intersections
	diff, frontier = cut_mesh(diff, m)	# remark the same intersections
	res = difference(diff, m)		# boolean on top of already marked intersections

	res.check()
	assert res.isenvelope()
	
	return [res]

