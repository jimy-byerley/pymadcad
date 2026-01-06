from madcad import *
from madcad.offseting import expand
from . import visualcheck

@visualcheck
def test_expand():
	results = []

	m1 = pyramid(0.5*Y, Wire([
		vec3(0.2209, 0, 0.2095),
		vec3(0.4065, 0, 0.1058),
		vec3(0.3941, 0, -0.0477),
		vec3(0.3609, 0, -0.02904),
		vec3(0.2323, 0, -0.008296),
		vec3(0.2074, 0, -0.06222),
		vec3(0.2385, 0, -0.1597),
		vec3(0.2416, 0, -0.1867),
		mix(vec3(0.2416, 0, -0.1867),  vec3(-0.01348, 2.25e-08, -0.1887), 0.5),
		vec3(-0.01348, 0, -0.1887),
		vec3(0.001037, 0, 0.09333),
		vec3(0.01141, 0, 0.1079),
		]).close().segmented())
	m2 = pyramid(0.3*Y + vec3(0.5348, 0., -0.09662), Wire([
		vec3(0.3665, 0., 0.04015),
		vec3(0.3428, 0., -0.08222),
		vec3(0.2916, 0., -0.1232),
		vec3(0.1931, 0., -0.1974),
		vec3(0.1905, 0., -0.3126),
		vec3(0.3723, 0., -0.3843),
		vec3(0.5413, 0., -0.3664),
		vec3(0.7768, 0., 0.2301),
		vec3(0.6616, 0., 0.2327),
		vec3(0.6686, 0., 0.12),
		vec3(0.6165, 0., 0.09289),
		vec3(0.4787, 0., 0.1077),
		]).close().segmented().flip())
	m3 = union(m1, m2)

	exp = expand(m1, 0.5)
	exp.check()
	assert exp.issurface()
	results.append(exp.transform(mat3(2)))

	exp = expand(m3, 0.2)
	exp.check()
	assert exp.issurface()
	results.append(exp.transform(mat3(2)))

	sphere = pierce(
				icosphere(O, 1), 
				brick(min=vec3(0), max=vec3(2)) .flip(),
				)
	exp = expand(sphere, 0.5)
	exp.check()
	assert exp.issurface()
	results.append(exp)

	sphere = pierce(
				uvsphere(O, 1), 
				brick(min=vec3(0), max=vec3(2)) .flip(),
				)
	exp = expand(sphere, 0.5)
	exp.check()
	assert exp.issurface()
	results.append(exp)

	cylinder = revolution(wire([vec3(0,1,0), vec3(1,1,0)]), Axis(O,X), pi) .flip()
	exp = expand(cylinder, 0.5)
	exp.check()
	assert exp.issurface()
	results.append(exp)


	x = 0
	for i,r in enumerate(results):
		w = r.box().size.x
		results[i] = r.transform((x+w)*X)
		x += 2*w

	return results
