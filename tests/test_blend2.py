from madcad import *
from madcad.blending import *

a = wire(Softened([
		vec3(-0.8411, 0.4827, -0.2212),
		vec3(-0.1341, 0.4284, 0.2713),
		vec3(0.4169, -0.164, 0.1753),
		vec3(0.7683, -0.0219, 0.5677),
		vec3(0.6404, 0.3006, 0.7514)]))
b = wire(Interpolated([
		vec3(-0.6198, 0.5273, -0.4572),
		vec3(-0.07677, 0.1935, -0.2357),
		vec3(0.09098, 0.3421, 0.06265),
		vec3(0.2342, 0.3291, 0.07885),
		vec3(0.4923, -0.03592, 0.07334),
		vec3(0.8163, -0.09317, 0.3876)]))
c = wire(Softened([
		vec3(-0.4823, 0.5805, -0.7559),
		vec3(-0.2198, 0.4575, -0.6071),
		vec3(-0.1817, 0.6625, -0.26),
		vec3(0.1945, 0.4716, -0.06826),
		vec3(0.2814, 0.1778, -0.3889),
		vec3(0.5664, 0.06961, -0.1906),
		vec3(0.5155, 0.2737, 0.04415),
		vec3(0.8524, 0.1459, 0.2789)]))

start, stop = vec3(0), vec3(1,0,0)
mapped = [
	mapcurve(a, start, stop, scale=True, rotate=True, match='length'),
	mapcurve(a, start, stop, scale=True, rotate=False, match='length'),
	mapcurve(a, start, stop, scale=False, rotate=True, match='length'),
	mapcurve(a, start, stop, scale=False, rotate=False, match='length'),
	
	mapcurve(a, start, stop, scale=True, rotate=True, match='distance'),
	mapcurve(a, start, stop, scale=True, rotate=False, match='distance'),
	mapcurve(a, start, stop, scale=False, rotate=True, match='distance'),
	mapcurve(a, start, stop, scale=False, rotate=False, match='distance'),
	]
#show([a, mapped])

s1 = swipe([a,b,c], Interpolated2, interlace=0.4)

show([a, b, c, mapped, s1], options={'display_wire':True})
