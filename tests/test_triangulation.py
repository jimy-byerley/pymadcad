from madcad import *
from madcad.triangulation import *
from copy import deepcopy

from variants import variants

res = 13
lines = Web(
		[ vec3(2*cos(i/res*2*pi), 2*sin(i/res*2*pi), 0)	for i in range(res) ]
	+	[ vec3(  cos(i/res*2*pi),   sin(i/res*2*pi), 0)	for i in reversed(range(res)) ], 
	[ (i, (i+1)%res)  for i in range(res) ],
	)

# tests variants over those points
for variant in variants(connection=range(4)):
	w = deepcopy(lines)
	
	# with a hole piercing
	if variant['hole']:
		w += Web(w.points, [ (i +res, (i+1)%res +res)  for i in range(res) ])
		# with a hole filled
		if variant['hole_cluster']:
			w += Web(w.points, [ ((i+1)%res +res, i +res)  for i in range(res) ])
		
		# with connections between holes
		if variant['connection'] & 0b01:
			w += Web(w.points, [ (0,25), (25,0) ])		# y oriented
		# with double connection
		if variant['connection'] & 0b10:
			w += Web(w.points, [ (12,13), (13,12) ])	    # random

	print(variant)
	w.check()
	f = triangulation(w)
	f.mergeclose()
	try:
		f.check()
		assert f.issurface()
		show([f, [note_floating(p, text=' '+str(i)) for i,p in enumerate(f.points)]], display_points=True, display_wire=True)
	except:
		print(w.edges)
		show([w, f], display_points=True, display_wire=True)
		raise

# higher resolution test
print('case splines')
out = Interpolated([
		vec3(-0.06407, -1.339e-08, 0.1123),
		vec3(-0.1764, 1.885e-09, -0.01581),
		vec3(-0.1822, 1.974e-08, -0.1656),
		vec3(0.01997, 5.853e-09, -0.0491),
		vec3(0.04993, 1.954e-08, -0.1639),
		vec3(0.1539, 5.654e-09, -0.04743),
		vec3(0.2172, -1.518e-08, 0.1273),
		vec3(0.05076, -1.726e-08, 0.1448),
		vec3(-0.06407, -1.339e-08, 0.1123)])

inside = Softened([
		vec3(-0.1365, 3.373e-09, -0.02829),
		vec3(-0.1381, 1.3e-08, -0.109),
		vec3(0.03162, 1.587e-09, -0.01331),
		vec3(0.07406, 1.22e-08, -0.1024),
		vec3(0.1639, -4.96e-09, 0.04161),
		vec3(0.0466, -1.042e-08, 0.08737),
		vec3(-0.006657, -4.067e-09, 0.03412),
		vec3(-0.07323, -6.448e-09, 0.05409),
		vec3(-0.1365, 3.373e-09, -0.02829)])

lines = web(out).flip() + web(inside)
lines.mergeclose()
face = triangulation(lines)
face.check()
assert face.issurface()

# circles test
print('case circles')
lines = web([
			Circle((O,Z), 1),
			Circle((vec3(0.3,0.2,0),-Z), 0.2),
			Circle((vec3(-0.1,-0.4,0),-Z), 0.3),
			Circle((vec3(-0.5,0.3,0),-Z), 0.4),
			
			Circle((vec3(0.3,0.2,0),Z), 0.2),
			Circle((vec3(-0.1,-0.4,0),Z), 0.3),
			Circle((vec3(-0.5,0.3,0),Z), 0.4),
			]) .flip()
lines.mergeclose()
face = triangulation(lines)
face.check()
assert face.issurface()

# final display
show([face], options={'display_wire':True})
