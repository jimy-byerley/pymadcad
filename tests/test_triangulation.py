from madcad import *
from madcad.triangulation import *
from copy import deepcopy

from variants import variants


def cases():
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
		yield w
	
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
	yield lines

	# circles test
	print('case circles')
	lines = web([
				Circle((O,Z), 1),
				Circle((vec3(0.3,0.2,0),-Z), 0.2),
				Circle((vec3(-0.1,-0.4,0),-Z), 0.3),
				Circle((vec3(-0.5,0.3,0),-Z), 0.4),
				]).flip()
	yield lines
	
	print('case circles filled')
	lines += web([
				Circle((vec3(0.3,0.2,0),Z), 0.2),
				Circle((vec3(-0.1,-0.4,0),Z), 0.3),
				Circle((vec3(-0.5,0.3,0),Z), 0.4),
				]) .flip()
	lines.mergeclose()
	yield lines
	
	# flat front
	print('flat front')
	lines = wire([
		vec3(0, 1, 1e-5),
		vec3(1e-12, 0.8, -1e-5),
		vec3(-1e-14, 0.7, 2e-8),
		vec3(-3e-10, 0.5, 4e-9),
		vec3(4e-8, 0, 0),
		vec3(4e-8, -0.5, 0),
		vec3(5e-9, -0.8, 1e-12),
		vec3(2e-10, -1, 0),
		
		vec3(0.2, -1, 0),
		vec3(0.3, -1.000045, 0),
		vec3(0.4, -0.9999645, 0),
		vec3(0.5, -1, 1e-8),
		vec3(0.5, -1, -1e-12),
		vec3(3, -1, 1e-10),
		
		vec3(3, -0.8, 1e-13),
		vec3(3.0000056, -0.5, 2e-13),
		vec3(2.9999945, -0.2, 0),
		vec3(2.9999993, 0, 0),
		vec3(2.9999994, 0.1, 1e-12),
		vec3(3.00000004, 1, 1e-14),
		
		vec3(2.9, 1.00000002, 0), 
		vec3(1.7, 1.00000023, 0),
		vec3(1.5, 1, 0),
		vec3(1.4, 1, 0),
		vec3(1.2, 1, 0),
		vec3(1.1, 1, 0),
		vec3(0.15, 1, 0),
		vec3(0.05, 0.999999993, 1e-12),
		]).close()
	yield web(lines)
	
	print('flat front with hole')
	inside = wire([
		vec3(0.5, 0.5, 0),
		vec3(0.5, 0.2, 0),
		vec3(0.5, 0, 0),
		vec3(0.5000004, -0.1, 0),
		vec3(0.5000034, -0.2, 0),
		vec3(0.5, -0.5, 0),
		
		vec3(0.6, -0.50000045, 0),
		vec3(0.7, -0.50000455, 0),
		vec3(0.8, -0.49999993, 0),
		vec3(1.1, -0.500000014, 0),
		vec3(1.5, -0.5, 0),
		
		vec3(1.499999932, -0.4, 0),
		vec3(1.500000056, -0.2, 0),
		vec3(1.499999963, 0.1, 0),
		vec3(1.5, 0.5, 0),
		]).close().flip()
	yield web(lines) + web(inside)
		

results = []
offset = 0
for w in cases():
	w.check()
	try:
		f = triangulation(w)
		f.mergeclose()
	except:
		show([mat3(), w, [note_floating(p, text=' '+str(i)) for i,p in enumerate(w.points)]], display_points=True)
		raise
	try:
		f.check()
		assert f.issurface()
		# show([w, f], display_points=True, display_wire=True)
		# show([mat3(), w, f, [note_floating(p, text=' '+str(i)) for i,p in enumerate(f.points)]], display_points=True, display_wire=True)
	except:
		print(w.edges)
		show([mat3(), w, f, [note_floating(p, text=' '+str(i)) for i,p in enumerate(w.points)]], display_points=True, display_wire=True)
		raise
		
	box = f.box()
	offset -= box.min.x*1.2
	results.append(f.transform(offset*X))
	offset += box.max.x*1.2

show(results)
