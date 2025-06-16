from pnprint import nprint
from madcad import vec3, Mesh, Web, show
from madcad.hashing import *
from . import visualcheck

@visualcheck
def test_positionmap():
	triangles = Mesh([
						vec3(1,1,0), vec3(3,1,0), vec3(1,3,0),
						vec3(0,1,1), vec3(0,3,1), vec3(0,1,4),
						vec3(1,0,1), vec3(3,0,1), vec3(1,0,4),
						vec3(1,2,5), vec3(-3,0,4), vec3(4,-2,7),
						vec3( -0.2, -1, 0.452081 ), vec3( -0.796834, -0.403166, -0.550011 ), vec3( 0.114227, -1.31423, 0.979673 ),
						],
					[(0,1,2), (3,4,5), (6,7,8), (9,10,11), (12,13,14)],
					)
	lines = Web([
					vec3(5,0,0), vec3(5,0,4),
					vec3(0,5,0), vec3(0,6,4),
					#vec3(-6,-8,9), vec3(-2,-5,2), 
					vec3(-6,-7,2), vec3(-4,-5,9),
					],
				[(0,1),(2,3),(4,5)],
				)
	m = PositionMap(1, [
		(triangles.facepoints(0), 'x'),
		(triangles.facepoints(1), 'y'),
		(triangles.facepoints(2), 'z'),
		(triangles.facepoints(3), 'truc'),
		(triangles.facepoints(4), 'chose'),
		((vec3(5,0,0), vec3(5,0,4)), 'a'),
		((vec3(0,5,0), vec3(0,6,4)), 'b'),
		((vec3(-6,-7,2), vec3(-4,-5,9)), 'c'),
		])

	nprint(m.dict)
	print(set( m.get((vec3(8,0,0),vec3(0,8,0),vec3(-2,-2,8))) ))

	m.options['color'] = (0.7, 0.9, 1)

	return [m, triangles, lines]

def test_asso():
	m = Asso([
			(1,'truc'), 
			(2,'machin'),
			(12, 'chose'),
			(1, 'bidule'),
			])
	nprint('created', m)
	
	m.update([
		(1,'machin'),
		(-1,'gna'),
		])
	nprint('updated', m)
	
	m.add(1, 2)
	nprint('inserted', m)
	
	assert set(m[1]) == {'truc', 'bidule', 2, 'machin'}
	assert set(m[12]) == {'chose'}
