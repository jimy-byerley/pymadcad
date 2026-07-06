from __future__ import annotations

from math import cos

from .mathutils import dot, cross, vec3, typedlist, NUMPREC
from .mesh import Mesh, Web, mktri, mkquad
from .hashing import connef, edgekey, flipedge


def minkowski(a: Mesh, b: Mesh, sharp=0.3, raw=True, bilateral=False) -> Mesh:
	a = a.own(points=True)
	b = b.own(points=True)
	a.strippoints()
	b.strippoints()
	prec = max(a.precision(), b.precision())
	cos_sharp = cos(sharp/2) + NUMPREC
	connb = connef(b.faces)
	conna = connef(a.faces)
	edgesa = a.edges()
	normalsb = b.facenormals()
	
	# points are offseted by summits in their normal if not sharp
	# points are replaced by portion of b mesh if sharp
	# edges are replaced by portion of horizon
	# faces are offseted by summit in their normal
	
	# decide smooth points
	smooth = [True] * len(a.points)
	normals = a.vertexnormals()
	for face in a.faces:
		normal = a.facenormal(face)
		for point in face:
			if dot(normal, normals[point]) <= cos_sharp:
				smooth[point] = False
	
	# find edge directions toward points
	adjacents = [[normal]  for normal in normals]
	for edge in edgesa:
		dir = a.points[edge[0]] - a.points[edge[1]]
		adjacents[edge[0]].append(+dir)
		adjacents[edge[1]].append(-dir)
		
	outlinersa = set()
	outlinersb = set()
	for edge in a.outlines_oriented():
		outlinersa.update(edge)
	for edge in b.outlines_oriented():
		outlinersb.update(edge)
	
	new = Mesh()
	index_points = {}
	index_groups = {}
	
	def insert_point(key):
		if key not in index_points:
			index_points[key] = len(new.points)
			point, offset = key
			new.points.append(a.points[point] + b.points[offset])
		return index_points[key]
			
	def insert_group(key):
		if key not in index_groups:
			index_groups[key] = len(new.groups)
			groupa, groupb = key
			if groupa is None and groupb is None:
				new.groups.append({})
			elif groupb is None:
				new.groups.append(a.groups[groupa])
			elif groupa is None:
				new.groups.append(b.groups[groupb])
			elif a.groups[groupa] is b.groups[groupb]:
				new.groups.append(a.groups[groupa])
			else:
				raise Exception('internal logic error')
		return index_groups[key]
	
	for point in range(len(a.points)):
		# no sum on outlines
		if point in outlinersa:
			continue
		# concave points do not have front faces
		if any(dot(dir, normals[point]) < 0   for dir in adjacents[point]):
			continue
		# if smooth[point]:
		# 	offset = _summit(b, normals[point])
		# 	insert_point((point, offset))
		# 	for face, _ in _front(b, adjacents[point]):
		# 		for merged in face:
		# 			index_points[(point, merged)] = index_points[(point, offset)]
		# else:
		for face, track in _front(b, adjacents[point]):
			mktri(new, (
				insert_point((point, face[0])),
				insert_point((point, face[1])),
				insert_point((point, face[2])),
				), insert_group((None, track)))
	
	for edge in edgesa:
		# no sum on outlines
		if not (edge in conna and flipedge(edge) in conna):
			continue
		# if smooth[edge[0]] and smooth[edge[1]]:
		# 	indev
		# else:
		a_direction = a.points[edge[1]] - a.points[edge[0]]
		a_side = (
			a.facenormal(conna[edge]),
			a.facenormal(conna[flipedge(edge)]),
			)
		for horizon in _horizon(b, normalsb, connb, a_direction, a_side):
			mkquad(new, (
				insert_point((edge[0], horizon[0])),
				insert_point((edge[0], horizon[1])),
				insert_point((edge[1], horizon[1])),
				insert_point((edge[1], horizon[0])),
				), insert_group((None, None)))
	
	for face, track in zip(a.faces, a.tracks):
		offset = _summit(b, a.facenormal(face))
		if offset in outlinersb:
			continue
		mktri(new, (
			insert_point((face[0], offset)),
			insert_point((face[1], offset)),
			insert_point((face[2], offset)),
			), insert_group((track, None)))
	
	if raw:
		return new
	return autounion(new)
			

def _summit(mesh: Mesh, direction: vec3) -> int:
	return max(
		(point  for face in mesh.faces for point in face),
		key = lambda point:  dot(mesh.points[point], direction)
		)

def _front(mesh: Mesh, limits: typedlist[vec3]) -> Iterator[uvec3]:
	if not limits:
		return
		
	prec = NUMPREC*8
	for face, track in zip(mesh.faces, mesh.tracks):
		normal = mesh.facenormal(face)
		if all(dot(normal, dir) > prec  for dir in limits):
			yield face, track

def _horizon(mesh: Mesh, normals: typedlist[vec3], conn: dict, a_direction: vec3, a_side: typedlist[vec3]) -> Iterator[uvec2]:
	prec = NUMPREC*8
	
	def ishorizon(face0, face1, a_direction):
		side0 = dot(face0, a_direction)
		side1 = dot(face1, a_direction)
		return side0 * side1 < prec**2 and max(abs(side0), abs(side1)) > prec
			
	for edge in conn:
		if edge != edgekey(*edge):
			continue
		reverse = flipedge(edge)
		if reverse not in conn:
			continue
	
		b_direction = mesh.points[edge[0]] - mesh.points[edge[1]]
		b_side = (
			normals[conn[edge]], 
			normals[conn[reverse]],
			)
		# two horizons are crossing
		if (ishorizon(b_side[0], b_side[1], a_direction)
		and ishorizon(a_side[0], a_side[1], b_direction)
		# same outer direction
		and dot(a_side[0] + a_side[1], b_side[0] + b_side[1]) > 0
			):
			if dot(cross(b_direction, a_direction), a_side[0] + a_side[1] + b_side[0] + b_side[1]) < 0:
				yield edge
			else:
				yield flipedge(edge)

				
# def minkowski(a: Mesh, b: Web, raw=True, bilateral=True) -> Mesh:
# 	indev
# 	
# def minkowski(a: Web, b: Web, raw=True) -> Web:
# 	indev
