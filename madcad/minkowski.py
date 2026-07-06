from __future__ import annotations
from .mathutils import dot, cross, vec3, typedlist, NUMPREC
from .mesh import Mesh, Web, mktri, mkquad
from .hashing import connef, edgekey, flipedge


def minkowski(a: Mesh, b: Mesh, sharp=0.1, raw=True, bilateral=False) -> Mesh:
	prec = max(a.precision(), b.precision())
	# cos_sharp = cos(sharp/2) + NUMPREC
	connb = connef(b.faces)
	conna = connef(a.faces)
	normalsb = b.facenormals()
	
	# points are offseted by summits in their normal if not sharp
	# points are replaced by portion of b mesh if sharp
	# edges are replaced by portion of horizon
	# faces are offseted by summit in their normal
	
	# # decide smooth points
	# smooth = [True] * len(a.points)
	# normals = a.vertexnormals()
	# for face in a.faces:
	# 	normal = a.facenormal(face)
	# 	for point in face:
	# 		if dot(normal, normals[point]) <= cos_sharp:
	# 			smooth[point] = False
	
	# find edge directions toward points
	vertexnormals = a.vertexnormals()
	adjacents = [[]  for _ in range(len(a.points))]
	for edge in conna:
		if edge != edgekey(*edge):
			continue
		dir = a.points[edge[0]] - a.points[edge[1]]
		adjacents[edge[0]].append(+dir)
		adjacents[edge[1]].append(-dir)
	
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
			new.groups.append({})
		return index_groups[key]
	
	for point in range(len(a.points)):
		# if smooth[point]:
		# 	offset = _summit(b, normals[point])
		# 	insert_point((point, offset), a.points[point] + b.points[offset])
		# else:
		for face, track in _front(b, adjacents[point] + [vertexnormals[point]]):
			mktri(new, (
				insert_point((point, face[0])),
				insert_point((point, face[1])),
				insert_point((point, face[2])),
				), insert_group((None, track)))
	
	for edge in conna:
		if edge != edgekey(*edge):
			continue
		if flipedge(edge) not in conna:
			continue
		# if smooth[edge[0]] and smooth[edge[1]]:
		# 	indev
		# else:
		direction = a.points[edge[1]] - a.points[edge[0]]
		side0 = a.facenormal(conna[edge])
		side1 = a.facenormal(conna[flipedge(edge)])
		for horizon in _horizon(b, normalsb, connb, direction, [
				cross(side0, -direction),
				cross(side1, direction),
				side0 + side1,
				]):
			if dot(side0 + side1, cross(direction, b.points[horizon[1]] - b.points[horizon[0]])) > 0:
				horizon = flipedge(horizon)
			mkquad(new, (
				insert_point((edge[0], horizon[0])),
				insert_point((edge[0], horizon[1])),
				insert_point((edge[1], horizon[1])),
				insert_point((edge[1], horizon[0])),
				), insert_group(None))
	
	for face, track in zip(a.faces, a.tracks):
		offset = _summit(b, a.facenormal(face))
		mktri(new, (
			insert_point((face[0], offset)),
			insert_point((face[1], offset)),
			insert_point((face[2], offset)),
			), insert_group((track, None)))
	
	return new
			

def _summit(mesh: Mesh, direction: vec3) -> int:
	return max(
		(point  for face in mesh.faces for point in face),
		key = lambda point:  dot(mesh.points[point], direction)
		)

def _front(mesh: Mesh, limits: typedlist[vec3]) -> Iterator[uvec3]:
	prec = NUMPREC*8
	for face, track in zip(mesh.faces, mesh.tracks):
		normal = mesh.facenormal(face)
		if all(dot(normal, dir) > prec  for dir in limits):
			yield face, track
				
def _horizon(mesh: Mesh, normals: typedlist[vec3], conn: dict, direction: vec3, limits: typedlist[vec3]) -> Iterator[uvec2]:
	prec = NUMPREC*8
	for edge in conn:
		if edge != edgekey(*edge):
			continue
		reverse = flipedge(edge)
		if reverse not in conn:
			continue
		sides = (
			dot(normals[conn[edge]], direction),
			dot(normals[conn[reverse]], direction),
			)
		if sides[0]*sides[1] < prec**2 and max(sides) > prec:
		# if sides[0]*sides[1] < prec**2:
			normal = normals[conn[edge]] + normals[conn[reverse]]
			if all(dot(normal, limit) > prec  for limit in limits):
				yield edge

# def minkowski(a: Mesh, b: Web, raw=True, bilateral=True) -> Mesh:
# 	indev
# 	
# def minkowski(a: Web, b: Web, raw=True) -> Web:
# 	indev
