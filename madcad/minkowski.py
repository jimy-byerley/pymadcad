from __future__ import annotations

from math import cos
from random import Random
from dataclasses import dataclass

from .mathutils import dot, cross, vec3, typedlist, NUMPREC
from .mesh import Mesh, Web, mktri, mkquad
from .hashing import connef, edgekey, flipedge


def minkowski(a: Mesh, b: Mesh, sharp=0.3, raw=True, bilateral=False) -> Mesh:
	a = _precompute(a)
	b = _precompute(b)
	
	# TODO curve flat areas to avoid coplanar cluster of faces
	
	# points are offseted by summits in their normal if not sharp
	# points are replaced by portion of b mesh if sharp
	# edges are replaced by portion of horizon
	# faces are offseted by summit in their normal
	
	new = Mesh()
	index_points = {}
	index_groups = {}
	biases = {}
	
	def insert_point(key):
		if key not in index_points:
			index_points[key] = len(new.points)
			point, offset = key
			new.points.append(a.original.points[point] + b.original.points[offset])
		return index_points[key]
			
	def insert_group(key):
		if key not in index_groups:
			index_groups[key] = len(new.groups)
			groupa, groupb = key
			if groupa is None and groupb is None:
				new.groups.append({})
			elif groupb is None:
				new.groups.append(a.original.groups[groupa])
			elif groupa is None:
				new.groups.append(b.original.groups[groupb])
			elif a.groups[groupa] is b.groups[groupb]:
				new.groups.append(a.original.groups[groupa])
			else:
				raise Exception('internal logic error')
		return index_groups[key]
	
	# prec = NUMPREC*100
	prec = 0
	
	for a_edge in a.adjacency:
		# no sum on outlines
		reverse = flipedge(a_edge)
		if a_edge[0] < a_edge[1] or not (a_edge in a.adjacency and reverse in a.adjacency):
			continue
		
		a_direction = a.randomized.points[a_edge[1]] - a.randomized.points[a_edge[0]]
		a_side = (
			a.randomized_normals[a.adjacency[a_edge]],
			a.randomized_normals[a.adjacency[reverse]],
			)
		
		# concave edges do not generate surface
		if dot(cross(a_side[0], a_side[1]), a_direction) < 0:
			continue
		
		for b_edge in b.adjacency:
			# no sum on outlines
			reverse = flipedge(b_edge)
			if b_edge[0] < b_edge[1] or not (b_edge in b.adjacency and reverse in b.adjacency):
				continue
		
			b_direction = b.randomized.points[b_edge[1]] - b.randomized.points[b_edge[0]]
			b_side = (
				b.randomized_normals[b.adjacency[b_edge]],
				b.randomized_normals[b.adjacency[reverse]],
				)
			
			# concave edges do not generate surface
			if dot(cross(b_side[0], b_side[1]), b_direction) < 0:
				continue
			
			if dot(a_side[0] + a_side[1], b_side[0] + b_side[1]) <= 0:
				continue
			b_horizon = dot(a_side[0], b_direction) * dot(a_side[1], b_direction)
			a_horizon = dot(b_side[0], a_direction) * dot(b_side[1], a_direction)
			
			# two horizons are crossing
			if b_horizon < -prec and a_horizon < -prec:
				if dot(cross(b_direction, a_direction), a_side[0] + a_side[1] + b_side[0] + b_side[1]) > 0:
					b_edge = b_edge
				else:
					b_edge = flipedge(b_edge)
		
				mkquad(new, (
					insert_point((a_edge[0], b_edge[0])),
					insert_point((a_edge[0], b_edge[1])),
					insert_point((a_edge[1], b_edge[1])),
					insert_point((a_edge[1], b_edge[0])),
					), insert_group((None, None)))
	
	from .hashing import connpp
	
	# merges = {}
	# renormalize = dict()
	
	b_adjacency = connpp(b.original.faces)
	a_undetermined = []
	for face, (a_face, a_track, a_normal_original, a_normal_randomized) in enumerate(zip(a.original.faces, a.original.tracks, a.original_normals, a.randomized_normals)):
		for b_point in range(len(b.original.points)):
			# concave points do not generate surface
			if dot(b.original_vertexnormals[b_point], a_normal_randomized) < 0:
				continue
			# no sum on outlines
			if b_point in b.outliners:
				continue
				
			if all(dot(b.randomized.points[adjacent] - b.randomized.points[b_point], a_normal_randomized) < -prec for adjacent in b_adjacency[b_point]):
				mktri(new, (
					insert_point((a_face[0], b_point)),
					insert_point((a_face[1], b_point)),
					insert_point((a_face[2], b_point)),
					), insert_group((a_track, None)))
			
				# ref = index_points[(a_face[0], b_point)]
				
				# if b.smooth[b_point]:
				# 	for a_adjacent in a_face:
				# 		if a.smooth[a_adjacent]:
				# 			adjacent = (a_adjacent, b_point)
				# 			if adjacent in index_points:
				# 				target = merges.get(ref, ref)
				# 				merges[index_points[adjacent]] = target
				# 				new.points[target] += new.points[index_points[adjacent]] / renormalize.get(index_points[adjacent], 1)
				# 				renormalize[target] = renormalize.get(target, 1) + 1
	
	a_adjacency = connpp(a.original.faces)
	b_undetermined = []
	for face, (b_face, b_track, b_normal_original, b_normal_randomized) in enumerate(zip(b.original.faces, b.original.tracks, b.original_normals, b.randomized_normals)):
		for a_point in range(len(a.original.points)):
			# concave points do not generate surface
			if dot(a.original_vertexnormals[a_point], b_normal_randomized) < 0:
				continue
			# no sum on outlines
			if a_point in a.outliners:
				continue
			
			if all(dot(a.randomized.points[adjacent] - a.randomized.points[a_point], b_normal_randomized) < -prec for adjacent in a_adjacency[a_point]):
				mktri(new, (
					insert_point((a_point, b_face[0])),
					insert_point((a_point, b_face[1])),
					insert_point((a_point, b_face[2])),
					), insert_group((None, b_track)))

	# for index, amount in renormalize.items():
	# 	new.points[index] /= amount
	# new.mergepoints(merges)
	
	if raw:
		return new
	return autounion(new)
			
@dataclass
class _Mesh:
	original: Mesh
	randomized: Mesh
	adjacency: dict[uvec2, int]
	outliners: set[int]
	original_vertexnormals: typedlist[vec3]
	original_normals: typedlist[vec3]
	randomized_normals: typedlist[vec3]
	outliners: set[int]
	adjacents: list[list[vec3]]
	smooth: list[bool]

def _precompute(mesh: Mesh) -> _Mesh:
	mesh = mesh.own(points=True)
	mesh.strippoints()
	adjacency = connef(mesh.faces)
	outliners = set()
	for edge in mesh.outlines_oriented():
		outliners.update(edge)
	# randomized = _randomize(_convexify(mesh, adjacency, 1e-5), 1e-8)
	from .mathutils import quat
	epsilon = 1e-6
	randomized = _convexify(mesh, adjacency, epsilon).transform(quat(vec3(1, 2, 3)*epsilon))
	original_normals = mesh.facenormals()
	randomized_normals = randomized.facenormals()
	
	original_vertexnormals = mesh.vertexnormals()
	adjacents = [[normal]  for normal in original_vertexnormals]
	for edge in mesh.edges():
		dir = mesh.points[edge[0]] - mesh.points[edge[1]]
		adjacents[edge[0]].append(+dir)
		adjacents[edge[1]].append(-dir)
	
	# decide smooth points
	sharp = 0.3
	cos_sharp = cos(sharp/2) + NUMPREC
	smooth = [True] * len(mesh.points)
	for face, normal in zip(mesh.faces, original_normals):
		for point in face:
			if dot(normal, original_vertexnormals[point]) < cos_sharp:
				smooth[point] = False
	
	return _Mesh(
		original = mesh,
		randomized = randomized,
		adjacency = adjacency,
		outliners = outliners,
		original_vertexnormals = original_vertexnormals,
		original_normals = original_normals,
		randomized_normals = randomized_normals,
		adjacents = adjacents,
		smooth = smooth,
		)

def _summit(mesh: Mesh, direction: vec3, biased: vec3) -> int:
	prec = NUMPREC*8
	return max(
		(point  for face in mesh.faces for point in face),
		key = lambda point:  _fallback_dot((direction, biased), mesh.points[point], prec)
		)

def _front(mesh: Mesh, normals: typedlist[vec3], limits: typedlist[vec3], rnormals: typedlist[vec3]) -> Iterator[uvec3]:
	if not limits:
		return
		
	prec = NUMPREC*8
	for face, track, normal, biased in zip(mesh.faces, mesh.tracks, normals, rnormals):
		if all(_fallback_dot((normal, biased), dir, prec) > prec  for dir in limits):
			yield face, track

def _horizon(mesh: Mesh, normals: typedlist[vec3], conn: dict, a_direction: vec3, a_side: typedlist[vec3], biased_normals: typedlist[vec3]) -> Iterator[uvec2]:
	prec = NUMPREC*8
	
	def ishorizon(face0, face1, direction):
		side0 = _fallback_dot(face0, direction, prec)
		side1 = _fallback_dot(face1, direction, prec)
		# return side0 * side1 < prec**2 and max(abs(side0), abs(side1)) > prec
		return side0 * side1 < -prec**2
			
	for edge in conn:
		if edge != edgekey(*edge):
			continue
		reverse = flipedge(edge)
		if reverse not in conn:
			continue
	
		b_direction = mesh.points[edge[0]] - mesh.points[edge[1]]
		b_side = (
			(normals[conn[edge]], biased_normals[conn[edge]]), 
			(normals[conn[reverse]], biased_normals[conn[reverse]]),
			)
		# two horizons are crossing
		if (ishorizon(b_side[0], b_side[1], a_direction)
		and ishorizon(a_side[0], a_side[1], b_direction)
		# same outer direction
		and dot(a_side[0][0] + a_side[1][0], b_side[0][0] + b_side[1][0]) > 0
		):
			if dot(cross(b_direction, a_direction), a_side[0][0] + a_side[1][0] + b_side[0][0] + b_side[1][0]) < 0:
				yield edge
			else:
				yield flipedge(edge)

def _fallback_dot(a: tuple[vec3, vec3], b: vec3, prec):
	return dot(a[0], b)
	
	prod = dot(a[0], b)
	if prod <= prec:
		prod = dot(a[1], b)
	return prod
	
def _convexify(mesh: Mesh, adjacency: dict[uvec2, int]=None, amplitude: float=1e-5) -> Mesh:
	if adjacency is None:
		adjacency = connef(mesh.faces)
	new = typedlist.full(vec3(0), len(mesh.points))
	for edge in adjacency:
		if edge[0] < edge[1]:
			continue
		reverse = flipedge(edge)
		if not (edge in adjacency and reverse in adjacency):
			continue
		normals = (
			mesh.facenormal(adjacency[edge]), 
			mesh.facenormal(adjacency[reverse]),
			)
		if dot(normals[0], normals[1]) >= 1-NUMPREC:
			for point in edge:
				new[point] -= sum(normals)
	
	for i, offset in enumerate(new):
		new[i] = mesh.points[i] - offset*amplitude
	return Mesh(points=new, faces=mesh.faces, tracks=mesh.tracks, groups=mesh.groups)

def _randomize(mesh: Mesh, seed: int=42, amplitude: float=1e-6) -> Mesh:
	rng = Random(seed)
	new = typedlist(vec3)
	new.reserve(len(mesh.points))
	for point in mesh.points:
		new.append(point + vec3(rng.random(), rng.random(), rng.random())*amplitude)
	return Mesh(points=new, faces=mesh.faces, tracks=mesh.tracks, groups=mesh.groups)

# def minkowski(a: Mesh, b: Web, raw=True, bilateral=True) -> Mesh:
# 	indev
# 	
# def minkowski(a: Web, b: Web, raw=True) -> Web:
# 	indev
