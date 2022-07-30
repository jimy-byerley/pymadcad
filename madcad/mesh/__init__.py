from .mesh import Mesh, mkquad, mktri
from .web import Web
from .wire import Wire
from .conversions import *
from .container import (
	MeshError, NMesh,
	connpe, connef, connpp,
	facekeyo, edgekey, arrangeface, arrangeedge,
	suites, striplist,
	typedlist_to_numpy, numpy_to_typedlist,
	)

Mesh2 = Web
Mesh3 = Mesh

__all__ = [
		'Mesh', 'Web', 'Wire', 'MeshError', 'web', 'wire', 
		'connpp', 'connpp', 'connpe', 'connef',
		'edgekey', 'facekeyo', 'arrangeface', 'arrangeedge', 
		'suites', 'line_simplification', 'mesh_distance', 'striplist',
		'typedlist_to_numpy', 'numpy_to_typedlist',
		]

		
from ..mathutils import *


def line_simplification(web, prec=None):
	''' return a dictionnary of merges to simplify edges when there is points aligned.
	
		This function sort the points to remove on the height of the triangle with adjacent points.
		The returned dictionnary is guaranteed without cycles
	'''
	if not prec:	prec = web.precision()
	pts = web.points
	
	# simplify points when it forms triangles with too small height
	merges = {}
	def process(a,b,c):
		# merge with current merge point if height is not sufficient, use it as new merge point
		height = length(noproject(pts[c]-pts[b], pts[c]-pts[a]))
		if height > prec:
			#scn3D.add(text.Text(pts[b], str(height), 8, (1,0,1), align=('left', 'center')))
			return b
		else:
			merges[b] = a
			return a
	
	for k,line in enumerate(suites(web.edges, oriented=False)):
		s = line[0]
		for i in range(2, len(line)):
			s = process(s, line[i-1], line[i])
		if line[0]==line[-1]: process(s, line[0], line[1])
		
	# remove redundancies in merges (there can't be loops in merges)
	for k,v in merges.items():
		while v in merges and merges[v] != v:
			merges[k] = v = merges[v]
	return merges


def distance2_pm(point, mesh) -> '(d, prim)':
	''' squared distance from a point to a mesh
	'''
	score = inf
	best = None
	if isinstance(mesh, Mesh):
		for face in mesh.faces:
			f = mesh.facepoints(face)
			n = cross(f[1]-f[0], f[2]-f[0])
			if not n:	continue
			# check if closer to the triangle's edges than to the triangle plane
			plane = True
			for i in range(3):
				d = f[i-1]-f[i-2]
				if dot(cross(n, d), point-f[i-2]) < 0:
					x = dot(point-f[i-2], d) / length2(d)
					# check if closer to the edge points than to the edge axis
					if x < 0:	dist, candidate = distance2(point, f[i-2]), face[i-2]
					elif x > 1:	dist, candidate = distance2(point, f[i-1]), face[i-1]
					else:		dist, candidate = length2(noproject(point - f[i-2], d)), (face[i-2], face[i-1])
					plane = False
					break
			if plane:
				dist, candidate = dot(point-f[0], n) **2 / length2(n), face
			if dist < score:
				best, score = candidate, dist
	elif isinstance(mesh, (Web,Wire)):
		if isinstance(mesh, Web):	edges = mesh.edges
		else:						edges = mesh.edges()
		for edge in edges:
			e = mesh.edgepoints(edge)
			d = e[1]-e[0]
			x = dot(point - e[0], d) / length2(d)
			# check if closer to the edge points than to the edge axis
			if x < 0:	dist, candidate = distance2(point, e[0]), edge[0]
			elif x > 1:	dist, candidate = distance2(point, e[1]), edge[1]
			else:		dist, candidate = length2(noproject(point - e[0], d)), edge
			if dist < score:
				best, score = candidate, dist
	elif isinstance(mesh, vec3):
		return distance2(point, mesh), 0
	else:
		raise TypeError('cannot evaluate distance from vec3 to {}'.format(type(mesh)))
	return score, best

def mesh_distance(m0, m1) -> '(d, prim0, prim1)':
	''' minimal distance between elements of meshes 
	
		The result is a tuple `(distance, primitive from m0, primitive from m1)`.
		`primitive` can be:
		
			:int:				index of the closest point
			:(int,int):			indices of the closest edge
			:(int,int,int): 	indices of the closest triangle
	'''
	# compute distance from each points of m to o
	def analyse(m, o):
		# get an iterator over actually used points only
		if isinstance(m, Mesh):
			usage = [False]*len(m.points)
			for f in m.faces:
				for p in f:	usage[p] = True
			it = (i for i,u in enumerate(usage) if u)
		elif isinstance(m, Web):
			usage = [False]*len(m.points)
			for e in m.edges:
				for p in e:	usage[p] = True
			it = (i for i,u in enumerate(usage) if u)
		elif isinstance(m, Wire):
			it = m.indices
		elif isinstance(m, vec3):
			return (*distance2_pm(m, o), 0)
		# comfront to the mesh
		return min((
				(*distance2_pm(m.points[i], o), i)
				for i in it), 
				key=lambda t:t[0])
	# symetrical evaluation
	d0 = analyse(m0, m1)
	d1 = analyse(m1, m0)
	if d0[0] < d1[0]:	return (sqrt(d0[0]), d0[2], d0[1])
	else:				return (sqrt(d1[0]), d1[1], d1[2])
	
