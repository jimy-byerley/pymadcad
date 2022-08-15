
import scipy.spatial

from .mathutils import *
from .mesh import Mesh, Web, Wire, edgekey, facekeyo, arrangeface, connef, connpe, numpy_to_typedlist, typedlist_to_numpy
from .asso import Asso

from copy import copy
from collections import Counter



def expand(surface: Mesh, offset: float) -> Mesh:
	''' generate a surface expanding the input mesh on the tangent of the ouline neighboring faces
	'''
	# outline with associated face normals
	pts = surface.points
	edges = {}
	for face in self.faces:
		for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
			if e in edges:	del edges[e]
			else:			edges[(e[1], e[0])] = self.facenormal(face)
	
	#def tangent(e0, e1):
		#''' find tangent between the given edges '''
		#c = cross(	edges[e0], 
					#edges[e1] )
		#o = cross(  pts[e0[0]] - pts[e1[1]],
					#edges[e0] + edges[e1] )
		#return normalize(mix(o, c, clamp(length2(c)/length2(o)/NUMPREC, 0, 1) ))
		
	def tangent(e0, e1):
		d = cross(edges[e0], edges[e1])
		v = pts[e1[0]] - pts[e0[1]]
		return mix(  
		          pts[e0[1]] + unproject(project(v, edges[e1]), cross(edges[e0], n))
				+ pts[e1[0]] + unproject(project(-v, edges[e0]), cross(edges[e1], n)),
				0.5)
	
	# cross neighbooring normals
	for loop in suites(edges, cut=False):
		assert loop[-1] == loop[0],  "non-manifold mesh"
		# compute the offsets, and remove anticipated overlapping geometries
		extended = [None]*len(loop)
		for i in range(len(loop)):
			ei0 = (loop[i-2], loop[i-1])
			ei1 = (loop[i-1], loop[i])
			ti = tangent(e0, e1) * offset
			ej0, ej1 = e0, e1
			for j in reversed(range(i-len(loop), i)):
				ej0, ej1 = (loop[i-1], loop[i]), ej0
				tj = tangent(ej0, ej1) * offset
				if dot(ti - tj, pts[ei1[0]] - pts[ej0[1]]) < 0:
					tj = tangent(ej0, ei1) * offset
				else:
					break
			for j in range(j, i):
				extended[j] = tj
		
		# insert the new points
		for i in range(len(extended)):
			if extended[i] != extended[i-1]:
				pts.append(extended[i])
				
		# create the faces
		for i in range(len(extended)):
			if extended[i] != extended[i-1]:
				mkquad(surface, loop[i-1], loop[i], j-1, j)
			else:
				mktri(surface, loop[i-1], loop[i], j)
	
	return surface
		
	
def simple_convexhull(points: '[vec3]') -> '[uvec3]':
	return numpy_to_typedlist(scipy.spatial.ConvexHull(typedlist_to_numpy(points, 'f8'), qhull_options='QJ Pp').simplices, uvec3)
	
def convexhull(source: typedlist) -> Mesh:
	''' compute the convex hull of the input container 
		
		Parameters:
			points (typedlist/Web/Mesh):	the input container, if it is a Web or Mesh, their groups are kept in the output data
	'''
	if isinstance(source, (Mesh,Web,Wire)):
		source = copy(source)
		source.strippoints()
		points = source.points
	else:
		points = typedlist(source, dtype=vec3)
		source = None
	
	return restore_groups(source, simple_convexhull(points)).orient()
	
def restore_groups(source, indices) -> Mesh:	
	groups = source.groups
	tracks = typedlist.full(len(groups), len(indices), dtype='I')
	
	if isinstance(source, Mesh):
		former = { facekeyo(*face): track   
						for face, track in zip(source.faces, source.tracks)}
	elif isinstance(source, Web):
		former = { edge: track
						for edge, track in zip(source.edges, source.tracks)}
	else:
		raise TypeError('expected a Mesh or Web')
	
	# create an associative dictionnary where associated tracks only appears one per point
	combiner = Asso() 
	for simplex, track in former.items():
		for p in simplex:
			if track not in combiner[p]:
				combiner.add(p, track)
	# search for original group or new group at each simplex
	groupindex = {}
	for i, simplex in enumerate(indices):
		belong = Counter()
		for p in simplex:
			belong.update(combiner[p])
		# get a shortlist of the groups reaching this simplex through its points
		couple = sorted(belong, key=lambda i: belong[i], reverse=True)
		shortlist = []
		amount = 0
		while amount < len(simplex):
			n = couple[len(shortlist)]
			shortlist.append(n)
			amount += belong[n]
		# one group only 
		if len(shortlist) == 1:
			tracks[i] = couple[0]
		else:
			combine = tuple(sorted(shortlist))
			if combine not in groupindex:
				groupindex[combine] = len(groups)
				groups.append(combine)
			tracks[i] = groupindex[combine]
	
	outdim = len(simplex)
	if outdim == 3:
		return Mesh(source.points, indices, tracks, groups)
	elif outdim == 2:
		return Web(source.points, indices, tracks, groups)
	else:
		raise TypeError('outer simplex dimension {} is not supported'.format(outdim))

def horizon(mesh, direction: vec3) -> Web:
	horizon = Web(points=mesh.points, groups=mesh.groups)
	signs = {}	# dictionnary for crossing face directions around edges
	groups = {} # track of the positive face connected for each edge
	for face, track in zip(mesh.faces, mesh.tracks):
		# pick the sign of the projected surface
		s = dot(mesh.facenormal(face), direction)
		if abs(s) <= NUMPREC:	s = 0
		elif s > 0:				s = 1
		else:					s = -1
		
		for i in range(3):
			edge = (face[i], face[i-2])
			key = edgekey(*edge)
			# pick the group of the positive face neighboring the edge
			if s > 0 or (s >= 0 and key not in groups):
				groups[key] = track
			# the edge belong to the horizon if the neigboring faces are each appart the projection plane
			if edge in signs and signs[edge] * s <= 0 and max(signs[edge], s) == 1:
				horizon.edges.append(edge if s > 0 else flipped(edge))
				horizon.tracks.append(groups[key])
				del signs[edge]
				del groups[key]
			# if matching edge not found yet, index the current face
			else:
				signs[flipped(edge)] = s
	return horizon

def flipped(simplex):
	if len(simplex) == 2:
		return (simplex[1], simplex[0])
	elif len(simplex) == 3:
		return (simplex[2], simplex[1], simplex[2])
	else:
		raise TypeError('expected and edge or a triangle')
	
def widest_surface_direction(mesh) -> vec3:
	# find a direction not orthogonal to any of the mesh faces
	normals = mesh.facenormals()
	directionmap = simple_convexhull(normals)
	score = 0
	proj = None
	for face in directionmap:
		surf = length2(cross(normals[face[1]]-normals[face[0]], normals[face[2]]-normals[face[0]]))
		if surf >= score:
			score, proj = surf, normals[face[0]] + normals[face[1]] + normals[face[2]]
	
	# project half of the mesh surface
	direction = vec3(0)
	for face in mesh.faces:
		surf = cross(mesh.points[face[1]]-mesh.points[face[0]], mesh.points[face[2]]-mesh.points[face[0]])
		if dot(surf, proj) > 0:
			direction += surf
	
	# select the average surface direction
	return normalize(direction)
	
	
def convexoutline(points: typedlist, normal=None, flatten=False) -> Web:
	''' based on `convexhull()` but will extract the loop formed by the edges in the biggest planar projection of the convex hull 
	'''
	hull = convexhull(points)
	direction = normal or widest_surface_direction(hull)
	outline = horizon(hull, direction)
	if flatten:
		outline.strippoints()
		center = outline.barycenter()
		outline.points = typedlist(center + noproject(p-center, direction)   for p in outline.points)
	return outline
	

def hull(bounds: Web) -> Web:
	conn = connpe(bounds)
	bridges = line_bridges(bounds, conn)
	# select a start point the most outward
	center = bounds.barycenter()
	start = min( (i  for e in bounds.edges for i in e), 
				key=lambda i: distance2(bounds.points[i], center))
	indev
	
def hull(bounds: Mesh) -> Mesh:
	indev
	
def minkowski(a: Mesh, b: Mesh) -> Mesh:
	''' minkowski sum of the input meshes '''
	indev
	
def brush(brush: Mesh, path, orient=None) -> Mesh:
	''' almost the same as a minkowski sum, but allows for orientation changes of the brush mesh, for each vertex of the path mesh 
	
		Parameters:
			brush (Mesh/Web):	the input mesh to move along `path` and being reoriented by `orient`
			path (Mesh/Web):	the input set to move the `brush` along, can be a Web or a Mesh, independently of the dimension of `brush`
			orient (typedlist):	
				list of vectors matching the point buffer of `path`, and giving the orientation that `brush` is taking in each of the `path` points
				If set to `None`, it defaults to the path normals
	'''
	indev
	
	
def test_convexhull():
	from .mesh import web
	from .generation import brick, icosphere, uvsphere
	from .primitives import Circle, Interpolated
	from .rendering import show
	
	results = []
	for i, obj in enumerate([
			brick(width=vec3(1)),
			icosphere(O, 1),
			brick(width=vec3(1)) + icosphere(vec3(1,2,3), 1),
			brick(width=vec3(1)) + icosphere(vec3(0,1,0), 1),
			brick(center=vec3(0,0,1), width=vec3(1)) + uvsphere(vec3(0,1,0), 1) + icosphere(vec3(0,-1,0), 0.8),
			web(Circle((O,Z), 1)),
			web(Circle((O,Z), 1), Circle((vec3(0,1,1), vec3(1,2,1)), 1), Circle((vec3(1.2,-1,1), X), 0.5)),
			web(Circle((O,Z), 0.2), Circle((vec3(0,2,0.5),Z), 1)),
			]):
		hull = convexhull(obj)
		hull.check()
		assert hull.isenvelope()
		results.append(hull.transform(i*2*X))
		
		contour = convexoutline(obj, normal=Z)
		contour.check()
		assert contour.isloop()
		results.append(contour.transform(i*2*X+4*Y))
		
		contour = convexoutline(obj, flatten=True)
		contour.check()
		assert contour.isloop()
		results.append(contour.transform(i*2*X+8*Y))

	show(results)
