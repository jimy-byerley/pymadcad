# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module provides functions to compute convex hulls 
	(See https://en.wikipedia.org/wiki/Convex_hull) 
	and other hull-related operations for `Mesh` and `Web`
	
	Those can be very helpful to complete sketches or parts by adding the missing surface portions. Also very helpful to sort a set of directions.
'''

import scipy.spatial
from .mathutils import *
from .mesh import Mesh, Web, Wire, edgekey, facekeyo, arrangeface, connef, connpe, numpy_to_typedlist, typedlist_to_numpy
from .asso import Asso

from copy import copy
from collections import Counter



def simple_convexhull(points: '[vec3]') -> '[uvec3]':
	''' Just like `convexhull()` but minimalist.
		It does not take care of groups crossing. It doesn't return a new Mesh but a buffer of triangles indices 
	'''
	return numpy_to_typedlist(scipy.spatial.ConvexHull(typedlist_to_numpy(points, 'f8'), qhull_options='QJ Pp').simplices, uvec3)
	

def convexhull(source: '[vec3]') -> Mesh:
	''' compute the convex hull of the input container 
		
		Parameters:
			source (typedlist/Web/Mesh):	the input container, if it is a Web or Mesh, their groups are kept in the output data

		Example:
			
			>>> m = convexhull(mesh([
			... 	uvsphere(O, 1, alignment=X),
			... 	uvsphere(4*X, 2, alignment=X),
			... 	]))
	'''
	if isinstance(source, (Mesh,Web,Wire)):
		source = copy(source)
		source.strippoints()
		points = source.points
	else:
		points = typedlist(source, dtype=vec3)
		source = None
	
	return restore_groups(source, simple_convexhull(points)).orient()
	

def convexoutline(source: '[vec3]', normal: vec3=None, flatten: bool=False) -> Web:
	''' based on `convexhull()` but will extract the loop formed by the edges in the biggest planar projection of the convex hull 
	
		Parameters:
			source (typedlist/Web/Mesh):     the input container, if it is a Web or Mesh, their groups are kept in the output data
			normal:                          the projection normal to retreive the outline using `horizon()`, if None is given it will default to the direction in which the outlien surface is the biggest
			flatten:                         whether to project the outline points in its mean plane
	
		Example:
			
			>>> convexoutline(web([
			... 	Circle(Axis(O,Z), 1),
			... 	Circle(Axis(4*X,Z), 2),
			... 	]))
	'''
	hull = convexhull(source)
	direction = normal or widest_surface_direction(hull)
	outline = horizon(hull, direction)
	if flatten:
		outline.strippoints()
		center = outline.barycenter()
		outline.points = typedlist(center + noproject(p-center, direction)   for p in outline.points)
	return outline
	

def restore_groups(source, indices) -> Mesh:	
	'''
		Create a mesh contaning the given simplices, associated with groups created from group crossing from the source mesh.
		The simplices must refer to points in `source`.
		The created groups are tuples of indices of the groups touched by each simplex.
	
		Parameters:
			source (Mesh/Web):    the mesh containing reference triangles and groups
			indices:              a buffer of simplices indices (depending on the dimensionnality of `source`) we want to find groups for
	'''
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
	''' 
		Return a Web of the ORIENTED edges of the given mesh that lay between triangles that are oriented either sides of `direction`
	'''
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
	
	
"""
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
"""

