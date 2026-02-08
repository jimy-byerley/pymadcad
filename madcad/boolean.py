# This file is part of pymadcad,  distributed under license LGPL v3

'''	
	This modules provide boolean operations for all kind of meshes.
	
	Strictly speaking, boolean operations are operations on mathematically defined sets. In a n-dimensinal space it applies to subsets of point of the same dimension (volumes in 3D, surfaces in 2D).
	Here, volumes are defined by their exterior envelope (a Mesh) and surfaces by their contour (Web). So the boolean operation consist of intersecting the envelopes and contours and decide which cutted parts to keep.
	
	.. image:: /schemes/boolean-principle.svg

	This relies on a new intersection algorithm named syandana. It finds candidates for intersections using a spacial hashing of triangles over a voxel (see madcad.hashing). This is solving the problem of putting triangles in an octree.
	Also to avoid the increasing complexity of the operation with flat planes divided in multiple parallel triangles, the algorithm is implemented with a detection of ngons.
	
	The syandana algorithm achieves intersections of meshes in nearly `O(n)` where usual methods are `O(n**2)`
	
	After intersection, the selection of surface sides to keep or not is done through a propagation.

	
	Note:
	
		Web boolean operations theoretically means something only in a 2d space, and it takes place in a 3d space here, causing many situations where a web portion can be both inside and outside the contour. The algorithm here works even with meshes that are not planar at all. but it always assumes that the web you give it can be processed consistently as if it was.
	
	Note:
		
		Mesh boolean operation is relying on the surface outer normal to decide what is exterior and interior. It's always a local decision (ie. a decision made at the intersection), So the meshes must be well oriented.
		
		Web boolean operations on the other hand CANNOT be local due to the 3d space it's laying in. (a higher dimension than the surface it's theoreticall meant to outline). So it may sometimes not behave the way you expect it.
		To solve ambiguities of interior and exterior, the most outer part of first mesh argument is always considered to belong to the exterior. And the information is propagated through the web.
'''

from copy import copy, deepcopy
from operator import itemgetter
from functools import singledispatch
from time import time
from math import inf
from .mathutils import *
from .hashing import *
from . import core
from .mesh import Mesh, Web, Wire, web, line_simplification
from . import triangulation

__all__ = ['pierce', 'boolean', 'intersection', 'union', 'difference']



	
# ------- implementated operators -------



def cut_mesh(m1, m2, prec=None) -> '(Mesh, Web)':
	''' Cut m1 faces at their intersections with m2.

		Return:

			`(cutted, frontier)`

				:cutted(Mesh):	is `m1` with the faces cutted, but representing the same surface as before
				:frontier(Web): is a Web where edges are the intersection edges, and whose groups are the causing faces in `m2`

		Returning the intersection edges in m1 and associated m2 faces.
		The algorithm is using ngon intersections and retriangulation, in order to avoid infinite loops and intermediate triangles.
	'''
	if not prec:	prec = m1.precision()
	mn, frontier = core.cut_surface(m1, m2, prec)
	frontier.groups = m2.faces
	return mn, frontier


def pierce_mesh(m1, m2, side=False, prec=None, strict=False) -> Mesh:

	if not prec:	prec = m1.precision()
	return core.pierce_surface(m1, m2, side, prec, strict)

def boolean_mesh(m1, m2, sides=(False,True), prec=None) -> Mesh:

	if not prec:	prec = max(m1.precision(), m2.precision())
	
	mc1 = pierce_mesh(m1, m2, sides[0], prec)
	mc2 = pierce_mesh(m2, m1, sides[1], prec)
	if sides[0] and not sides[1]:		mc1 = mc1.flip()
	if not sides[0] and sides[1]:		mc2 = mc2.flip()
	res = mc1 + mc2
	res.mergeclose()
	return res



def cut_web(w1: Web, ref: Web, prec=None) -> '(Web, Wire)':
	''' Cut the web edges at their intersectsions with the `ref` web
		
		Return:
			
			`(cutted, frontier)`
			
				:cutted(Web):	is `w1` with the edges cutted, but representing the same lines as before
				:frontier(Wire): is a Web where edges are the intersection edges, and whose groups are the causing faces in `ref`
		'''
	if not prec:  prec = w1.precision()
	frontier = Wire(w1.points, [], [], groups=ref.edges)
	
	# topology informations for optimization
	points = PointSet(prec, manage=w1.points)
	prox = PositionMap(meshcellsize(ref))
	for e in range(len(ref.edges)):
		prox.add(ref.edgepoints(e), e)
	conn = connpe(w1.edges)
	
	mn = Web(w1.points, groups=w1.groups)  # resulting web
	for e1 in range(len(w1.edges)):
		processed = False  # flag enabled when the edge is reconstructed
		
		# collect edges in ref that can have intersection with e1
		close = set( prox.get(w1.edgepoints(e1)) )
		if close:
			# no need to get the straight zone around because on the case of an edge, it will not grow in complexity more than the number of cut segments
			# and an edge is easy to reweb after multiple intersections
			# compute intersections
			segts = {}
			for e2 in close:
				intersect = intersect_edges(w1.edgepoints(e1), ref.edgepoints(e2), prec)
				if intersect:
					seg = points.add(intersect)
					segts.setdefault(seg, e2)
			
			# reconstruct the geometries
			if segts:
				e = w1.edges[e1]
				# reweb the cutted edge
				direction = w1.points[e[1]] - w1.points[e[0]]
				sorted_segts = sorted(segts.items(), key=lambda s: dot(w1.points[s[0]], direction))
				# append the rewebed edge in association with the original track
				frontier += Wire(
						w1.points, 
						map(itemgetter(0), sorted_segts), 
						map(itemgetter(1), sorted_segts), 
						ref.edges,
						)
				
				suite = list(map(itemgetter(0), sorted_segts))
				if suite[0] != e[0]:	suite.insert(0, e[0])
				if suite[-1] != e[-1]:	suite.append(e[-1])
				mn += web(Wire(
						w1.points, 
						suite, 
						[w1.tracks[e1]] * len(suite), 
						w1.groups,
						))
				processed = True
		
		# keep the non intersected faces as is
		if not processed:
			mn.edges.append(w1.edges[e1])
			mn.tracks.append(w1.tracks[e1])
	
	mn.check()
	return mn, frontier

def pierce_web(web, ref, side=False, prec=None) -> Web:

	if not prec:	prec = web.precision()
	web, frontier = cut_web(web, ref, prec)
	conn = connpe(web.edges)		# connectivity
	stops = set(frontier.indices)	# propagation stop points
	
	used = [0] * len(web.edges)		# whether to keep the matching edges or not
	
	# sort points by distance to a "center"
	center = web.barycenter()
	order = sorted(range(len(web.points)), key=lambda i:  distance2(web.points[i], center))
	
	# always start an island from the most exterior
	for start in order:
		if start in stops:	continue
		ei = next(conn[start], None)
		if not ei or used[ei]:	continue
		
		# propagate
		front = [(start, not side)]		# points on the propagation front
		while front:
			last, keep = front.pop()
			for ei in conn[last]:
				if used[ei]:	continue
				used[ei] = 1 if keep else 2
				direction = int(web.edges[ei][0] == last)
				current = web.edges[ei][direction]
				front.append((current, keep ^ (current in stops)))
		
	# filter web content to keep
	return Web(
				web.points,
				[e  for u,e in zip(used, web.edges) if u == 1],
				[t  for u,t in zip(used, web.tracks) if u == 1],
				web.groups,
				)
				
def boolean_web(w1, w2, sides, prec=None) -> Web:

	if not prec:	prec = max(w1.precision(), w2.precision())
	
	mc1 = pierce_web(w1, w2, sides[0], prec)
	w2, frontier = cut_web(w2, mc1, prec)
	conn2 = connpe(w2.edges)		# connectivity
	stops = set(frontier.indices)	# propagation stop points
	
	used = [False] * len(w2.edges)  # whether to keep the matching edges or not
	
	for p2, ei in zip(frontier.indices, frontier.tracks):
		front = []	# points on the propagation front
		
		# check which side to keep to respect the choices made in the first web
		e1 = mc1.edges[ei]
		direction = sides[0] ^ sides[1] ^ (distance2(w2.points[p2], mc1.points[e1[0]]) < distance2(w2.points[p2], mc1.points[e1[1]]))
		for ei in conn2[p2]:
			if w2.edges[ei][direction] == p2:
				front.append(w2.edges[ei][direction-1])
				used[ei] = True
		
		# propagate
		while front:
			last = front.pop()
			if last in stops:	continue
			
			for ei in conn2[last]:
				if used[ei]:	continue
				used[ei] = True
				direction = int(w2.edges[ei][0] == last)
				next = w2.edges[ei][direction]
				front.append(next)
					
	# filter web content to keep
	mc2 = Web(
			w2.points,
			[e  for u,e in zip(used, w2.edges) if u],
			[t  for u,t in zip(used, w2.tracks) if u],
			w2.groups,
			)
	
	if sides[0] and not sides[1]:		mc1 = mc1.flip()
	if not sides[0] and sides[1]:		mc2 = mc2.flip()
	res = mc1 + mc2
	res.mergeclose()
	return res
		

	
def intersect_edges(e1: '(vec3,vec3)', e2: '(vec3,vec3)', prec) -> vec3:
	''' Intersection between 2 segments '''
	d1 = e1[1]-e1[0]
	d2 = e2[1]-e2[0]
	g = e1[0]-e2[0]
	p = e2[0] + unproject(noproject(g, d1), d2)
	if not isfinite(p):		return
	#prec = NUMPREC * max(length2(d1), length2(d2))
	if dot(e1[0]-p, d1) > prec or dot(p-e1[1], d1) > prec:	return
	if dot(e2[0]-p, d2) > prec or dot(p-e2[1], d2) > prec:	return
	return p

	
	
def cut_web_mesh(w1: Web, ref: Mesh, prec=None) -> '(Web, Wire)':
	''' Cut the web inplace at its intersections with the `ref` web.
	
		Return:
		
			`(cutted, frontier)`
	'''
	if not prec:  prec = w1.precision()
	frontier = Wire(w1.points, [], [], groups=ref.faces)
	
	# topology informations for optimization
	points = PointSet(prec, manage=w1.points)
	prox = PositionMap(meshcellsize(ref))
	for f in range(len(ref.faces)):
		prox.add(ref.facepoints(f), f)
	conn = connpe(w1.edges)
	
	mn = Web(w1.points, groups=w1.groups)  # resulting web
	for e1 in range(len(w1.edges)):
		processed = False	# flag enabled when the edge is reconstructed
		
		# collect edges in ref that can have intersection with e1
		close = set(prox.get(w1.edgepoints(e1)))
		if close:
		
			# no need to get the straight zone around because on the case of an edge, it will not grow in complexity more than the number of cut segments
			# and an edge is easy to remesh after multiple intersections
			# compute intersections
			segts = {}
			for f2 in close:
				intersect = intersect_edge_face(w1.edgepoints(e1), ref.facepoints(f2), prec)
				if intersect:
					seg = points.add(intersect)
					segts.setdefault(seg, f2)
		
			# reconstruct the geometries
			if segts:
				e = w1.edges[e1]
				# reweb the cutted edge
				direction = w1.points[e[1]] - w1.points[e[0]]
				sorted_segts = sorted(segts.items(), key=lambda s: dot(w1.points[s[0]], direction))
				# append the rewebed edge in association with the original track
				frontier += Wire(
						w1.points, 
						list(map(itemgetter(0), sorted_segts)), 
						list(map(itemgetter(1), sorted_segts)), 
						ref.faces,
						)
				mn += web(Wire(
						w1.points, 
						[e[0]] + list(map(itemgetter(0), sorted_segts)) + [e[1]], 
						[w1.tracks[e1]] * (len(sorted_segts)+2), 
						w1.groups,
						))
				processed = True
		
		# keep the non intersected faces as is
		if not processed:
			mn.edges.append(w1.edges[e1])
			mn.tracks.append(w1.tracks[e1])
	
	return mn, frontier
	
					
def pierce_web_mesh(w1: Web, ref: Mesh, side=False, prec=None) -> Web:

	if not prec:	prec = w1.precision()
	
	w1, frontier = cut_web_mesh(w1, ref, prec)
	conn = connpe(w1.edges)			# connectivity
	stops = set(frontier.indices)	# propagation stop points
	
	used = [False] * len(w1.edges)	# whether to keep the matching edges
	
	for p1, fi in zip(frontier.indices, frontier.tracks):
		front = []	# points on the propagation front
		
		# check which side to keep
		normal = ref.facenormal(fi)
		for ei in conn[p1]:
			e1 = w1.edges[ei]
			direction = int(e1[0] == p1)
			if side ^ (dot(w1.points[e1[direction]] - w1.points[p1], normal) > 0):
				front.append(e1[direction])
				used[ei] = True
		
		# propagate
		while front:
			last = front.pop()
			if last in stops:	continue
			
			for ei in conn[last]:
				if used[ei]:	continue
				used[ei] = True
				direction = int(w1.edges[ei][0] == last)
				next = w1.edges[ei][direction]
				front.append(next)
	
	# filter web content to keep
	return Web(
			w1.points,
			[e  for u,e in zip(used, w1.edges) if u],
			[t  for u,t in zip(used, w1.tracks) if u],
			w1.groups,
			)

def intersect_edge_face(edge: '(vec3, vec3)', face: '(vec3, vec3, vec3)', prec) -> vec3:
	''' Intersection between a segment and a triangle
		In case of intersection with an edge of the triangle or an extremity of the edge, return the formed point.
	'''
	n = cross(face[1]-face[0], face[2]-face[0])
	a, b = edge
	da = dot(face[0]-a, n)
	db = dot(face[0]-b, n)
	if abs(da) <= prec:		p = a
	elif abs(db) <= prec:	p = b
	elif da * db > 0:		return None	# the segment doesn't reach the plane
	else:
		edgedir = b-a
		proj = dot(edgedir, n)
		if abs(proj) <= prec:	return None	# the segment is parallel to the plane
		p = a + da * edgedir / proj
	
	# check that the point is inside the triangle
	for i in range(3):
		if dot(face[i] - p, normalize(cross(n,face[i]-face[i-1]))) > prec:
			return None  # the point is outside the triangle
	return p



# --------------- stable API -------------------

pierce_ops = {
	(Mesh,Mesh):	pierce_mesh,
	(Web,Web):		pierce_web,
	(Web,Mesh):		pierce_web_mesh,
	}
def pierce(m, ref, side=False, prec=None):
	''' Cut a web/mesh and remove its parts considered inside the `ref` shape
		
		Overloads:
		
		.. code::
		
			pierce(Mesh, Mesh) -> Mesh
			pierce(Web, Web) -> Web
			pierce(Web, Mesh) -> Web
	
		`side` decides which part of each mesh to keep
	
		 - False keeps the exterior part (part exclusive to the other mesh)
		 - True keeps the interior part (the common part)
	'''
	op = pierce_ops.get((type(m), type(ref)))
	if not op:
		raise TypeError('pierce is not possible between {} and {}'.format(type(m).__name__, type(ref).__name__))
	return op(m, ref, side, prec)


boolean_ops = {
	(Mesh,Mesh):	boolean_mesh,
	(Web,Web):		boolean_web,
	}
def boolean(a, b, sides=(False,True), prec=None):
	''' Cut two web/mesh and keep its interior or exterior parts
	
		Overloads:
		
		.. code::
		
			boolean(Mesh, Mesh) -> Mesh
			boolean(Web, Web) -> Web
	
		`sides` decides which part of each mesh to keep
	
		 - False keeps the exterior part (part exclusive to the other mesh)
		 - True keeps the common part
	'''
	op = boolean_ops.get((type(a), type(b)))
	if not op:
		raise TypeError('boolean is not possible between {} and {}'.format(type(a), type(b)))
	return op(a, b, sides, prec)

def union(a, b) -> Mesh:
	''' Return a mesh for the union of the volumes. 
		It is a boolean with selector `(False,False)`
	'''
	return boolean(a,b, (False,False))

def intersection(a, b) -> Mesh:	
	''' Return a mesh for the common volume. 
		It is a boolean with selector `(True, True)`
	'''
	return boolean(a,b, (True,True))

def difference(a, b) -> Mesh:	
	''' Return a mesh for the volume of `a` less the common volume with `b`
		It is a boolean with selector `(False, True)`
	'''
	return boolean(a,b, (False,True))
