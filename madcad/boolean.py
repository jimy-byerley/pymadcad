# This file is part of pymadcad,  distributed under license LGPL v3

'''	
	Defines boolean operations for triangular meshes. Strictly speaking, boolean operations applies on sets, but considering the volumes delimited by their mesh envelopes, we can perform boolean operations on those volumes by manipulating only surface meshes.
	
	This relies on a new intersection algorithm named syandana. It finds candidates for intersections using a spacial hashing of triangles over a voxel (see madcad.hashing). This is solving the problem of putting triangles in an octree.
	Also to avoid the increasing complexity of the operation with flat planes divided in multiple parallel triangles, the algorithm is implemented with a detection of ngons.
	
	The syandana algorithm achieves intersections of meshes in nearly `O(n)` where usual methods are `O(n**2)`
	
	After intersection, the selection of surface sides to keep or not is done through a propagation.
'''

from copy import copy,deepcopy
from time import time
from math import inf
from .mathutils import *
from . import core
from .mesh import Mesh, Web, edgekey, connef, line_simplification
from . import hashing
from . import triangulation

__all__ = ['intersect', 'pierce', 'boolean', 'intersection', 'union', 'difference']
		


def intersect(m1, m2) -> '(Web, Web)':
	''' cut the faces of m1 and m2 at their intersections '''
	if not prec:	prec = max(m1.precision(), m2.precision())
	m3 = copy(m1)
	return intersectwith(m1, m2, prec), intersectwith(m2, m3, prec)
	
def pierce(m1, m2, selector=False) -> Mesh:
	''' cut the faces of m1 at their intersection with faces of m2, and remove the faces inside
	
		selector decides which part of each mesh to keep
	
		 - False keep the exterior part (part exclusive to the other mesh)
		 - True keep the common part
	'''
	m3 = copy(m1)
	booleanwith(m3, m2, selector)
	return m3

def boolean(m1, m2, selector=(False,True), prec=None) -> Mesh:
	''' execute boolean operation on volumes 
	
		selector decides which part of each mesh to keep
	
		 - False keep the exterior part (part exclusive to the other mesh)
		 - True keep the common part
	'''
	if not prec:	prec = max(m1.precision(), m2.precision())
	
	if selector[0] is not None:
		if selector[1] is not None:
			mc1, mc2 = copy(m1), copy(m2)
			booleanwith(mc1, m2, selector[0], prec)
			booleanwith(mc2, m1, selector[1], prec)
			if selector[0] and not selector[1]:		mc1 = mc1.flip()
			if not selector[0] and selector[1]:		mc2 = mc2.flip()
			res = mc1 + mc2
			res.mergeclose()
			return res
		else:
			mc1 = copy(m1)
			booleanwith(mc1, m2, selector[0], prec)
			return mc1
	elif selector[1] is not None:
		return boolean(m2, m1, (selector[1], selector[0]))

def union(a, b) -> Mesh:			
	''' return a mesh for the union of the volumes. 
		It is a boolean with selector (False,False) 
	'''
	return boolean(a,b, (False,False))

def intersection(a, b) -> Mesh:	
	''' return a mesh for the common volume. 
		It is a boolean with selector (True, True) 
	'''
	return boolean(a,b, (True,True))

def difference(a, b) -> Mesh:	
	''' return a mesh for the volume of a less the common volume with b
		It is a boolean with selector (False, True)
	'''
	return boolean(a,b, (False,True))



def intersectwith(m1, m2, prec=None) -> Web:
	''' Cut m1 faces at their intersections with m2. 
		Returning the intersection edges in m1 and associated m2 faces.
		
		m1 faces and tracks are replaced thus the underlying buffers stays untouched.
	
		The algorithm is using ngon intersections and retriangulation, in order to avoid infinite loops and intermediate triangles.
	'''
	if not prec:	prec = m1.precision()
	frontier = Web(m1.points, groups=m2.faces)	# cut points for each face from m2
	
	# topology informations for optimization
	points = hashing.PointSet(prec, manage=m1.points)
	prox2 = hashing.PositionMap(hashing.meshcellsize(m2))
	for f2 in range(len(m2.faces)):
		prox2.add(m2.facepoints(f2), f2)
	conn = connef(m1.faces)
	
	mn = Mesh(m1.points, groups=m1.groups)	# resulting mesh
	grp = [-1]*len(m1.faces)	# flat region id
	currentgrp = -1
	for i in range(len(m1.faces)):
		# process the flat surface starting here, if the m1's triangle hits m2
		if grp[i] == -1 and m1.facepoints(i) in prox2:
			
			# a triangle cutted by an other will split it into 5 triangles, and each will be divided in 5 more if the cutting is stupidly incrmental
			# here we collect all the planar triangles and cut it as one n-gon to reduce the complexity
			# get the flat region - aka the n-gon to be processed
			currentgrp += 1
			surf = []
			front = [i]
			normal = m1.facenormal(i)
			if not isfinite(normal):	continue
			track = m1.tracks[i]
			while front:
				fi = front.pop()
				if grp[fi] == -1 and m1.tracks[fi] == track and distance2(m1.facenormal(fi), normal) <= NUMPREC:
					surf.append(fi)
					grp[fi] = currentgrp
					f = m1.faces[fi]
					for edge in ((f[1],f[0]), (f[2],f[1]), (f[0],f[2])):
						if edge in conn:	front.append(conn[edge])
			
			# get region ORIENTED outlines - aka the outline of the n-gon
			outline = set()
			for f1 in surf:
				f = m1.faces[f1]
				for edge in ((f[1],f[0]), (f[2],f[1]), (f[0],f[2])):
					if edge in outline:	outline.remove(edge)
					else:				outline.add((edge[1], edge[0]))
			original = set(outline)
			
			# process all ngon triangles
			# enrich outlines with intersections
			segts = {}
			for f1 in surf:
				f = m1.faces[f1]
				for f2 in set( prox2.get(m1.facepoints(f1)) ):
					intersect = core.intersect_triangles(m1.facepoints(f1), m2.facepoints(f2), 8*prec)
					if intersect:
						ia, ib = intersect[0][2], intersect[1][2]
						if distance2(ia, ib) <= prec**2:	continue
						# insert intersection points
						seg = (points.add(ia), points.add(ib))
						# associate the intersection edge with the m2's face
						if seg in segts:	continue
						segts[seg] = f2
						
						# cut the outline if needed
						for i in range(2):
							ii = intersect[i]
							if ii[0] != 0:	continue
							o = f[ii[1]], f[ii[1]-2]
							if o not in original:	continue
							# find the place where the outline is cutted
							p = m1.points[seg[i]]
							e = find(outline, lambda e: distance_pe(p, (m1.points[e[0]], m1.points[e[1]])) <= prec)
							if seg[i] not in e:	# NOTE maybe not necessary
								outline.remove(e)
								outline.add((e[0],seg[i]))
								outline.add((seg[i],e[1]))
			
			# simplify the intersection lines
			segts = Web(m1.points, list(segts.keys()), list(segts.values()), m2.faces)
			segts.mergepoints(line_simplification(segts, prec))
			frontier += segts
			
			# retriangulate the cutted surface
			segts.edges.extend([(b,a) for a,b in segts.edges])
			segts.edges.extend(outline)
			segts.tracks = [0] * len(segts.edges)
			flat = triangulation.triangulation_closest(segts, normal)
			# append the triangulated face, in association with the original track
			flat.tracks = [track] * len(flat.faces)
			flat.groups = m1.groups
			mn += flat
	# append non-intersected faces
	for f,t,grp in zip(m1.faces, m1.tracks, grp):
		if grp == -1:
			mn.faces.append(f)
			mn.tracks.append(t)
	
	m1.faces = mn.faces
	m1.tracks = mn.tracks
	return frontier


def booleanwith(m1, m2, side, prec=None) -> set:
	''' execute the boolean operation inplace and only on m1 '''
	if not prec:	prec = m1.precision()
	frontier = intersectwith(m1, m2, prec)
	
	conn1 = connef(m1.faces)
	used = [False] * len(m1.faces)
	notto = set(edgekey(*e) for e in frontier.edges)
	front = []
	
	# get front and mark frontier faces as used
	for e,f2 in zip(frontier.edges, frontier.tracks):
		for edge in (e, (e[1],e[0])):
			if edge in conn1:
				fi = conn1[edge]
				f = m1.faces[fi]
				# find from which side to propagate
				for i in range(3):
					if f[i] not in edge:
						proj = dot(m1.points[f[i]] - m1.points[f[i-1]], m2.facenormal(f2))  * (-1 if side else 1)
						if proj > prec:
							used[fi] = True
							front.append((f[i], f[i-1]))
							front.append((f[i-2], f[i]))
						break
	if not front:
		if side:	
			m1.faces = []
			m1.tracks = []
		return notto
	
	# display frontier
	#from . import text
	#for edge in notto:
		#p = (m1.points[edge[0]] + m1.points[edge[1]]) /2
		#scn3D.add(text.Text(p, str(edge), 9, (1, 1, 0)))
	#if debug_propagation:
		#from .mesh import Web
		#w = Web([1.01*p for p in m1.points], [e for e,f2 in frontier])
		#w.options['color'] = (1,0.9,0.2)
		#scn3D.add(w)
	
	# propagation
	front = [e for e in front if edgekey(*e) not in notto]
	c = 1
	while front:
		newfront = []
		for edge in front:
			if edge in conn1:
				fi = conn1[edge]
				if not used[fi]:
					used[fi] = c
					f = m1.faces[fi]
					for i in range(3):
						if edgekey(f[i-1],f[i]) not in notto:	
							newfront.append((f[i],f[i-1]))
		c += 1
		front = newfront
	
	# selection of faces
	#if debug_propagation:
		#from . import text
		#for i,u in enumerate(used):
			#if u:
				#p = m1.facepoints(i)
				#scn3D.add(text.Text((p[0]+p[1]+p[2])/3, str(u), 9, (1,0,1), align=('center', 'center')))
	
	# filter mesh content to keep
	m1.faces =  [f for u,f in zip(used, m1.faces) if u]
	m1.tracks = [t for u,t in zip(used, m1.tracks) if u]
	return notto

#debug_propagation = False


def cut_web(mesh: Web, ref: Web, prec=None) -> Wire:
	if not prec:  prec = mesh.precision()
	frontier = Wire(mesh.points, groups=ref.faces)
	
	# topology informations for optimization
	points = hashing.PointSet(prec, manage=mesh.points)
	prox = hashing.PositionMap(hashing.webcellsize(ref))
	for e in range(len(ref.edges)):
		prox.add(prox.edgepoints(e), e)
	conn = connpe(mesh.faces)
	
	mn = Web(mesh.points, groups=mesh.groups)  # resulting mesh
	for e1 in range(len(mesh.edges)):
		close = set( prox.get(mesh.edgepoints(e1)) )
		if close:
			# no need to get the straight zone around because on the case of an edge, it will not grow in complexity more than the number of cut segments
			# and an edge is easy to remesh after multiple intersections
			# compute intersections
			segts = {}
			e = mesh.edges[e1]
			for e2 in close:
				intersect = intersect_edges(mesh.edgepoints(e1), ref.edgepoints(e2), prec)
				if intersect:
					seg = points.add(intersect)
					segts.setdefault(seg, e2)
			
			# remesh the cutted edge
			segts = sorted(segts.items(), key=lambda s: dot(mesh.points[s[0]], direction))
			edges = web(Wire(mesh.points, map(itemgetter(0), segts), map(itemgetter(1), segts), ref.faces))
			# append the remeshed edge in association with the original track
			frontier += segts
			mn += Web(segts.points, segts.edges, typedlist.full(track, len(segts.edges), 'I'), mesh.groups)
		
		else:
			mn.edges.append(mesh.edges[e1])
			mn.tracks.append(mesh.tracks[e1])
	
	return mn, frontier

	
def pierce_web(mesh, ref, side, prec=None) -> set:
	if not prec:	prec = mesh.precision()
	frontier = cut_web(mesh, ref, prec)
	
	conn1 = connpe(mesh.edges)
	used = [False] * len(mesh.edges)
	notto = set(frontier.indices)
	front = []
	
	# get front and mark frontier points as used
	for p,e2 in zip(frontier.indices, frontier.tracks):
		for ei in conn1[p]:
			e = mesh.edges[ei]
			# find from which side to propagate
			if e[0] == p and side:
				front.append(e[1])
				used[edge] = True
			elif e[1] == p and not side:
				front.append(e[0])
				used[edge] = True
	
	if not front:
		if side:
			mesh.edges = []
			mesh.tracks = []
		return notto
		
	# propagation
	c = 1
	while front:
		newfront = []
		for p in front:
			for ei in conn1[p]:
				if not used[ei]:
					used[ei] = c
					e = mesh.edges[ei]
					for i in e:
						if i not in notto:
							front.append(i)
		c += 1
		front = newfront
		
	# filter mesh content to keep
	return Web(
				mesh.points,
				[e  for u,e in zip(used, mesh.edges) if u],
				[t  for u,t in zip(used, mesh.tracks) if u],
				mesh.groups,
				)

	
def intersect_edges(e1, e2, prec):
	''' intersection between 2 segments '''
	d1 = e1[1]-e1[0]
	d2 = e2[1]-e2[0]
	p = unproject(noproject(e2[0]-e1[0], d1), d2)
	#prec = NUMPREC * max(length2(d1), length2(d2))
	if dot(e1[0]-p, d1) > prec or dot(p-e1[1], d1) > prec:	return
	if dot(e2[0]-p, d2) > prec or dot(p-e2[1], d2) > prec:	return
	return p

	
	
def cut_web_mesh(mesh: Web, ref: Web, prec=None) -> Wire:
	
	if not prec:  prec = mesh.precision()
	frontier = Wire(mesh.points, groups=ref.faces)
	
	# topology informations for optimization
	points = hashing.PointSet(prec, manage=mesh.points)
	prox = hashing.PositionMap(hashing.webcellsize(ref))
	for f in range(len(ref.edges)):
		prox.add(prox.facepoints(f), f)
	conn = connpe(mesh.faces)
	
	mn = Web(mesh.points, groups=mesh.groups)  # resulting mesh
	for e1 in range(len(mesh.edges)):
		close = set(prox.get(mesh.facepoints(e1)))
		if close:
		
			# no need to get the straight zone around because on the case of an edge, it will not grow in complexity more than the number of cut segments
			# and an edge is easy to remesh after multiple intersections
			# compute intersections
			segts = {}
			e = mesh.edges[e1]
			for f2 in set( prox.get(mesh.edgepoints(e1)) ):
				intersect = intersect_edge_face(mesh.edgepoints(e1), ref.facepoints(f2), prec)
				if intersect:
					seg = points.add(intersect)
					segts.setdefault(seg, f2)
		
			# remesh the cutted edge
			segts = sorted(segts.items(), key=lambda s: dot(mesh.points[s[0]], direction))
			edges = web(Wire(mesh.points, map(itemgetter(0), segts), map(itemgetter(1), segts), ref.faces))
			# append the remeshed edge in association with the original track
			frontier += segts
			mn += Web(segts.points, segts.edges, typedlist.full(track, len(segts.edges), 'I'), mesh.groups)
		
		else:
			mn.edges.append(mesh.edges[e1])
			mn.tracks.append(mesh.tracks[e1])
			
	return mn, frontier
	

def intersect_edge_face(edge, face, prec):
	''' intersection between a segment and a triangle
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
