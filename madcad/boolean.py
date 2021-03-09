# This file is part of pymadcad,  distributed under license LGPL v3

'''	
	Defines boolean operations for triangular meshes
	
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

__all__ = ['intersect', 'boolean', 'intersection', 'union', 'difference']
		


def intersect(m1: Mesh, m2: Mesh):
	''' cut the faces of m1 and m2 at their intersections '''
	m3 = copy(m1)
	intersectwith(m1, m2)
	intersectwith(m2, m3)

def boolean(m1: Mesh, m2: Mesh, selector: '(bool,bool)', prec=None) -> Mesh:
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

def union(a: Mesh, b: Mesh) -> Mesh:			
	''' return a mesh for the union of the volumes. 
		It is a boolean with selector (False,False) 
	'''
	return boolean(a,b, (False,False))

def intersection(a: Mesh, b: Mesh) -> Mesh:	
	''' return a mesh for the common volume. 
		It is a boolean with selector (True, True) 
	'''
	return boolean(a,b, (True,True))

def difference(a: Mesh, b: Mesh) -> Mesh:	
	''' return a mesh for the volume of a less the common volume with b
		It is a boolean with selector (False, True)
	'''
	return boolean(a,b, (False,True))



def intersectwith(m1, m2, prec=None):
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
			segts.edges.extend(segts.edges)
			segts.edges.extend(outline)
			segts.tracks = [0] * len(segts.edges)
			flat = triangulation.triangulation_sweepline(segts, normal, prec)
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


def booleanwith(m1, m2, side, prec=None):
	''' execute the boolean operation only on m1 '''
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
	
	m1.faces =  [f for u,f in zip(used, m1.faces) if u]
	m1.tracks = [t for u,t in zip(used, m1.tracks) if u]
	return notto

#debug_propagation = False


