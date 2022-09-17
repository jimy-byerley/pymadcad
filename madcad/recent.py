
import scipy.spatial

from .mathutils import *
from .mesh import Mesh, Web, Wire, edgekey, facekeyo, arrangeface, connef, connpe, suites, mkquad, mktri, numpy_to_typedlist, typedlist_to_numpy
from .asso import Asso

from copy import copy, deepcopy
from collections import Counter


def axis_midpoint(a0, a1):
	p0, d0 = a0
	p1, d1 = a1
	if dot(d0,d1)**2 == length2(d0)*length2(d1):
		return mix(p0, p1, 0.5)
	return mix(
		p0 + unproject(project(p1-p0, noproject(d0, d1)), d0),
		p1 + unproject(project(p0-p1, noproject(d1, d0)), d1),
		0.5)
	
#def axis_midpoint(a0, a1):
	#p0, d0 = a0
	#p1, d1 = a1
	#diff = p2 - p1
	#d1d1 = dot(d1, d1)
	#d1d2 = dot(d1, d2)
	#d2d2 = dot(d2, d2)
	#k = NUMPREC / (NUMPREC + d1d1 * d2d2 - d1d2**2)
	#x1, x2 = inverse(mat2(
				#d1d1 + k,  -d1d2,   
				#-d1d2,     d2d2 + k,
				#)) * vec2(dot(diff, d1), -dot(diff, d2))
	#return mix(p1 + x1*d1, p2 + x2*d2, 0.5)

def expand_offsets(surface: Mesh, offset: float, collapse=True) -> Mesh:
	''' generate a surface expanding the input mesh on the tangent of the ouline neighboring faces
	'''
	# outline with associated face normals
	pts = surface.points
	edges = {}
	for face in surface.faces:
		for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
			if e in edges:	del edges[e]
			else:			edges[(e[1], e[0])] = surface.facenormal(face)
	
	# return the point on tangent for a couple of edges from the frontier
	def tangent(e0, e1):
		mid = axis_midpoint(
				(pts[e0[1]], pts[e0[0]] - pts[e0[1]]), 
				(pts[e1[0]], pts[e1[1]] - pts[e1[0]]),
				)
		d0 = pts[e0[1]] - pts[e0[0]]
		n0, n1 = edges[e0], edges[e1]
		t = normalize(cross(n0, n1) + NUMPREC * cross(n0, d0))
		if dot(t, cross(n0, d0)) < 0:
			t = -t
		return mid + t * offset
	
	# cross neighbooring normals
	for loop in suites(edges, cut=False):
		assert loop[-1] == loop[0],  "non-manifold input mesh"
		loop.pop()
		# compute the offsets, and remove anticipated overlapping geometries
		extended = [None]*len(loop)
		for i in range(len(loop)):
			# consecutive edges around i-1
			ei0 = (loop[i-2], loop[i-1])
			ei1 = (loop[i-1], loop[i])
			ti = tangent(ei0, ei1)
			if collapse:
				tk = deepcopy(ti)
				weight = 1
				# j is moving to find how much points to gather
				ej0 = ei0
				for j in reversed(range(i-len(loop)+1, i-1)):
					# consecutive edges aroung j
					ej0, ej1 = (loop[j-1], loop[j]), ej0
					tj = tangent(ej0, ej1)
					
					if dot(ti - tj, pts[ei1[0]] - pts[ej0[1]]) <= NUMPREC * length2(pts[ei1[0]] - pts[ej0[1]]):
						tk += tj
						weight += 1
					else:
						break
				# store tangents
				for k in range(j+1, i):
					extended[k] = tk/weight
			else:
				extended[i-1] = ti
		
		# insert the new points
		j = l = len(pts)
		g = len(surface.groups)
		surface.groups.append(None)
		for i in range(len(extended)):
			if extended[i] != extended[i-1]:
				pts.append(extended[i])
				
		# create the faces
		for i in range(len(extended)):
			if extended[i] != extended[i-1]:
				mkquad(surface, (loop[i-1], loop[i], j, (j if j > l else len(pts)) -1), g)
				j += 1
			else:
				mktri(surface, (loop[i-1], loop[i], (j if j > l else len(pts)) -1), g)
	
	return surface
		
	
	

