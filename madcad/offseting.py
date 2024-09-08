# This file is part of pymadcad,  distributed under license LGPL v3
'''
	The functions here allow to move or extrude meshes along their normals
	
    .. note::
        Any offseting provided here only rely on the vertex and edge normals of the input meshes, so don't expect a nice result if your mesh is too chaotic to have meaningful normals
'''

from .mathutils import *
from .mesh import Mesh, Web, Wire, MeshError, web, wire, mkquad, mktri
from .hashing import edgekey, facekeyo, suites

__all__ = ['thicken', 'inflate', 'inflate_offsets', 'expand']


def inflate_offsets(surface: Mesh, offset: float, method='face') -> '[vec3]':
	''' Displacements vectors for points of a surface we want to inflate.
		
		Parameters:
			offset:
				the distance from the surface to the offset surface. Its meaning depends on `method`
			method:     
				determines if the distance is from the old to the new faces, edges or points
				possible values: `'face', 'edge', 'point'`
	'''
	pnormals = surface.vertexnormals()
	
	# smooth normal offsets laterally when they are closer than `offset`
	outlines = surface.outlines_oriented()
	l = len(pnormals)
	normals = deepcopy(pnormals)
	for i in range(5):	# the number of steps is the diffusion distance through the mesh
		for a,b in outlines:
			d = surface.points[a] - surface.points[b]		# edge direction
			t = cross(pnormals[a]+pnormals[b], d)	# surface tangent normal to the edge
			# contribution stars when the offset points are closer than `offset`
			contrib = 1 - smoothstep(0, offset, length(offset*(pnormals[a]-pnormals[b])+d))
			normals[a] += contrib * 0.5*project(pnormals[b]-pnormals[a], t)
			normals[b] += contrib * 0.5*project(pnormals[a]-pnormals[b], t)
		# renormalize
		for i in range(l):
			pnormals[i] = normals[i] = normalize(normals[i])
	
	# compute offset length depending on the method
	if method == 'face':
		lengths = [inf]*len(pnormals)
		for face in surface.faces:
			fnormal = surface.facenormal(face)
			for p in face:
				lengths[p] = min(lengths[p], 1/dot(pnormals[p], fnormal))
		return typedlist((pnormals[p]*lengths[p]*offset   for p in range(len(pnormals))), dtype=vec3)
	
	elif method == 'edge':
		lengths = [inf]*len(pnormals)
		for edge,enormal in surface.edgenormals().items():
			for p in edge:
				lengths[p] = min(lengths[p], 1/dot(pnormals[p], enormal))
		return typedlist((pnormals[p]*lengths[p]*offset	for p in range(len(pnormals))), dtype=vec3)
		
	elif method == 'point':
		return typedlist((pnormals[p]*offset	for p in range(len(pnormals))), dtype=vec3)

def inflate(surface:Mesh, offset:float, method='face') -> 'Mesh':
	''' Move all points of the surface to make a new one at a certain distance of the last one

		Parameters:
			offset:       the distance from the surface to the offseted surface. its meaning depends on `method`
			method:       determines if the distance is from the old to the new faces, edges or points
        
        Example:
            
            >>> sphere = pierce(
            ...         icosphere(O, 1), 
            ...         brick(min=vec3(0), max=vec3(2)) .flip(),
            ...         )
            >>> inflate(sphere, 0.1)
	'''
	return Mesh(
				typedlist((p+d   for p,d in zip(surface.points, inflate_offsets(surface, offset, method))), dtype=vec3),
				surface.faces,
				surface.tracks,
				surface.groups)

def thicken(surface: Mesh, thickness: float, alignment:float=0, method='face') -> 'Mesh':
	''' Thicken a surface by extruding it, points displacements are made along normal. 

		Parameters:
			thickness:    determines the distance between the two surfaces (can be negative to go the opposite direction to the normal).
			alignment:    specifies which side is the given surface: 0 is for the first, 1 for the second side, 0.5 thicken all apart the given surface.
			method:       determines if the thickness is from the old to the new faces, edges or points
			
        Example:
            
            >>> sphere = pierce(
            ...         icosphere(O, 1), 
            ...         brick(min=vec3(0), max=vec3(2)) .flip(),
            ...         )
            >>> thicken(sphere, 0.1)
	'''
	displts = inflate_offsets(surface, thickness, method)
	
	a = alignment
	b = alignment-1
	m = (	Mesh(
				typedlist((p+d*a  for p,d in zip(surface.points,displts)), dtype=vec3), 
				surface.faces[:], 
				surface.tracks[:], 
				surface.groups)
		+	Mesh(
				typedlist((p+d*b  for p,d in zip(surface.points,displts)), dtype=vec3), 
				surface.faces, 
				surface.tracks, 
				surface.groups[:]
				) .flip() 
		)
	t = len(m.groups)
	l = len(surface.points)
	m.groups.append(None)
	for e in surface.outlines_oriented():
		mkquad(m, (e[0], e[1], e[1]+l, e[0]+l), t)
	return m


def expand(surface: Mesh, offset: float, collapse=True) -> Mesh:
	''' Generate a surface expanding the input mesh on the tangent of the ouline neighboring faces
	
		Parameters:
			offset:		distance from the outline point to the expanded outline points
			collapse:	if True, expanded points leading to crossing edges will collapse into one
			
        Example:
        
            >>> expand(
            ...     revolution(wire([vec3(1,1,0), vec3(0,1,0)]), Axis(O,X), pi),
            ...     0.5)
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
		d1 = pts[e1[1]] - pts[e1[0]]
		n0, n1 = edges[e0], edges[e1]
		t = normalize(cross(n0, n1) + NUMPREC*(d0*length(d1)-d1*length(d0)) + NUMPREC**2 * (cross(n0, d0) + cross(n1, d1)))
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


def axis_midpoint(a0: Axis, a1: Axis, x=0.5) -> vec3:
	''' Return the midpoint of two axis. 
		`x` is the blending factor between `a0` and `a1`
		
		- `x = 0` gives the point of `a0` the closest to `a1`
		- `x = 1` gives the point of `a1` the closest to `a0`
	'''
	p0, d0 = a0
	p1, d1 = a1
	if dot(d0,d1)**2 == length2(d0)*length2(d1):
		return mix(p0, p1, 0.5)
	return mix(
		p0 + unproject(project(p1-p0, noproject(d0, d1)), d0),
		p1 + unproject(project(p0-p1, noproject(d1, d0)), d1),
		x)

