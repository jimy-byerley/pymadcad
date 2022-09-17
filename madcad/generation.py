# This file is part of pymadcad,  distributed under license LGPL v3
'''
	The functions here generare meshes from simpler objects (like lower dimension meshes, or simple primitives). You won't find here iterative methods, nor adaptative geometry operations, nor mesh modification operations.
	The functions here may eventually be complex but will always feel predictable and simple from a human perspective.
'''

from .mathutils import *
from .primitives import Axis
from .mesh import Mesh, Web, Wire, edgekey, facekeyo, MeshError, web, wire, suites, mkquad, mktri
from . import triangulation
from . import settings
from . import primitives
from .nprint import nprint

from copy import copy


__all__ = [
	'extrans', 'extrusion', 'revolution', 'saddle', 'tube', 
	'repeat', 'thicken', 'inflate', 'inflate_offsets', 'expand',
	'flatsurface', 'icosurface', 'subdivide',
	'square', 'brick', 'cylinder', 'cone', 'pyramid', 'icosahedron', 'icosphere', 'uvsphere', 'regon', 
	]

	
# --- extrusion things ---


def extrusion(trans, line: Web, alignment:float=0) -> Mesh:
	''' create a surface by extruding the given outline by a transformation
		
		Parameters:
			line:         a line (Web or Wire) or a surface (Mesh) to extrude
			trans:        any transformation object accepted by `mathutils.transform`
			alignment:	   when > 0 the line is extruded both sides (the transformation is linearly interpoled)
	'''
	trans = transform(trans)
	neutral = mat4()
	return extrans(line, [
				neutral + (trans-neutral)*(-alignment),
				neutral + (trans-neutral)*(1-alignment),
				],
				[(0,1,0)])

def revolution(angle: float, axis: Axis, profile: Web, resolution=None) -> Mesh:
	''' create a revolution surface by extruding the given outline
		`steps` is the number of steps between the start and the end of the extrusion
		
		Parameters:
			angle:    angle of rotation between the given profile and the final produced profile
			axis:     the axis to rotate around
			profile:  the shape to extrude
			resolution:   resolution setting for the biggest produced circle, such as for primitives
	'''
	if not isinstance(profile, (Mesh,Web,Wire)):	
		profile = web(profile)
	# get the maximum radius, to compute the curve resolution
	radius = 0
	for pt in profile.points:
		v = pt-axis[0]
		v -= project(v,axis[1])
		radius = max(radius, length(v))
	steps = settings.curve_resolution(abs(angle*radius), abs(angle), resolution)+2
	# use extrans
	def trans():
		for i in range(steps):
			yield rotatearound(i/(steps-1)*angle, axis)
	def links():
		for i in range(steps-2):          yield (i,i+1, 0)
		if abs(angle-2*pi) <= NUMPREC:    yield (steps-2, 0, 0)
		else:                             yield (steps-2, steps-1, 0)
	return extrans(profile, trans(), links())

def saddle(web1: Web, web2: Web) -> Mesh:
	''' create a surface by extruding outine1 translating each instance to the next point of outline2
	'''
	web1, web2 = web(web1), web(web2)
	def trans():
		s = web2.points[0]
		for p in web2.points:
			yield translate(mat4(1), p-s)
	return extrans(web1, trans(), ((*e,t)  for e,t in zip(web2.edges, web2.tracks)))

def tube(outline: Web, path: Wire, end=True, section=True) -> Mesh:
	''' create a tube surface by extrusing the outline along the path
		if `section` is True, there is a correction of the segments to keep the section undeformed by the curve
	'''
	path = wire(path)
	def trans():
		lastrot = quat()
		l = len(path)-1
		yield mat4(1)
		for i in range(1,l+1):
			
			if i < l:
				# calculer l'angle a ce coude
				o = path[i]
				v1 = normalize(path[i-1]-o)
				v2 = normalize(path[i+1]-o)
				c = cross(-v1,v2)
				o = dot(-v1,v2)
				cl = length(c)
				cn = c/cl
				ha = atan2(cl,o)/2
				hrot = angleAxis(ha, cn)
				# calcul de la rotation de la section
				rot = hrot * lastrot
				lastrot = hrot * rot
				m = mat3_cast(rot)
				# deformation de la section pour ne pas réduire le volume
				if section:	
					m = scaledir(normalize(v1+v2), 1/cos(ha)) * m
			else:
				m = mat3_cast(lastrot)
			# construction de la transformation
			yield transform(path[i]) * mat4(m) * transform(-path[0])
	
	trans = trans()
	if not end:		next(trans)
	# handle wires with no tracks
	if path.tracks:		links = ((i, i+1, path.tracks[i]) for i in range(len(path)-1))
	else:				links = ((i, i+1, 0) for i in range(len(path)-1))
	# generate
	return extrans(outline, trans, links)


def extrans(section, transformations, links) -> Mesh:
	''' create a surface by extruding and transforming the given outline.
		
		Parameters:
			section:           a `Web` or a `Mesh`
			transformations:   iterable of mat4, one each section
			link:              iterable of tuples (a,b,t)  with:
									`(a,b)` the sections to link (indices of values returned by `transformation`).
									`t` the group number of that link, to combine with the section groups		
	'''
	# prepare
	if isinstance(section, Mesh):
		face = copy(section)
		face.strippoints()
		section = face.outlines()
	else:
		face = None
		section = copy(web(section))
	reindex = section.strippoints()
	
	mesh = Mesh()	  # result mesh
	extremities = {}  # index of extremities in links graph (similar to Web extremities), associated to a boolean telling the direction of that extremity
	kept = {}         # transformations kept in memory, indexed by their sections
	groups = {}       # groups created by the extransion
	
	l = len(section.points)
	
	# generate all sections faces using links
	for a,b,u in links:
		al = a*l
		bl = b*l
		for (c,d),v in zip(section.edges, section.tracks):
			mesh.faces.append((al+c, al+d, bl+d))
			mesh.faces.append((al+c, bl+d, bl+c))
			t = groups.setdefault((u,v), len(groups))
			mesh.tracks.append(t)
			mesh.tracks.append(t)
		# find extremities
		if face:
			if a in extremities:   del extremities[a]
			else:	               extremities[a] = True
			if b in extremities:   del extremities[b]
			else:                  extremities[b] = False
	
	# generate all combined groups
	mesh.groups = [None] * len(groups)
	for (u,v), t in groups.items():
		mesh.groups[t] = section.groups[v]	# NOTE will change in the future to mention both u and v
	
	# generate all sections points using transformations
	for k,trans in enumerate(transformations):
		for p in section.points:
			mesh.points.append(vec3(trans*vec4(p,1)))
		# keep extremities transformations
		if face and k in extremities:
			kept[k] = trans
	
	# append faces at extremities
	if face:
		merges = {}  # point merge dictionnary at faces insertion
		
		for k, orient in extremities.items():
			for src,dst in enumerate(reindex):
				if dst >= 0:
					merges[src + len(mesh.points)] = dst + l*k
			end = face .transform(kept[k])
			mesh += end if extremities[k] else end.flip()
		mesh.mergepoints(merges)
	
	mesh.check()
	return mesh
	
def linstep(start, stop, x):
	if x <= start:	return 0
	if x >= stop:	return 1
	return (x-start)/(stop-start)

def inflate_offsets(surface: Mesh, offset: float, method='face') -> '[vec3]':
	''' displacements vectors for points of a surfaceace we want to inflate.
		
		Parameters:
			offset:
				the distance from the surface to the offseted surface. its meaning depends on `method`
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
			# contribution stars when the offseted points are closer than `offset`
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
	''' move all points of the surface to make a new one at a certain distance of the last one

		Parameters:
			offset:       the distance from the surface to the offseted surface. its meaning depends on `method`
			method:       determines if the distance is from the old to the new faces, edges or points
	'''
	return Mesh(
				typedlist((p+d   for p,d in zip(surface.points, inflate_offsets(surface, offset, method))), dtype=vec3),
				surface.faces,
				surface.tracks,
				surface.groups)

def thicken(surface: Mesh, thickness: float, alignment:float=0, method='face') -> 'Mesh':
	''' thicken a surface by extruding it, points displacements are made along normal. 

		Parameters:
			thickness:    determines the distance between the two surfaces (can be negative to go the opposite direction to the normal).
			alignment:    specifies which side is the given surface: 0 is for the first, 1 for the second side, 0.5 thicken all apart the given surface.
			method:       determines if the thickness is from the old to the new faces, edges or points
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
	''' generate a surface expanding the input mesh on the tangent of the ouline neighboring faces
	
		Parameters:
			offset:		distance from the outline point to the expanded outline points
			collapse:	if True, expanded points leading to crossing edges will collapse into one
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
	''' return the midpoint of two axis. 
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


# --- filling things ---

def flatsurface(outline, normal=None) -> 'Mesh':
	''' generates a surface for a flat outline using the prefered triangulation method .
	
		if `normal` is specified, it must be the normal vector to the plane, and will be used to orient the face.
	'''
	if isinstance(outline, Wire):
		m = triangulation.triangulation_outline(outline, normal)
	else:
		m = triangulation.triangulation(web(outline), normal)
	if normal and dot(m.facenormal(0), normal) < 0:
		m = m.flip()
	return m


def icosurface(pts, ptangents, resolution=None) -> 'Mesh':
	''' generate a surface ICO (a subdivided triangle) with its points interpolated using interpol2tri.
	
		- If normals are given instead of point tangents (for ptangents), the surface will fit a sphere.
		- Else ptangents must be a list of couples (2 edge tangents each point).
	'''
	# compute normals to points
	if isinstance(ptangents[0], tuple):
		normals = [None]*3
		for i in range(3):
			normals[i] = normalize(cross(ptangents[i][0], ptangents[i][1]))
	else:
		# if normals are given instead of tangents, compute tangents to fit a sphere surface
		normals = ptangents
		ptangents = [None]*3
		for i in range(3):
			ptangents[i] = (
				normalize(noproject(pts[i-2]-pts[i], normals[i])) * arclength(pts[i], pts[i-2], normals[i], normals[i-2]),
				normalize(noproject(pts[i-1]-pts[i], normals[i])) * arclength(pts[i], pts[i-1], normals[i], normals[i-1]),
				)
	
	# evaluate resolution (using a bad approximation of the length for now)
	div = max(( settings.curve_resolution(
					distance(pts[i-1], pts[i-2]), 
					anglebt(normals[i-1], normals[i-2]), 
					resolution)
				for i in range(3) ))
	
	return dividedtriangle(lambda u,v: intri_sphere(pts, ptangents, u,v), div)
	
def dividedtriangle(placement, div=1) -> 'Mesh':
	''' generate a subdivided triangle with points placed according to the placement closure
		`placement(a,b) -> vec3`
		with a,b such as a+b+c = 1 with a,b,c within [0;1]
	'''
	segts = div+2
	# place points
	mesh = Mesh(groups=[None])
	for i in range(segts):
		u = i/(segts-1)				
		for j in range(segts-i):
			v = j/(segts-1)
			p = placement(u,v)
			mesh.points.append(p)
	# create faces
	c = 0
	for i in reversed(range(1,segts+1)):
		for j in range(i-1):
			s = c+j
			mesh.faces.append((s, s+i, s+1))
		for j in range(1,i-1):
			s = c+j
			mesh.faces.append((s, s+i-1, s+i))
		c += i
	mesh.tracks = typedlist.full(0, len(mesh.faces), 'I')

	return mesh


def subdivide(mesh, div=1) -> 'Mesh':
	''' subdivide all faces by the number of cuts '''
	n = div+2
	pts = typedlist(dtype=vec3)
	faces = typedlist(dtype=uvec3)
	tracks = typedlist(dtype='I')
	c = 0
	for f,t in enumerate(mesh.tracks):
		# place the points
		o,p0,p1 = mesh.facepoints(f)
		x = p0-o
		y = p1-o
		for i in range(n):
			u = i/(n-1)	
			for j in range(n-i):
				v = j/(n-1)
				p = o + u*x + v*y
				pts.append(p)
		# create the faces
		for i in reversed(range(1,n+1)):
			for j in range(i-1):
				s = c+j
				faces.append(uvec3(s, s+i, s+1))
			for j in range(1,i-1):
				s = c+j
				faces.append(uvec3(s, s+i-1, s+i))
			c += i
		tracks.extend([t] * (len(faces)-len(tracks)))
	
	new = Mesh(pts, faces, tracks)
	new.mergeclose()
	return new


# --- standard shapes ---
	
def brick(*args, **kwargs) -> 'Mesh':
	''' a simple brick with rectangular sides 
	
		constructors
		
			- brick(Box)
			- brick(min, max)
			- brick(center=vec3(0), width=vec3(-inf))
	'''
	if len(args) == 1 and not kwargs and isinstance(args[0], Box):		
		box = args[0]
	else:							
		box = Box(*args, **kwargs)
	mesh = Mesh(
		[
			vec3(1, 0, 0),
			vec3(1, 0, 1),
			vec3(0, 0, 1),
			vec3(0, 0, 0),
			vec3(1, 1, 0),
			vec3(1, 1, 1),
			vec3(0, 1, 1),
			vec3(0, 1, 0)],
		[
			uvec3(0, 1, 2),
			uvec3(0, 2, 3),
			uvec3(4, 7, 6),
			uvec3(4, 6, 5),
			uvec3(0, 4, 5),
			uvec3(0, 5, 1),
			uvec3(1, 5, 6),
			uvec3(1, 6, 2),
			uvec3(2, 6, 7),
			uvec3(2, 7, 3),
			uvec3(4, 0, 3),
			uvec3(4, 3, 7)],
		[	0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5	],
		[None] * 6,
		)
	for i in range(len(mesh.points)):
		mesh.points[i] = mesh.points[i]*box.width + box.min
	return mesh
	
def cylinder(bottom:vec3, top:vec3, radius:float, fill=True) -> 'Mesh':
	''' create a revolution cylinder, with the given radius 
	
		Parameters:
		
			bottom, top (vec3): the cylinder extremities centers
			fill (bool):        whether to put faces at both extremities
	'''
	direction = top-bottom
	base = wire(primitives.Circle((bottom,normalize(direction)), radius))
	if fill:	base = flatsurface(base).flip()
	return extrusion(direction, base)

def cone(summit:vec3, base:vec3, radius:float, fill=True) -> 'Mesh':
	''' create a revolution cone, with a base of the given radius 
	
		Parameters:
			
			summit (vec3):  The point at the top of the cone
			base (vec3):    the center point of the base
			fill (bool):    whether to put a face at the base
	'''
	base = wire(primitives.Circle((base, normalize(summit-base)), radius))
	if fill:	base = flatsurface(base)
	return pyramid(summit, base)
		
def pyramid(summit:vec3, base) -> 'Mesh':
	''' create a pyramid with the given summit point and the given base 
	
		Parameters:
			summit (vec3):   the top (summit) of the cone, not necessarity in the center of the shape
			base: (Mesh,Web,Wire):  the base shape
	'''
	if isinstance(base, Mesh):
		outline = base.outlines().flip()
		outline.stripgroups()
		result = Mesh(base.points, groups=outline.groups)
	else:
		outline = web(base)
		result = Mesh(points=outline.points, groups=outline.groups)
	
	p = len(result.points)
	result.points.append(summit)
	for edge, track in zip(outline.edges, outline.tracks):
		result.faces.append(uvec3(edge, p))
		result.tracks.append(track)
	
	if isinstance(base, Mesh):
		result += base.flip()

	return result

def square(axis:primitives.Axis, width:float) -> 'Mesh':
	''' return a simple square with the given normal axis and square width.
		Useful to quickly create a cutplane
	'''
	x,y,z = dirbase(axis[1])
	return Mesh(
		typedlist([axis[0]+0.6*width*p   for p in ((x+y), (y-x), (-y-x), (-y+x))]),
		typedlist([uvec3(0,1,2), uvec3(2,3,0)]),
		groups=[None],
		)

def icosahedron(center:vec3, radius:float) -> 'Mesh':
	''' a simple icosahedron (see https://en.wikipedia.org/wiki/Icosahedron) '''
	phi = (1+ sqrt(5)) /2	# golden ratio
	m = Mesh(
		typedlist([
			vec3(0, 1, phi),
			vec3(1, phi, 0),
			vec3(phi, 0, 1),
			vec3(0, -1, phi),
			vec3(-1, phi, 0),
			vec3(phi, 0, -1),
			vec3(0, 1, -phi),
			vec3(1, -phi, 0),
			vec3(-phi, 0, 1),
			vec3(0, -1, -phi),
			vec3(-1, -phi, 0),
			vec3(-phi, 0, -1),
		]),
		typedlist([
			uvec3(0,1,4), uvec3(0,2,1), uvec3(0,3,2), uvec3(0,8,3), uvec3(0,4,8),
			uvec3(2,5,1), uvec3(1,6,4), uvec3(4,11,8), uvec3(8,10,3), uvec3(3,7,2),
			uvec3(3,10,7), uvec3(2,7,5), uvec3(1,5,6), uvec3(4,6,11), uvec3(8,11,10),
			uvec3(7,9,5), uvec3(5,9,6), uvec3(6,9,11), uvec3(11,9,10), uvec3(10,9,7),
		]),
		)
	f = radius/length(m.points[0])
	for i,p in enumerate(m.points):
		m.points[i] = f*p + center
	return m

def icosphere(center:vec3, radius:float, resolution=None) -> 'Mesh':
	''' a simple icosphere with an arbitrary resolution (see https://en.wikipedia.org/wiki/Geodesic_polyhedron).
	
		Points are obtained from a subdivided icosahedron and reprojected on the desired radius.
	'''
	ico = icosahedron(center, radius)
	div = settings.curve_resolution(2/6*pi*radius, 2/6*pi, resolution)
	ico = subdivide(ico, div-1)
	for i,p in enumerate(ico.points):
		ico.points[i] = center + radius * normalize(p-center)
	return ico

def uvsphere(center:vec3, radius:float, alignment=vec3(0,0,1), resolution=None) -> 'Mesh':
	''' a simple uvsphere (simple sphere obtained with a revolution of an arc) '''
	x,y,z = dirbase(alignment)
	mesh = revolution(2*pi, 
			(center, z),
			web(primitives.ArcCentered(
				(center,x), 
				center+radius*z, 
				center-radius*z, 
				resolution=resolution)),
			resolution=resolution)
	mesh.mergeclose()
	return mesh

def regon(axis:primitives.Axis, radius, n, alignment=None) -> 'Wire':
	''' create a regular n-gon `Wire`, the same way we create a `Circle` '''
	return primitives.Circle(axis, radius, 
				resolution=('div',n), 
				alignment=alignment or vec3(1,0,0),
				).mesh() .segmented()

def repeat(pattern, n:int, transform):
	''' create a mesh duplicating n times the given pattern, each time applying the given transform.
		
		Parameters:
		
			pattern:   can either be a `Mesh`, `Web` or `Wire`   
						the return type will depend on the input type
			n:         the number of repetitions
			transform:     is the transformation between each duplicate
	'''
	current = pattern
	pool = type(pattern)(groups=pattern.groups)
	if n:	pool += current
	for i in range(1,n):
		current = current.transform(transform)
		pool += current
	return pool


