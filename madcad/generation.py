# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .mesh import Mesh, Web, Wire, edgekey, facekeyo, MeshError, web, wire, mkquad
from . import triangulation
from . import settings
from . import primitives
from .nprint import nprint

from copy import copy


__all__ = [
	'extrans', 'extrusion', 'revolution', 'saddle', 'tube', 
	'repeat', 'thicken', 'inflate', 'inflateoffsets',
	'flatsurface', 'icosurface', 'subdivide',
	'square', 'brick', 'icosahedron', 'icosphere', 'uvsphere', 'regon', 
	]

	
# --- extrusion things ---


def extrusion(trans, line, alignment=0):
	''' create a surface by extruding the given outline by a transformation
		
		parameters:
			:line:         a line (Web or Wire) or a surface (Mesh) to extrude
			:trans:        any transformation object accepted by `mathutils.transform`
			:alignment:	   when > 0 the line is extruded both sides (the transformation is linearly interpoled)
	'''
	trans = transform(trans)
	neutral = mat4()
	return extrans(line, [
				neutral + (trans-neutral)*(-alignment),
				neutral + (trans-neutral)*(1-alignment),
				],
				[(0,1,0)])

def revolution(angle, axis, profile, resolution=None):
	''' create a revolution surface by extruding the given outline
		`steps` is the number of steps between the start and the end of the extrusion
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

def saddle(web1, web2):
	''' create a surface by extruding outine1 translating each instance to the next point of outline2
	'''
	web1, web2 = web(web1), web(web2)
	def trans():
		s = web2.points[0]
		for p in web2.points:
			yield translate(mat4(1), p-s)
	return extrans(web1, trans(), ((*e,t)  for e,t in zip(web2.edges, web2.tracks)))

def tube(outline, path, end=True, section=True):
	''' create a tube surface by extrusing the outline along the path
		if section is True, there is a correction of the segments to keep the section undeformed by the curve
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


def extrans(section, transformations, links) -> 'Mesh':
	''' create a surface by extruding and transforming the given outline.
		
		:transformations:   iterable of mat4, one each section
		:link:              iterable of tuples (a,b,t)  with:
								`(a,b)` the sections to link (indices of values returned by `transformation`).
								`t` the group number of that link, to combine with the section groups		
	'''
	# prepare
	face = None
	if isinstance(section, Mesh):
		face = copy(section)
		face.strippoints()
		section = section.outlines()
	else:
		section = web(section)
	section = copy(section)
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
				merges[src + len(mesh.points)] = dst + l*k
			end = face .transform(kept[k])
			mesh += end if extremities[k] else end.flip()
		mesh.mergepoints(merges)
	
	return mesh
	
def linstep(start, stop, x):
	if x <= start:	return 0
	if x >= stop:	return 1
	return (x-start)/(stop-start)

def inflateoffsets(surf, distance, method='face') -> '[vec3]':
	''' displacements vectors for points of a surface we want to inflate.
		
		:method:     
			determines if the distance is from the old to the new faces, edges or points
			possible values: `'face', 'edge', 'point'`
	'''
	pnormals = surf.vertexnormals()
	
	# smooth normal offsets laterally when they are closer than `distance`
	outlines = surf.outlines_oriented()
	l = len(pnormals)
	normals = deepcopy(pnormals)
	for i in range(5):	# the number of steps is the diffusion distance through the mesh
		for a,b in outlines:
			d = surf.points[a] - surf.points[b]		# edge direction
			t = cross(pnormals[a]+pnormals[b], d)	# surface tangent normal to the edge
			# contribution stars when the offseted points are closer than `distance`
			contrib = 1 - smoothstep(0, distance, length(distance*(pnormals[a]-pnormals[b])+d))
			normals[a] += contrib * 0.5*project(pnormals[b]-pnormals[a], t)
			normals[b] += contrib * 0.5*project(pnormals[a]-pnormals[b], t)
		# renormalize
		for i in range(l):
			pnormals[i] = normals[i] = normalize(normals[i])
	
	# compute offset length depending on the method
	if method == 'face':
		lengths = [inf]*len(pnormals)
		for face in surf.faces:
			fnormal = surf.facenormal(face)
			for p in face:
				lengths[p] = min(lengths[p], 1/dot(pnormals[p], fnormal))
		return [pnormals[p]*lengths[p]*distance   for p in range(len(pnormals))]
	
	elif method == 'edge':
		lengths = [inf]*len(pnormals)
		for edge,enormal in surf.edgenormals().items():
			for p in edge:
				lengths[p] = min(lengths[p], 1/dot(pnormals[p], enormal))
		return [pnormals[p]*lengths[p]*distance	for p in range(len(pnormals))]
		
	elif method == 'point':
		return [pnormals[p]*distance	for p in range(len(pnormals))]

def inflate(surf, distance, method='face') -> 'Mesh':
	''' move all points of the surface to make a new one at a certain distance of the last one

		:method:       determines if the distance is from the old to the new faces, edges or points
	'''
	return Mesh([p+d   for p,d in zip(surf.points, inflateoffsets(surf, distance, method))],
				surf.faces,
				surf.tracks,
				surf.groups)

def thicken(surf, thickness, alignment=0, method='face') -> 'Mesh':
	''' thicken a surface by extruding it, points displacements are made along normal. 

		:thickness:    determines the distance between the two surfaces (can be negative to go the opposite direction to the normal).
		:alignment:    specifies which side is the given surface: 0 is for the first, 1 for the second side, 0.5 thicken all apart the given surface.
		:method:       determines if the thickness is from the old to the new faces, edges or points
	'''
	displts = inflateoffsets(surf, thickness, method)
	
	a = alignment
	b = alignment-1
	m = (	Mesh([p+d*a  for p,d in zip(surf.points,displts)], surf.faces[:], surf.tracks[:], surf.groups)
		+	Mesh([p+d*b  for p,d in zip(surf.points,displts)], surf.faces, surf.tracks, surf.groups[:])
			.flip() 
		)
	t = len(m.groups)
	l = len(surf.points)
	m.groups.append('junction')
	for e in surf.outlines_oriented():
		mkquad(m, (e[0], e[1], e[1]+l, e[0]+l), t)
	return m



# --- filling things ---

def flatsurface(outline, normal=None) -> 'Mesh':
	''' generates a surface for a flat outline using the prefered triangulation method .
	
		if `normal` is specified, it must be the normal vector to the plane, and will be used to orient the face.
	'''
	if isinstance(outline, Wire):
		m = triangulation.triangulation_outline(outline, normal)
	else:
		m = triangulation.triangulation(wire(outline), normal)
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
	mesh = Mesh(groups=['blend'])
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
	mesh.tracks = [0] * len(mesh.faces)

	return mesh


def subdivide(mesh, div=1) -> 'Mesh':
	''' subdivide all faces by the number of cuts '''
	n = div+2
	pts = []
	faces = []
	tracks = []
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
				faces.append((s, s+i, s+1))
			for j in range(1,i-1):
				s = c+j
				faces.append((s, s+i-1, s+i))
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
			(0, 1, 2),
			(0, 2, 3),
			(4, 7, 6),
			(4, 6, 5),
			(0, 4, 5),
			(0, 5, 1),
			(1, 5, 6),
			(1, 6, 2),
			(2, 6, 7),
			(2, 7, 3),
			(4, 0, 3),
			(4, 3, 7)],
		[	0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5	],
		[None] * 6,
		)
	for i in range(len(mesh.points)):
		mesh.points[i] = mesh.points[i]*box.width + box.min
	return mesh
	
def square(axis, width:float) -> 'Mesh':
	''' return a simple square with the given normal axis and square width.
		Useful to quickly create a cutplane
	'''
	x,y,z = dirbase(axis[1])
	return Mesh(
		[axis[0]+0.6*width*p   for p in ((x+y), (y-x), (-y-x), (-y+x))],
		[(0,1,2), (2,3,0)],
		groups=['flat'],
		)

def icosahedron(center, radius) -> 'Mesh':
	''' a simple icosahedron (see https://en.wikipedia.org/wiki/Icosahedron) '''
	phi = (1+ sqrt(5)) /2	# golden ratio
	m = Mesh([
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
		],
		[
		(0,1,4), (0,2,1), (0,3,2), (0,8,3), (0,4,8),
		(2,5,1), (1,6,4), (4,11,8), (8,10,3), (3,7,2),
		(3,10,7), (2,7,5), (1,5,6), (4,6,11), (8,11,10),
		(7,9,5), (5,9,6), (6,9,11), (11,9,10), (10,9,7),
		],
		)
	f = radius/length(m.points[0])
	for i,p in enumerate(m.points):
		m.points[i] = f*p + center
	return m

def icosphere(center, radius, resolution=None) -> 'Mesh':
	''' a simple icosphere with an arbitrary resolution (see https://en.wikipedia.org/wiki/Geodesic_polyhedron).
	
		Points are obtained from a subdivided icosahedron and reprojected on the desired radius.
	'''
	ico = icosahedron(center, radius)
	div = settings.curve_resolution(2/6*pi*radius, 2/6*pi, resolution)
	ico = subdivide(ico, div-1)
	for i,p in enumerate(ico.points):
		ico.points[i] = center + radius * normalize(p-center)
	return ico

def uvsphere(center, radius, alignment=vec3(0,0,1), resolution=None) -> 'Mesh':
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

def regon(axis, radius, n, alignment=None) -> 'Wire':
	''' create a regular n-gon `Wire`, the same way we create a `Circle` '''
	return primitives.Circle(axis, radius, 
				resolution=('div',n), 
				alignment=alignment or vec3(1,0,0),
				).mesh() .segmented()

def repeat(pattern, n, trans):
	''' create a mesh duplicating n times the given pattern, each time applying the given transform.
		
		Parameters:
		
			pattern:   can either be a `Mesh`, `Web` or `Wire`   
						the return type will depend on the input type
			n:         the number of repetitions
			trans:     is the transformation between each duplicate
	'''
	current = pattern
	pool = type(pattern)(groups=pattern.groups)
	if n:	pool += current
	for i in range(1,n):
		current = current.transform(trans)
		pool += current
	return pool


