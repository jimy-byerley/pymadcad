# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .mesh import Mesh, Web, Wire, edgekey, facekeyo, MeshError, web, wire, mkquad
from . import triangulation
from . import settings
from . import primitives
from .nprint import nprint


__all__ = [
	'extrans', 'extrusion', 'revolution', 'saddle', 'tube', 
	'multiple', 'thicken', 'inflate', 'inflateoffsets',
	'flatsurface', 'icosurface', 'subdivide',
	'brick', 'icosahedron', 'icosphere', 'uvsphere', 
	]

	
# --- extrusion things ---
	
def extrans_pre(obj):
	face = None
	if isinstance(obj, Web):	
		line = Web(obj.points[:], obj.edges, obj.tracks, obj.groups)
	elif isinstance(obj, Mesh):	
		line = obj.outlines()
		face = obj
	else:
		line = web(obj)
	line.strippoints()
	return line, face
	
def extrans_post(mesh, face, first, last):
	if face:
		mesh += face.transform(first) + face.transform(last).flip()
		

def extrusion(displt, line):
	''' create a surface by extruding the given outline by a displacement vector '''
	line, face = extrans_pre(line)
	mesh = Mesh([], [], [], line.groups)
	mesh.points.extend(line.points)
	mesh.points.extend((p+displt for p in line.points))
	l = len(line.points)
	for (a,b),t in zip(line.edges, line.tracks):
		mesh.faces.append((a, b,   b+l))
		mesh.faces.append((a, b+l, a+l))
		mesh.tracks.append(t)
		mesh.tracks.append(t)
	extrans_post(mesh, face, vec3(0), displt)
	return mesh

def revolution(angle, axis, profile, resolution=None):
	''' create a revolution surface by extruding the given outline
		`steps` is the number of steps between the start and the end of the extrusion
	'''
	if not isinstance(profile, (Mesh,Wire,Web)):	
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
			#yield translate(rotate(translate(mat4(1), -axis[0]), i/(steps-1)*angle, axis[1]), axis[0])
			r = mat3_cast(angleAxis(i/(steps-1)*angle, axis[1]))
			m = mat4(r)
			m[3] = vec4(axis[0] - r*axis[0], 1)
			yield m
	return extrans(profile, trans(), ((i,i+1) for i in range(steps-1)))

def saddle(web1, web2):
	''' create a surface by extruding outine1 translating each instance to the next point of outline2
	'''
	web1, web2 = web(web1), web(web2)
	def trans():
		s = web2.points[0]
		for p in web2.points:
			yield translate(mat4(1), p-s)
	return extrans(web1, trans(), web2.edges)

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
	return extrans(outline, trans, ((i,i+1) for i in range(len(path)-1)))



def extrans(web, transformations, links) -> 'Mesh':
	''' create a surface by extruding and transforming the given outline.
		transformations must be an iterable of mat4
	'''
	web, face = extrans_pre(web)
	mesh = Mesh(groups=web.groups)
	l = len(web.points)
	transformations = iter(transformations)
	first = trans = None
	for k,trans in enumerate(transformations):
		for p in web.points:
			mesh.points.append(vec3(trans*vec4(p,1)))
		if not first:	first = trans
	for (a,b) in links:
		al = a*l
		bl = b*l
		for (c,d),t in zip(web.edges, web.tracks):
			mesh.faces.append((al+c, al+d, bl+d))
			mesh.faces.append((al+c, bl+d, bl+c))
			mesh.tracks.append(t)
			mesh.tracks.append(t)
	extrans_post(mesh, face, first, trans)
	return mesh

def inflateoffsets(surf, distance, method='face') -> '[vec3]':
	''' displacements vectors for points of a surface we want to inflate.
		
		:method:     
			determines if the distance is from the old to the new faces, edges or points
			possible values: `'face', 'edge', 'point'`
	'''
	pnormals = surf.vertexnormals()
	if method == 'face':
		lengths = [0]*len(pnormals)
		for i,face in enumerate(surf.faces):
			fnormal = surf.facenormal(i)
			for p in face:
				lengths[p] = max(lengths[p], 1/dot(pnormals[p], fnormal))
		return [pnormals[p]*lengths[p]*distance   for p in range(len(pnormals))]
	
	elif method == 'edge':
		lengths = [0]*len(pnormals)
		for edge,enormal in surf.edgenormals():
			for p in edge:
				lengths[p] = max(lengths[p], 1/dot(pnormal[p], enormal))
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
		+	Mesh([p+d*b  for p,d in zip(surf.points,displts)], surf.faces, surf.tracks, surf.groups)
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
	''' generates a surface for a flat outline using the prefered triangulation method '''
	m = triangulation.triangulation_outline(wire(outline), normal)
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
	''' a simple brick with rectangular sides '''
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


def multiple(pattern, n, trans=None, axis=None, angle=None) -> 'Mesh':
	''' create a mesh duplicating n times the given pattern, each time applying the given transform.
		
		:pattern:   can either be a `Mesh`, `Web` or `Wire`
		:trans:     is the transformation between each duplicate. If none, `axis` and `angle` must define a rotation around an axis.
	'''
	if trans is None:
		if isinstance(axis, tuple) and len(axis) == 2 and angle:
			trans = transform(axis[0]) * transform(angleAxis(angle, axis[1])) * transform(axis[0])
		else:
			raise TypeError('transform must be set, or axis and angle must be axis and angle for rotation')
	
	current = pattern
	pool = type(pattern)(groups=pattern.groups)
	if n:	pool += current
	for i in range(1,n):
		current = current.transform(trans)
		pool += current
	return pool


