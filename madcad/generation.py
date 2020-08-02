# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import (
					vec2,mat2,dvec2,dmat2,vec3,vec4,mat3,mat4, mat3_cast, quat, 
					rotate, translate, dot, cross, perpdot, project, noproject, scaledir, 
					dirbase, transform, normalize, inverse, length, 
					cos, asin, acos, sqrt, atan2, 
					NUMPREC, COMPREC, pi, 
					angleAxis, angle, distance, anglebt, interpol1, spline, mix,
					)
from .mesh import Mesh, edgekey, MeshError, web, distance_pa
from . import triangulation
from . import settings
from . import primitives
from .nprint import nprint


__all__ = [
	'extrans', 'extrusion', 'revolution', 'saddle', 'tube',
	'curvematch', 'join', 'junction', 'junctioniter',
	'matchexisting', 'matchclosest', 'dividematch',
	'flatsurface',
	'icosahedron', 'icosphere', 'uvsphere',
	]


def extrusion(displt, web):
	''' create a surface by extruding the given outline by a displacement vector '''
	web.strippoints()
	mesh = Mesh([], [], [], web.groups)
	mesh.points.extend(web.points)
	mesh.points.extend((p+displt for p in web.points))
	l = len(web.points)
	for (a,b),t in zip(web.edges, web.tracks):
		mesh.faces.append((a, b,   b+l))
		mesh.faces.append((a, b+l, a+l))
		mesh.tracks.append(t)
		mesh.tracks.append(t)
	return mesh

def revolution(angle, axis, web, resolution=None):
	''' create a revolution surface by extruding the given outline
		`steps` is the number of steps between the start and the end of the extrusion
	'''
	web.strippoints()
	# get the maximum radius, to compute the curve resolution
	radius = 0
	for pt in web.points:
		v = pt-axis[0]
		v -= project(v,axis[1])
		radius = max(radius, length(v))
	steps = settings.curve_resolution(angle*radius, angle, resolution)+2
	# use extrans
	def trans():
		for i in range(steps):
			#yield translate(rotate(translate(mat4(1), -axis[0]), i/(steps-1)*angle, axis[1]), axis[0])
			r = mat3_cast(angleAxis(i/(steps-1)*angle, axis[1]))
			m = mat4(r)
			m[3] = vec4(axis[0] - r*axis[0], 1)
			yield m
	return extrans(web, trans(), ((i,i+1) for i in range(steps-1)))

def saddle(web1, web2):
	''' create a surface by extruding outine1 translating each instance to the next point of outline2
	'''
	def trans():
		s = web2.points[0]
		for p in web2.points:
			yield translate(mat4(1), p-s)
	return extrans(web1, trans(), web2.edges)

def tube(web, path, end=True, section=True):
	''' create a tube surface by extrusing the outline 1 along the outline 2
		if section is True, there is a correction of the segments to keep the section undeformed by the curve
	'''
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
				cl = length(c)
				cn = c/cl
				ha = asin(cl)/2
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
	return extrans(web, trans, ((i,i+1) for i in range(len(path)-1)))


def extrans(web, transformations, links):
	''' create a surface by extruding and transforming the given outline.
		transformations must be an iterable of mat4
	'''
	mesh = Mesh([], [], [], web.groups)
	l = len(web.points)
	transformations = iter(transformations)
	for k,trans in enumerate(transformations):
		for p in web.points:
			mesh.points.append(vec3(trans*vec4(p,1)))
	for (a,b) in links:
		al = a*l
		bl = b*l
		for (c,d),t in zip(web.edges, web.tracks):
			mesh.faces.append((al+c, al+d, bl+d))
			mesh.faces.append((al+c, bl+d, bl+c))
			mesh.tracks.append(t)
			mesh.tracks.append(t)
	return mesh
	
def curvematch(line1, line2):
	''' yield couples of point indices where the curved absciss are the closest '''
	yield line1.indices[0], line2.indices[0]
	l1, l2 = line1.length(), line2.length()
	i1, i2 = 1, 1
	x1, x2 = 0, 0
	while i1 < len(line1.indices) and i2 < len(line2.indices):
		p1 = distance(line1.points[line1.indices[i1-1]], line1.points[line1.indices[i1]]) / l1
		p2 = distance(line2.points[line2.indices[i2-1]], line2.points[line2.indices[i2]]) / l2
		if x1 <= x2 and x2 <= x1+p1   or   x2 <= x1 and x1 <= x2+p2:
			i1 += 1; x1 += p1
			i2 += 1; x2 += p2
		elif x1 < x2:	
			i1 += 1; x1 += p1
		else:				
			i2 += 1; x2 += p2
		yield line1.indices[i1-1], line2.indices[i2-1]
	while i1 < len(line1.indices):
		i1 += 1
		yield line1.indices[i1-1], line2.indices[i2-1]
	while i2 < len(line2.indices):
		i2 += 1
		yield line1.indices[i1-1], line2.indices[i2-1]

def join(mesh, line1, line2):
	''' simple inplace junction of line1 and line2 using mesh indices for lines '''
	group = len(mesh.groups)
	mesh.groups.append('junction')
	match = iter(curvematch(Wire(mesh.points, line1), Wire(mesh.points, line2)))
	last = next(match)
	for couple in match:
		a,b = last
		d,c = couple
		if b == c:		mktri(mesh, (a,b,c), group)
		elif a == d:	mktri(mesh, (a,b,c), group)
		else:
			mkquad(mesh, (a,b,c,d), group)
		last = couple
			
def matchexisting(line1, line2) -> [(vec3,vec3)]:
	''' create couple of points using curvematch '''
	return  ( (line1.points[i1], line2.points[i2]) 
				for i1,i2 in curvematch(line1, line2))

def matchclosest(line1, line2) -> [(vec3, vec3)]:
	''' create couple of points by cutting each line at the curvilign absciss of the points of the other '''
	l1, l2 = line1.length(), line2.length()
	p1, p2 = line1.points[line1.indices[0]], line2.points[line2.indices[0]]
	x = 0
	i1, i2 = 1, 1
	yield (p1, p2)
	while i1 < len(line1.indices) and i2 < len(line2.indices):
		n1 = line1.points[line1.indices[i1]]
		n2 = line2.points[line2.indices[i2]]
		dx1 = distance(p1, n1) / l1
		dx2 = distance(p2, n2) / l2
		if dx1 > dx2:
			x += dx2
			p1 = interpol1(p1, n1, dx2/dx1)
			p2 = n2
			i2 += 1
		else:
			x += dx1
			p1 = n1
			p2 = interpol1(p2, n2, dx1/dx2)
			i1 += 1
		yield (p1, p2)

def dividematch(match, resolution=None):
	''' insert additional couples to ensure smoothness '''
	match = iter(match)
	last = next(match)
	yield last
	for couple in match:
		a,b = last
		d,c = couple
		v = normalize((a+b) - (d+c))
		angle = anglebt(a-b - project(a-b,v), d-c - project(d-c,v))
		dist = min(distance(a,b), distance(d,c))
		div = settings.curve_resolution(dist, angle, resolution)
		for j in range(div):
			t = (j+1)/(div+1)
			yield (interpol1(a,d, t), interpol1(b,c, t))
		yield couple
		last = couple

def junction(match, resolution=None) -> Mesh:
	''' create a surface using couple of points
		resolution is the segments curve resolution
		linecut allow the function to subdivide portions of the given lines to get better result on smooth surfaces
	'''	
	match = list(match)
	# get the discretisation
	segts = 0
	for i in range(1, len(match)):
		a,d = match[i-1]
		b,c = match[i]
		v = normalize((a+b) - (d+c))
		angle = anglebt(a-b - project(a-b,v), d-c - project(d-c,v))
		dist = min(distance(a,b), distance(d,c))
		lsegts = settings.curve_resolution(dist, angle, resolution) + 2
		if lsegts > segts:
			segts = lsegts
	
	return junctioniter(match, segts-2, interpol1)

def junctioniter(parameters, div, interpol) -> Mesh:
	''' create a junction surface using the matching parameters and the given interpolation 
		parameters is an iterable of tuples of arguments for the interpolation function
		interpol receive the elements iterated and the interpolation position at the end
	'''
	segts = div+2
	mesh = Mesh([], [], [], ['junction'])
	group = 0
	# create interpolation points
	steps = 0
	for params in parameters:
		steps += 1
		for i in range(segts):
			x = i/(segts-1)
			mesh.points.append(interpol(*params, x))
	# create faces
	for i in range(steps-1):
		j = i*segts
		for k in range(segts-1):
			s = j+k
			mkquad(mesh, (s, s+1, s+segts+1, s+segts), group)
	mesh.mergeclose()
	return mesh
		
		
def mktri(mesh, pts, track):
	mesh.faces.append(pts)
	mesh.tracks.append(track)

def mkquad(mesh, pts, track):
	if (	distance(mesh.points[pts[0]], mesh.points[pts[2]]) 
		<=	distance(mesh.points[pts[1]], mesh.points[pts[3]]) ):
		mesh.faces.append((pts[:-1]))
		mesh.faces.append((pts[3], pts[0], pts[2]))
	else:
		mesh.faces.append((pts[0], pts[1], pts[3]))
		mesh.faces.append((pts[2], pts[3], pts[1]))
	mesh.tracks.append(track)
	mesh.tracks.append(track)

def facekeyo(a,b,c):
	if a < b and b < c:		return (a,b,c)
	elif a < b:				return (c,a,b)
	else:					return (b,c,a)


def prolongation(mesh, line): 	# TODO
	''' donne les directions d'extrusion des points de la ligne pour prolonger les faces qui y donnent lieu '''
	indev()




def regularizeloops(loops, faces=()):	# NOTE A REVOIR
	''' cut the loops when tere is subloops inside it.
		if faces are provided, this function also make sure loops are not enclosing faces 
		preserves the orientation of each loop
	'''
	print('regularize', loops)
	# cut the loops when there is subloops inside it
	for i,loop in enumerate(loops):	
		# merge with loops that share points with current loops
		for j in range(len(loop)):
			k = i+1
			while k < len(loops):
				if loops[k][0] == loop[j] and loops[k][-1] == loop[j]:	loop[j:j+1] = loops.pop(k)
				else:	k += 1
		# cut loop in subloops
		j = 1
		while j < len(loop):
			p = loop[j]
			try:	k = loop.index(p, j+1)
			except ValueError:	
				j += 1
			else:
				# avoid creating loops that enclose faces
				tri = (loop[j], loop[j+1], loop[k-1])
				if tri in faces or (tri[1],tri[2],tri[0]) in faces or (tri[2],tri[0],tri[1]) in faces:
					j += 1
					continue
				cut = loop[j:k+1]
				# reorient the cutted loop - for unoriented edges
				#for edge in edges:
					#if   edge[0]==cut[0] and edge[1]==cut[1]:		
						#break
					#elif edge[1]==cut[0] and edge[0]==cut[1]:
						#cut.reverse()
						#break
				# correct loops
				loops.append(cut)
				loops[i] = loop = loop[:j]+loop[k:]

def closeenvelope(mesh):	# TODO: refaire
	''' create surfaces to close the loop holes in the envelope '''
	for hole in loopholes(mesh):
		hole.pop()
		triangulate(mesh, hole)

def flatsurface(outline, normal=None):
	m = triangulation.triangulation_outline(outline, normal)
	if normal and dot(m.facenormal(0), normal) < 0:
		m.flip()
	return m

def arclen(p1, p2, n1, n2):
	''' approximated length of an arc between p1 and p2, with associated normals '''
	c = dot(n1,n2)
	v = p1-p2
	return sqrt(dot(v,v) / (2-2*c)) * acos(c)
			
def icosurface(pts, ptangents, etangents=None, resolution=None):
	''' generate a surface ICO (a subdivided triangle) with its points interpolated using interpol2tri 
		if normals are given instead of point tangents (for ptangents), the surface will fit a sphere
		else ptangents must be a list of couples (2 edge tangents each point)
		if etangents is not given, it will be computed perpendicular the edge's point's normals
	'''
	# compute normals to points
	if isinstance(ptangents[0], tuple):
		normals = [None]*3
		for i in range(3):
			normals[i] = normalize(cross(ptangents[i][0], ptangents[i][1]))
	else:
		normals = ptangents
	
	# compute tangents to side splines
	if etangents is None:
		etangents = [None]*3
		for i in range(3):
			etangents[i] = normalize(cross(normals[i-2], normals[i-1]))
	
	# if normals are given instead of tangents, compute tangents to fit a sphere surface
	if not isinstance(ptangents[0], tuple):
		ptangents = [None]*3
		for i in range(3):
			n = (pts[i], normals[i])
			dist = (  distance_pa(pts[i-1], n) 
					+ distance_pa(pts[i-2], n) )/2
			angle = anglebt(normals[i], normalize(normals[i-1]+normals[i-2]))
			etangents[i] *= dist*angle
			ptangents[i] = (
				normalize(noproject(pts[i-2]-pts[i], normals[i])) * arclen(pts[i], pts[i-2], normals[i], normals[i-2]),
				normalize(noproject(pts[i-1]-pts[i], normals[i])) * arclen(pts[i], pts[i-1], normals[i], normals[i-1]),
				)
	
	# evaluate resolution (using a bad approximation of the distance for now)
	div = max(( settings.curve_resolution(
					distance(pts[i-1], pts[i-2]), 
					anglebt(normals[i-1], normals[i-2]), 
					resolution)
				for i in range(3) ))
	
	# place points
	mesh = Mesh(groups=['interptri', 'flat'])
	for i in range(div):
		u = i/(div-1)				
		for j in range(div-i):
			v = j/(div-1)
			p = interpol2tri(pts, ptangents, etangents, u,v)
			mesh.points.append(p)
	# create faces
	c = 0
	for i in reversed(range(1,div+1)):
		for j in range(i-1):
			s = c+j
			mesh.faces.append((s, s+i, s+1))
		for j in range(1,i-1):
			s = c+j
			mesh.faces.append((s, s+i-1, s+i))
		c += i
	mesh.tracks = [0] * len(mesh.faces)

	return mesh

def interpol2tri(pts, ptangents, etangents, a,b):
	''' cubic interpolation like interpol2, but interpolates over a triangle (2d parameter space) '''
	A,B,C = pts
	ta,tb,tc = ptangents
	tbc,tca,tab = etangents
	c = 1-a-b
	return (	0
			+	a**2 * (A + b*ta[0] + c*ta[1])
			+	b**2 * (B + c*tb[0] + a*tb[1])
			+	c**2 * (C + a*tc[0] + b*tc[1])
			+	2*(b*a) * ((a*A + b*B)/(a+b+1e-5) + c*tab)
			+	2*(c*b) * ((b*B + c*C)/(b+c+1e-5) + a*tbc)
			+	2*(a*c) * ((c*C + a*A)/(c+a+1e-5) + b*tca)
			)

def icosahedron(center, radius):
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

def subdivide(mesh, div=1):
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



def icosphere(center, radius, resolution=None):
	ico = icosahedron(center, radius)
	div = settings.curve_resolution(2/6*pi*radius, 2/6*pi, resolution)
	ico = subdivide(ico, div-1)
	for i,p in enumerate(ico.points):
		ico.points[i] = center + radius * normalize(p-center)
	return ico

def uvsphere(center, radius, alignment=vec3(0,0,1), resolution=None):
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

