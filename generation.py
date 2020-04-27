from mesh import Mesh, edgekey, MeshError, web, distance_pa
from mathutils import vec2,mat2,dvec2,dmat2,vec3,vec4,mat3,mat4, mat3_cast, quat, rotate, translate, dot, cross, perpdot, project, noproject, scaledir, transform, normalize, inverse, length, cos, asin, acos, sqrt, NUMPREC, COMPREC, angleAxis, angle, distance, anglebt, interpol1, spline
from math import atan2
from primitives import Primitive
import settings
from nprint import nprint


__all__ = [
	'extrans', 'extrusion', 'revolution', 'saddle', 'tube',
	'curvematch', 'join', 'junction', 'junctioniter',
	'matchexisting', 'matchclosest', 'dividematch',
	'triangulate', 'flatsurface', 'makeloops', 'loopholes', 'closeenvelope', 
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




def makeloops(lines, faces=(), oriented=True):	# TODO: mettre en commun avec boolean et cut
	''' return a list of the loops that can be formed with lines.
		lines must be oriented suite of points and loops are returned as suite of points.
	'''
	lines = list(lines)
	# get contiguous suite of points
	loops = []
	while lines:
		loop = list(lines.pop())
		found = True
		while found:
			found = False
			for i,edge in enumerate(lines):
				if edge[-1] == loop[0]:		loop[0:1] = edge
				elif edge[0] == loop[-1]:	loop[-1:] = edge
				# for unoriented lines
				elif not oriented and edge[0] == loop[0]:	loop[0:1] = reversed(edge)
				elif not oriented and edge[-1] == loop[-1]:	loop[-1:] = reversed(edge)
				else:
					continue
				lines.pop(i)
				found = True
				break
		loops.append(loop)
	# cut at loop intersections (sub loops or crossing loops)
	reach = Counter()
	for loop in loops:
		for p in loop:
			reach[p] += 1
	for loop in loops:
		for i in range(1,len(loop)-1):
			if reach[loop[i]] > 1:
				loops.append(loop[i:])
				loop[i+1:] = []
				break
	return loops

from collections import Counter

def regularizeloops(loops, faces=()):
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

def loopholes(mesh):
	''' return the loop delimiting holes of a mesh '''
	return makeloops(mesh.outlines_oriented(), mesh.faces)
	

def closeenvelope(mesh):
	''' create surfaces to close the loop holes in the envelope '''
	for hole in loopholes(mesh):
		hole.pop()
		triangulate(mesh, hole)

def triangulate(mesh, outline, group=None):
	''' insert triangles in mesh in order to create a flat surface in the outline 
		outline must be oriented
		if group is specified, the new faces will use this group id
	'''
	if not group:	
		group = len(mesh.groups)
		mesh.groups.append('flat')
	
	# project outline points in a plane
	x,y,z = makebase([mesh.points[i] for i in outline])
	pts = [vec2(dot(mesh.points[p],x),dot(mesh.points[p],y)) for p in outline]
	
	# make sure the projection respects the outline rotation
	i = max(range(len(pts)), key=lambda i: pts[i][0])
	s = perpdot(pts[(i+1)%len(pts)]-pts[i], pts[i-1]-pts[i])
	if s < 0:
		for p in pts:	p[1] = -p[1]
		
	# create faces
	while len(outline) > 2:
		best_surf = -NUMPREC
		index = None
		l = len(pts)
		for i in range(l):
			o = pts[i]
			u = pts[(i+1)%l] - o
			v = pts[(i-1)%l] - o
			#surf = perpdot(u,v) 
			#surf = perpdot(u,v) / (length(u)+length(v)+length(u-v))		# ratio surface/perimeter -> leads to equilateral triangles
			surf = anglebt(u,v)
			# greatest surface
			if surf < best_surf:	continue
			# check hat there is not point of the outline inside the triangle
			decomp = inverse(dmat2(u,v))
			inside = False
			for j,p in enumerate(pts):
				if j != i and j != (i-1)%l and j != (i+1)%l:
					uc,vc = decomp * dvec2(p-o)
					if 0 < uc and 0 < vc and uc+vc < 1:
						inside = True
						break
			if inside:	continue
			# there is no more tests
			best_surf = surf
			index = i
		
		# create face using this vertex
		if index is None: 
			print(surf, best_surf)
			raise MeshError("no more feasible triangles in "+str(outline))
		mesh.faces.append((outline[(index-1)%l], outline[index], outline[(index+1)%l]))
		mesh.tracks.append(group)
		outline.pop(index)
		pts.pop(index)

def makebase(points):
	''' create a base that keep the points in the XY plane (list version, use the center of the points) '''
	center = vec3(0)
	for p in points:	center += p
	center /= len(points)
	for p in points:
		x = p - center
		if length(x) > NUMPREC:	break
	for p in points:
		y = p - center
		if length(y) > NUMPREC and distance(x,y) > NUMPREC:	break
	x = normalize(x)
	y -= project(y,x)
	z = cross(x,y)
	return x,y,z


def flatsurface(outline):
	''' create a surface in the given outline '''
	mesh = Mesh(outline.points, [], [], [])
	triangulate(mesh, outline.indices)
	return mesh


def arclen(p1, p2, n1, n2):
	''' length of an arc between p1 and p2, with associated normals '''
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

if __name__ == '__main__':
	from nprint import nprint
	from math import pi
	from mesh import web, Web, Wire
	import view
	from primitives import ArcThrough, Segment
	import sys
	from PyQt5.QtWidgets import QApplication
	
	app = QApplication(sys.argv)
	main = scn3D = view.Scene()
	
	# test extrusion
	m1 = extrusion(vec3(0,0,0.5), Web(
			[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0)],
			[(0,1), (1,2), (2,3), (3,0)],
			[0,1,2,3],
			))
	m1.check()
	assert m1.issurface()
	
	# test revolution
	m2 = revolution(pi, (vec3(0,0,0), vec3(0,1,0)), web(
			Segment(vec3(1,1,0), vec3(0.9,0.5,0)), 
			ArcThrough(vec3(0.9,0.5,0), vec3(0.7,0,0), vec3(0.9,-0.5,0)), 
			Segment(vec3(0.9,-0.5,0), vec3(1,-1,0)),
			))
	m2.check()
	assert m2.issurface()
	
	# test saddle
	m3 = saddle(
			Web(
				[vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], 
				[(0,1), (1,2), (2,3), (3,4)],
				[0,1,2,3]),
			web(vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)),
			#web(Arc(vec3(0,1,-1),vec3(0,1.5,0),vec3(0,1,1))),
			)
	m3.check()
	assert m3.issurface()
	#m.options.update({'debug_display': True, 'debug_points': False })
	scn3D.add(m3)
	
	# test tubes
	m4 = tube(
			Web(
				[vec3(1,0,0), vec3(0,1,0), vec3(-1,0,0), vec3(0,-1,0), vec3(1,0,0)],
				[(0,1),(1,2),(2,3),(3,0)],
				[0,1,2,3],
				),
			#Arc(vec3(0,0,0), vec3(4,1,4), vec3(6,0,3)).mesh()[0],
			[vec3(0,0,0), vec3(0,0,2), vec3(1,0,3), vec3(4,0,3)],
			)
	m4.check()
	assert m4.issurface()
	#m4.options.update({'debug_display': True, 'debug_points': True})
	m4.transform(vec3(-4,0,0))
	scn3D.add(m4)
	
	# test closeholes
	m = m1+m2
	m.mergeclose()
	m.strippoints()
	#print(loopholes(m))
	#assert len(loopholes(m)) == 5, len(loopholes(m))
	#closeenvelope(m)
	#m.check()
	#assert m.issurface()
	#assert m.isenvelope()
	m.transform(vec3(0,0,4))
	#m.options.update({'debug_display': True, 'debug_points': True})
	scn3D.add(m)
	
	# test junction
	m5 = junction(
		dividematch(
		matchclosest(
			Wire([vec3(-2,0,0), vec3(-1,0,0), vec3(0,0,0), vec3(1,0,0), vec3(2,0,0)]),
			Wire([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)]),
		)))
	m5.check()
	assert m5.issurface()
	#m4.options.update({'debug_display': True, 'debug_points': True})
	scn3D.add(m5)
	
	# test icosurface
	m6 = icosurface(
		[vec3(0.5,-1,0), vec3(0,1,0), vec3(0,0,1)], 
		[normalize(vec3(0.5,-1,0)), vec3(0,1,0), vec3(0,0,1)], 
		#[vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)], 
		#[1.57*vec3(1,0,0), 1.57*vec3(0,1,0), 1.57*vec3(0,0,1)],  
		#[1.57*vec3(1,0,0), 1.57*vec3(0,1,0), 1.57*vec3(0,0,1)], 
		)
	m6.check()
	assert m6.issurface()
	#m6.options.update({'debug_display': True, 'debug_points':True})
	m6.transform(vec3(0,0,-4))
	scn3D.add(m6)
	
	
	scn3D.look(m6.box())
	main.show()
	sys.exit(app.exec())
