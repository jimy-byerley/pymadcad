from mesh import Mesh, edgekey, MeshError, web
from mathutils import vec2,mat2,vec3,vec4,mat3,mat4, mat3_cast, quat, rotate, translate, dot, cross, perpdot, project, scaledir, transform, normalize, inverse, length, cos, asin, acos, NUMPREC, COMPREC, angleAxis, angle, distance, anglebt, interpol1, spline
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
	for (a,b),t in zip(web.lines, web.tracks):
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
			yield translate(rotate(translate(mat4(1), -axis[0]), i/(steps-1)*angle, axis[1]), axis[0])
	return extrans(web, trans(), ((i,i+1) for i in range(steps-1)))

def saddle(web1, web2):
	''' create a surface by extruding outine1 translating each instance to the next point of outline2
	'''
	def trans():
		s = web2.points[0]
		for p in web2.points:
			yield translate(mat4(1), p-s)
	return extrans(web1, trans(), web2.lines)

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
		for (c,d),t in zip(web.lines, web.tracks):
			mesh.faces.append((al+c, al+d, bl+d))
			mesh.faces.append((al+c, bl+d, bl+c))
			mesh.tracks.append(t)
			mesh.tracks.append(t)
	return mesh
	
def curvematch(line1, line2):
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




def makeloops(lines, faces=()):
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
				#elif edge[0] == loop[0]:	loop[0:1] = reversed(edge)
				#elif edge[-1] == loop[-1]:	loop[-1:] = reversed(edge)
				else:
					continue
				lines.pop(i)
				found = True
				break
		loops.append(loop)
	regularizeloops(loops, faces)
	
	return loops

def regularizeloops(loops, faces=()):
	''' cut the loops when tere is subloops inside it.
		if faces are provided, this function also make sure loops are not enclosing faces 
		preserves the orientation of each loop
	'''
	# cut the loops when there is subloops inside it
	for i,loop in enumerate(loops):
		# merge with loops that share points with current loops
		for j in range(len(loop)):
			k = i+1
			while k < len(loops):
				if loops[k][0] == loop[j]:	loop[j:j+1] = loops.pop(k)
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
					print('disarmed')
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
		best_surf = 0.0
		index = None
		l = len(pts)
		for i in range(l):
			o = pts[i]
			u = pts[(i+1)%l] - o
			v = pts[(i-1)%l] - o
			surf = perpdot(u,v) #/ (length(u)+length(v)+length(u-v))		# ratio surface/perimeter -> leads to equilateral triangles
			# greatest surface
			if surf < best_surf:	continue
			# check hat there is not point of the outline inside the triangle
			decomp = inverse(mat2(u,v))
			inside = False
			for j,p in enumerate(pts):
				if j != i and j != (i-1)%l and j != (i+1)%l:
					uc,vc = decomp * (p-o)
					if 0 < uc and 0 < vc and uc+vc < 1:
						inside = True
						break
			if inside:	continue
			# there is no more tests
			best_surf = surf
			index = i
		
		# create face using this vertex
		if index is None: 
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
	if isinstance(outline, Outline):
		outline = outline.points
	mesh = Mesh(outline, [], [], [])
	triangulate(mesh, list(range(len(outline))))
	return mesh




if __name__ == '__main__':
	from nprint import nprint
	from math import pi
	from mesh import web, Web, Wire
	import view
	from primitives import Arc, Segment
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
			Arc(vec3(0.9,0.5,0), vec3(0.7,0,0), vec3(0.9,-0.5,0)), 
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
	#m3.options.upgrade({'debug_display': True, 'debug_points', True})
	m4.transform(vec3(-4,0,0))
	scn3D.add(m3)
	
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
	#m.options.upgrade({'debug_display': True, 'debug_points', True})
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
	
	#m4.options.upgrade({'debug_display': True, 'debug_points', True})
	scn3D.add(m5)
	
	
	scn3D.look(m3.box())
	main.show()
	sys.exit(app.exec())
