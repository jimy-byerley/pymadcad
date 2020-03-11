from mesh import Mesh, edgekey, MeshError
from mathutils import vec2,mat2,vec3,vec4,mat3,mat4, mat3_cast, quat, rotate, translate, dot, cross, perpdot, project, scaledir, transform, normalize, inverse, length, cos, asin, NUMPREC, COMPREC, angleAxis, angle, distance, anglebt, interpol1
from math import atan2
from primitives import Primitive
import settings
from nprint import nprint

__all__ = [
	'Outline', 'outline', 
	'extrusion', 'revolution', 'extrans', 
	'outlines', 'makeloops', 'loopholes', 'flatsurface', 'closeenvelope', 'triangulate',
	]

class Outline:
	''' Hold datas for creation of surfaces from outlines 
		the outline is a contiguous suite of points linked by segments, tracks are identifiers for each group associated with segments
	'''
	def __init__(self, points, tracks=None, groups=None):
		self.points = points
		self.tracks = tracks or [0] * (len(points)-1)
		self.groups = groups or [None] * (max(self.tracks)+1)
	
	def length(self):
		s = 0
		for i in range(1,len(self.points)):
			s += distance(self.points[i-1], self.points[i])
		return s
	# NOTE: voir si il est plus interessant d'utiliser de lignes continues decrites par une suite de points, ou des lignes quelconques decrites par des couples d'indices pour les aretes


def outline(*args):
	if not args:	raise ValueError('outline take at least one arg')
	if hasattr(args[0], 'mesh'):
		pts,info = args[0].mesh()
		return Outline(pts, groups=[info])
	elif len(args[0]) and isinstance(args[0][0], vec3):
		return Outline(*args)
	else:
		primitives = args[0]
		groups = [None] * len(primitives)	# primitive index is the group id
		line = []	# line as contiguous suite of points
		tracks = []
		
		for i in range(len(primitives)):
			# get the primitive as points
			if not hasattr(primitives[i], '__getitem__'):
				primitives[i], group = primitives[i].mesh()
				groups[i] = group
		
		line = list(primitives.pop())
		tracks = [len(primitives)]*(len(line)-1)
		while primitives:
			found = False
			for i,prim in enumerate(primitives):
				# search for the primitive extremity in the line
				if length(prim[-1] - line[0]) < NUMPREC:
					line[0:0] = prim[:-1]
					tracks[0:0] = [i] * (len(prim)-1)
				elif length(prim[0] - line[-1]) < NUMPREC:	
					line.extend(prim[1:])
					tracks.extend([i] * (len(prim)-1))
				elif length(prim[0] - line[0]) < NUMPREC:	
					line[0:0] = reversed(prim[1:])
					tracks[0:0] = [i] * (len(prim)-1)
				elif length(prim[-1] - line[-1]) < NUMPREC:	
					line.extend(reversed(prim[:-1]))
					tracks.extend([i] * (len(prim)-1))
				else:	continue
				primitives.pop(i)
				found = True
				break
			if not found:
				nprint(primitives)
				raise ValueError('line is discontinuous over points {} and {}'.format(repr(line[0]), repr(line[-1])))
		return Outline(line, tracks, groups)


def extrusion(displt, outline):
	''' create a surface by extruding the given outline by a displacement vector '''
	mesh = Mesh([], [], [], outline.groups)
	mesh.points.extend(outline.points)
	mesh.points.extend((p+displt for p in outline.points))
	l = len(outline.points)
	for i,t in zip(range(l-1), outline.tracks):
		mesh.faces.append((i, i+1, i+1+l))
		mesh.faces.append((i, i+1+l, i+l))
		mesh.tracks.append(t)
		mesh.tracks.append(t)
	return mesh

def revolution(angle, axis, outline, resolution=None):
	''' create a revolution surface by extruding the given outline
		`steps` is the number of steps between the start and the end of the extrusion
	'''
	# get the maximum radius, to compute the curve resolution
	radius = 0
	for pt in outline.points:
		v = pt-axis[0]
		v -= project(v,axis[1])
		radius = max(radius, length(v))
	steps = settings.curve_resolution(angle*radius, angle, resolution)
	# use extrans
	def trans():
		for i in range(steps+1):
			yield translate(rotate(translate(mat4(1), -axis[0]), i/steps*angle, axis[1]), axis[0])
	return extrans(trans(), outline)

def saddle(outline1, outline2):
	''' create a surface by extruding outine1 translating each instance to the next point of outline2
	'''
	def trans():
		for i in range(len(outline2.points)):
			yield translate(mat4(1), outline2.points[i]-outline2.points[0])
	return extrans(trans(), outline1)

def tube(outline1, outline2, end=True, section=True):
	''' create a tube surface by extrusing the outline 1 along the outline 2
		if section is True, there is a correction of the segments to keep the section undeformed by the curve
	'''
	def trans():
		lastrot = quat()
		l = len(outline2.points)-1
		yield mat4(1)
		for i in range(1,l+1):
			
			if i < l:
				# calculer l'angle a ce coude
				o = outline2.points[i]
				v1 = normalize(outline2.points[i-1]-o)
				v2 = normalize(outline2.points[i+1]-o)
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
			yield transform(outline2.points[i]) * mat4(m) * transform(-outline2.points[0])
	
	trans = trans()
	if not end:		next(trans)
	return extrans(trans, outline1)


def extrans(transformations, outline):
	''' create a surface by extruding and transforming the given outline.
		transformations must be an iterable of mat4
	'''
	mesh = Mesh([], [], [], outline.groups)
	l = len(outline.points)
	transformations = iter(transformations)
	for k,trans in enumerate(transformations):
		for p in outline.points:
			mesh.points.append(vec3(trans*vec4(p,1)))
		if k:
			j = k*l
			for i,t in zip(range(l-1), outline.tracks):
				mesh.faces.append((j+i-l, j+i+1-l, j+i+1))
				mesh.faces.append((j+i-l, j+i+1,   j+i))
				mesh.tracks.append(t)
				mesh.tracks.append(t)
	return mesh

def matchcurves(line1, line2):
	match = [(0,0)]
	l1, l2 = line1.length(), line2.length()
	i1, i2 = 1, 1
	x1, x2 = 0, 0
	while i1 < len(line1.points) and i2 < len(line2.points):
		p1 = distance(line1.points[i1-1], line1.points[i1])
		p2 = distance(line2.points[i2-1], line2.points[i2])
		if x1 <= x2 and x2 <= x1+p1   or   x2 <= x1 and x1 <= x2+p2:
			i1 += 1; x1 += p1
			i2 += 1; x2 += p2
		elif x1/l1 < x2/l2:	
			i1 += 1; x1 += p1
		else:				
			i2 += 1; x2 += p2
		match.append((i1-1,i2-1))
	while i1 < len(line1.points):
		i1 += 1
		match.append((i1-1,i2-1))
	while i2 < len(line2.points):
		i2 += 1
		match.append((i1-1,i2-1))
	return match

def simplejunction(mesh, line1, line2):
	group = len(mesh.groups)
	mesh.groups.append('junction')
	match = matchcurves(line1, line2)
	for i in range(1,len(match)):
		a,b = match[i-1]
		d,c = match[i]
		if b == c:		mktri(mesh, (a,b,c), group)
		elif a == d:	mktri(mesh, (a,b,c), group)
		else:
			mkquad(mesh, (a,b,c,d), group)

def junction(line1, line2, resolution=None, linecut=True):
	''' create a smooth surface between two lines, using matchcurves()
		resolution is the segments curve resolution
		linecut allow the function to subdivide portions of the given lines to get better result on smooth surfaces
	'''
	mesh = Mesh([], [], [], ['junction'])
	group = 0
	match = matchcurves(line1, line2)
	match = [(line1.points[i1], line2.points[i2]) for i1,i2 in match]
	
	# get the discretisation
	segts = 0
	i = 1
	while i < len(match):
		a,b = match[i-1]
		d,c = match[i]
		v = normalize((a+b) - (d+c))
		angle = anglebt(a-b - project(a-b,v), d-c - project(d-c,v))
		dist = min(distance(a,b), distance(d,c))
		lsegts = settings.curve_resolution(dist, angle, resolution) + 2
		if lsegts > segts:
			segts = lsegts
		if lsegts and linecut:
			match[i:i] = [	(interpol1(a,d, j/(lsegts-1)), interpol1(b,c, j/(lsegts-1)))
							for j in range(1,lsegts-1)]
			i += lsegts-2
		i += 1
	
	# create interpolation points
	for p1,p2 in match:
		for i in range(segts):
			x = i/(segts-1)
			mesh.points.append(interpol1(p1, p2, x))
	
	# create faces
	for i in range(1,len(match)):
		j = (i-1)*segts
		a,b = match[i-1]
		d,c = match[i]
		if a==d:	mktri(mesh, (j,j+1,j+segts+1), group)
		else:		mkquad(mesh, (j, j+1, j+segts+1, j+segts), group)
		for k in range(1,segts-2):
			mkquad(mesh, (j+k, j+k+1, j+k+segts+1, j+k+segts), group)
		if b==c:	mktri(mesh, (j+segts-2, j+segts-1, j+segts+segts-2), group)
		else:		mkquad(mesh, (j+segts-2, j+segts-1, j+segts+segts-1, j+segts+segts-2), group)
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

def makeloops(lines, faces=()):
	''' return a list of the loops that can be formed with lines.
		lines must be oriented suite of points and loops are returned as suite of points.
	'''
	lines = list(lines)
	# get contiguous suite of points
	loops = []
	loop = list(lines.pop())
	while lines:
		found = False
		for i,edge in enumerate(lines):
			edge = lines[i]
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
		if not found:
			loops.append(loop)
			loop = list(lines.pop())
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
	x,y,z = makebase((mesh.points[i] for i in outline))
	pts = [vec2(dot(mesh.points[p],x),dot(mesh.points[p],y)) for p in outline]
	# remove possible flat faces
	i = 0
	while i < len(pts):
		surf = perpdot(pts[i]-pts[i-1], pts[i-2]-pts[i-1])
		if surf == 0:
			pts.pop(i-1)
			outline.pop(i-1)
		else:
			i += 1
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
	''' create a base that keep the points in the XY plane '''
	points = iter(points)
	x = y = o = next(points)
	for p in points:
		if p != o:
			x = p
			break
	for p in points:
		if abs(length(cross(normalize(p-o), normalize(x-o)))) > NUMPREC:
			y = p
			break
	vx = normalize(x-o)
	vz = normalize(cross(vx, y-o))
	vy = cross(vz,vx)
	return vx,vy,vz


def flatsurface(outline):
	''' create a surface in the given outline '''
	if isinstance(outline, Outline):
		outline = outline.points
	mesh = Mesh(outline, [], [], [])
	triangulate(mesh, list(range(len(outline))))
	return mesh

def tangentellipsis(axis1, axis2, axis3, resolution=None):
	axis = (axis1, axis2, axis3)
	base = [ -normalize(cross(axis[i-1][0] - axis[i-2][0], axis[i][1]))
				for i in range(3)]
	dims = inverse(mat3(base)) * (-2*axis[0][0] + axis[1][0] + axis[2][0])
	base[0] *= -2*dim[0]
	base[1] *= dim[1]
	base[2] *= dim[2]
	origin = axis[0][0] - base[0]
	pts = []
	faces = []
	for i in range(div+1):
		for j in range(i+1):
			u,v = i/(div+1), j/(i+1)
			pts.append((base[0]*(1-cos(u)) + base[1]*(1-sin(u))) * (1-cos(v)) + base[2]*(1-sin(v)) + origin)
	return Mesh(pts, faces)




if __name__ == '__main__':
	from nprint import nprint
	from math import pi
	from mathutils import Box
	import view
	from primitives import Arc, Segment
	import sys
	from PyQt5.QtWidgets import QApplication
	
	
	app = QApplication(sys.argv)
	main = scn3D = view.Scene()
	
	# test extrusion
	m1 = extrusion(vec3(0,0,0.5), outline(
			[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0), vec3(1,1,0)],
			[0,1,2,3]))
	
	# test revolution
	#m1 = extrusion(vec3(0,0,0.5), Outline(
		#[vec3(1,1,0), vec3(-1,1,0), vec3(-1,0.5,0), vec3(-1,-0.5,0), vec3(-1,-1,0), vec3(1,-1,0), vec3(1,-0.5,0),vec3(1,0.5,0), vec3(1,1,0)],
		#[0,1,2,3,4,5,6,7]))
	#m2 = revolution(pi, vec3(0,1,0), vec3(0,0,0), Outline(
		#[vec3(1,1,0), vec3(1,0.5,0), vec3(0.8,0,0), vec3(1,-0.5,0), vec3(1,-1,0)],
		#[0, 1, 1, 2]), 16)
	m2 = revolution(pi, (vec3(0,0,0), vec3(0,1,0)), outline([
			Segment(vec3(1,1,0), vec3(0.9,0.5,0)), 
			Arc(vec3(0.9,0.5,0), vec3(0.7,0,0), vec3(0.9,-0.5,0)), 
			Segment(vec3(0.9,-0.5,0), vec3(1,-1,0)),
			]))
	
	# test closeholes
	m = m1+m2
	m.mergedoubles()
	m.strippoints()
	#assert m.isvalid()
	#print(loopholes(m))
	#assert len(loopholes(m)) == 5, len(loopholes(m))
	#closeenvelope(m)
	#assert m.isenvelope()
	assert m.isvalid()
	
	m.transform(vec3(0,0,4))
	#m.options['debug_display'] = True
	#m.options['debug_points'] = True
	scn3D.add(m)
	
	
	# test tubes
	m3 = tube(
			outline([vec3(1,0,0), vec3(0,1,0), vec3(-1,0,0), vec3(0,-1,0), vec3(1,0,0)], [0,1,2,3]),
			#outline(Arc(vec3(0,0,0), vec3(4,1,4), vec3(6,0,3))),
			outline([vec3(0,0,0), vec3(0,0,2), vec3(1,0,3), vec3(4,0,3)]),
			)
	assert m3.isvalid()
	m3.transform(vec3(-4,0,0))
	
	#m3.options['debug_display'] = True
	#m3.options['debug_points'] = True
	scn3D.add(m3)
	#scn3D.look(m3.box())
	
	# test junction
	m4 = junction(
			outline([vec3(-2,0,0), vec3(-1,0,0), vec3(0,0,0), vec3(1,0,0), vec3(2,0,0)]),
			outline([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)]),
			#resolution=('div', 2),
			)
	#m4.options['debug_display'] = True
	#m4.options['debug_points'] = True
	scn3D.add(m4)
	
	main.show()
	sys.exit(app.exec())
