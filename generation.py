from mesh import Mesh, edgekey, MeshError
from mathutils import vec2,mat2,vec3,vec4,mat3,mat4, rotate, translate, dot, cross, perpdot, project, normalize, inverse, length, NUMPREC, COMPREC
from math import atan2
from primitives import Primitive
import settings

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
	radius = 0
	for pt in outline.points:
		v = pt-axis[0]
		v -= project(v,axis[1])
		radius = max(radius, length(v))
	trans = lambda x: translate(rotate(translate(mat4(1), -axis[0]), x*angle, axis[1]), axis[0])
	steps = settings.curve_resolution(angle*radius, angle, resolution)
	return extrans(trans, steps, outline)

def saddle(outline1, outline2):
	#print(outline1.points)
	#for i,p in enumerate(outline2.points):
		#for j,p2 in enumerate(outline2.points):
			#if j!=i and p == p2:	print('!!! same as',j)
		#print(i,p)
	l = len(outline2.points)-1
	def trans(x):
		i = int(x*l+0.5)
		return translate(mat4(1), outline2.points[i]-outline2.points[0])
	return extrans(trans, l, outline1)

def tube(outline1, outline2, end=True, section=True):
	''' create a tube surface by extrusing the outline 1 along the outline 2
		if section is True, there is a correction of the segments to keep the section undeformed by the curve
	'''
	lastrot = quat()
	hrot = quat()
	l = len(outline2.points)-1
	offset = 0 if end else 1
	def trans(x):
		i = int(x*(l-2*offset)+0.5) + offset
		if i > 0 and i < l:
			# calculer l'angle a ce coude
			o = outline.points[i]
			c = cross(o-outline.points[i-1], outline2.points[i+1]-o)
			cl = length(c)
			ha = asin(cl)/2
			cn = c/cl
			hrot = quat(ha, cn)
			# calcul de la rotation de la section
			rot *= hrot * lasthrot
			lasthrot = hrot*rot
			trans = mat4(1)
			m = mat3_cast(rot)
			# deformation de la section pour ne pas réduire le volume
			if section:	
				m = scaledir(cn, 1/cos(ha)) * m
		else:
			m = mat3_cast(rot)
		# construction de la transformation
		center = outline.points[i]-outline2.points[0]
		t = translate(t, -center)
		t = rotate(t, m)
		t = translate(t, center)
		return trans
	
	if not end:		trans(0)
	return extrans(trans, l, outline1)


def extrans(transformer, steps, outline):
	''' create a surface by extruding and transforming the given outline.
		transformer gives the transform at each step    
			transformer(x:[0:1]) -> mat4
		`steps` is the number of extrusion steps
	'''
	mesh = Mesh([], [], [], outline.groups)
	for i in range(steps+1):
		mat = transformer(i/steps)
		for p in outline.points:
			mesh.points.append(vec3(mat*vec4(p,1)))
	l = len(outline.points)
	for j in range(0,steps*l,l):
		for i,t in zip(range(l-1), outline.tracks):
			mesh.faces.append((j+i, j+i+1, j+i+1+l))
			mesh.faces.append((j+i, j+i+1+l, j+i+l))
			mesh.tracks.append(t)
			mesh.tracks.append(t)
	return mesh
	

def junction(line1, line2, steps):
	pass

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
		best_surf = 0.
		index = None
		l = len(pts)
		for i in range(l):
			o = pts[i]
			u = pts[(i+1)%l] - o
			v = pts[(i-1)%l] - o
			surf = perpdot(u,v)
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
	
	m1 = extrusion(vec3(0,0,0.5), Outline(
		[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0), vec3(1,1,0)],
		[0,1,2,3]))
		
	print(m1)
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
	
	m = m1+m2
	m.mergedoubles()
	m.strippoints()
	assert len(loopholes(m)) == 5
	closeenvelope(m)
	assert m.isenvelope()
	assert m.isvalid()
	
	m.transform(vec3(0,0,5))
	
	#m.options['debug_display'] = True
	#m.options['debug_points'] = 'tracks'
	scn3D.objs.append(m)
	scn3D.look(m.box())
	
	main.show()
	sys.exit(app.exec())
