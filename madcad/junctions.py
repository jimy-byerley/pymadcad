# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module focuses on automated envelope generations, based on interface outlines
	the user has only to define the interface outlines or surfaces to join, and the algorithm makes the surface. No more pain to imagine some fancy geometries.
	
	formal definitions:
	
	interface   a surface or an outline (a loop) with associated exterior normals.
	node        a group of interfaces meant to be attached together by a blended surface.
	
	In order to generate envelopes, this module asks for cutting all the surfaces to join into 'nodes'. The algorithm decides how to join shortly all the outlines in a node. Once splited in nodes, you only need to generate node junctions for each, and concatenate the resulting meshes.
	
	details
	-------
	
	The blended surfaces are created between interfaces, linked as the points of a convex polyedron of the interfaces directions to the node center.

		
	example
	-------
	
	>>> x,y,z = vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)
	>>> m = junction(
			# can pass a surface: the surface outlines and normals will be used as the generated surface tangents 
			extrusion(2*z, web(Circle((vec3(0),z), 1))),	
			# can pass wire or primitives: the wire loops are used and the approximate normal to the  wire plane
			Circle((2*x,x), 0.5),	
			# more parameters when building directly the interfqce
			(Circle((2*y,y), 0.5), 'tangent', 2.),
			
			generate='normal',
			)
			
	# to come in a next version
	# create junction for each iterable of interface, if some are not interfaces, they are used as placeholder objects for auto-determined interfaces
	>> multijunction(
			(surf1, surf2, 42, *),
			(42, surf3, surf4),
			generate='straight',
			)
'''

from .mesh import Mesh, Wire, Web, wire, connef, edgekey, glmarray, suites, arrangeface
from .mathutils import *
from . import settings
from . import generation

from .nprint import nprint


def interfaces(objs, tangents='normal', weight=1.):
	''' collects the data definig interfaces for junctions and blendings out of an iterable of tuples 
		This function has no real interest for the enduser.
	'''
	# simply merge results of interface_parse
	v_pts = []
	v_tangents = []
	v_weights = []
	loops = []
	for obj in objs:
		# parse
		args = [None, tangents, weight]
		if isinstance(obj, tuple):
			for i,o in enumerate(obj):	args[i] = o
		else:
			args[0] = obj
		e_pts, e_tangents, e_weights, e_loops = interface(*args)
		# concatenate
		l = len(v_pts)
		v_pts.extend(e_pts)
		v_tangents.extend(e_tangents)
		v_weights.extend(e_weights)
		loops.extend([i+l  for i in loop] for loop in e_loops)
	return v_pts, v_tangents, v_weights, loops
		
def interface(base, tangents='normal', weight=1.):
	''' collects the data definig interfaces for junctions and blendings out of one object 
		This function has no real interest for the enduser.
	'''
	if not base:
		raise ValueError('interface is not fully defined, loops are missing')
	if not isinstance(base, (Wire, Web, Mesh)) and hasattr(base, 'mesh'):	
		base = base.mesh()
	if not isinstance(base, (Wire, Web, Mesh)):
		raise TypeError('expected one of Wire,Web,Mesh   and not {}'.format(type(base).__name__))
	
	# get the interface outline to connect to the others
	base.strippoints()
	points = base.points
	if isinstance(base, Wire):		loops = [base.indices]
	elif isinstance(base, Web):		loops = suites(base.edges)
	elif isinstance(base, Mesh): 
		loops = suites(base.outlines_oriented())
	
	# normal to each point
	# tangents provided explicitely
	if isinstance(tangents, (list,dict)):
		tangents = tangents
	# one tangent shared by all points
	elif isinstance(tangents, vec3):
		tangents = [tangents] * len(points)
		
	elif tangents == 'straight':
		tangents = [vec3(0)] * len(points)
	
	elif tangents == 'normal':
		# tangents normal to surface
		if isinstance(base,Mesh):
			tangents = base.vertexnormals()
		# tangents are in the common direction of adjacent faces
		else:
			tangents = [None] * len(points)
			for loop in loops:
				for i,n in zip(loop, Wire(points, loop).vertexnormals(True)):
					tangents[i] = n
	
	elif tangents == 'tangent':
		tangents = [None] * len(points)
		# tangents tangents to surface outline
		if isinstance(base,Mesh):
			for p,t in base.tangents().items():
				tangents[p] = t
		# tangents are tangents to a guessed surface in the loop
		else:
			for loop in loops:
				for i,n in zip(loop, Wire(points, loop).tangents(True)):
					tangents[i] = n
	else:
		raise ValueError('wrong tangents specification')
	
	# factor applied on each tangents
	weights = [weight] * len(points)
	return points, tangents, weights, loops



def junction(*args, center=None, tangents='normal', weight=1., match='length', resolution=None):
	''' join several outlines with a blended surface
		
		generate:	
			'straight'	no interpolation, straight lines
			'normal'	interpolated surface starting normal to the interfaces
			'tangent'	interpolated surface starting tangent to the interfaces
			
		match:
			'length'	share the outline between 2 matched points to assign the same length to both sides
			'corner'	split the curve to share around a corner
			
		.. note::
			match method 'corner' is not yet implemented
	'''
	pts, tangents, weights, loops = interfaces(args, tangents, weight)
	
	# determine center and convex hull of centers
	if not center:
		center = interfaces_center(pts, *loops)
	node = convexhull([	normalize(interfaces_center(pts, interf)-center) 
						for interf in loops])
	for i,f in enumerate(node.faces):
		if dot(node.facenormal(f), sum(node.facepoints(i))) < 0:
			node.faces[i] = (f[0],f[2],f[1])
	nodeconn = connef(node.faces)
	
	# determine the junction triangles and cuts
	middles = []
	cuts = {}
	for face in node.faces:
		n = node.facenormal(face)
		middle = [	max(loops[i], key=lambda p: dot(pts[p], n) )	
					for i in face]
		#middle = [None]*3
		#for i in range(3):
			#interface = loops[face[i]]
			#middle[i] = interface[max(range(len(interface)), 
				#key=lambda j: dot(pts[interface[i]], n) * dot(n, 
									#cross(	pts[interface[j]] - pts[interface[j-1]], 
											#node.points[face[i]] - node.points[face[i-1]])),
				#)]
		middles.append(middle)
		for i in range(3):
			if middle[i-1] in cuts and cuts[middle[i-1]] != middle[i-2]:	continue
			cuts[middle[i-1]] = middle[i]
	# cut the interfaces
	parts = {}
	for interf in loops:
		l = len(interf)
		last = None
		first = None
		for i,p in enumerate(interf):
			if p in cuts:
				if first is None:	first = i
				if last is not None:
					parts[interf[last]] = interf[last:i+1]
				last = i
		if (first-last) % len(interf) > 1:
			parts[interf[last]] = interf[last:] + interf[1:first+1]
	nprint('cuts', cuts)
	nprint('parts', parts)
	# assemble parts in matchings
	matchs = []
	done = set()
	for i in cuts:
		if i in done:	continue
		j = parts[cuts[i]][-1]
		assert parts[cuts[j]][-1] == i, "failed to match outlines"
		matchs.append((parts[cuts[i]], parts[cuts[j]]))
		done.add(j)
	
	# generate the surfaces
	result = Mesh(pts)
	div = 11
	for la,lb in matchs:
		def infos():
			for a,b in curvematch(Wire(pts,lb), Wire(pts,list(reversed(la)))):
				# normalize except for 0
				ta, tb = tangents[a], tangents[b]
				ta /= length(ta) or 1
				tb /= length(tb) or 1
				# scale and weight
				l = distance(pts[a],pts[b])  # NOTE not the arclength for the moment
				yield (pts[a], ta*l*weights[a]), (pts[b], tb*l*weights[b])
		result += blenditer(infos(), div, interpol2)
	for tri in middles:
		assert len(tri) == 3
		ptgts = [None]*3
		ptri = [None]*3
		for i in range(3):
			s = tri[i]
			ptgts[i] = (tangents[s]*weights[s]*distance(pts[s], pts[tri[i-2]]), 
						tangents[s]*weights[s]*distance(pts[s], pts[tri[i-1]]),
						)
			ptri[i] = pts[tri[i]]
		#etangents = [2*pts[tri[i]]-pts[tri[i-1]]-pts[tri[i-2]]	for i in range(3)]
		#result += blendtri([pts[i] for i in tri], ptgts, div)
		#result += closingtri([pts[i] for i in tri], ptgts, div)
		result += generation.dividedtriangle(lambda u,v: intri_smooth(ptri, ptgts, u,v), div)
	
	#result.tracks = [0]*len(result.faces)
	#result.groups = ['junction']
	result.mergeclose()
	return result
	


def multijunction(*nodes, **kwargs):
	''' resolves partial definitions of interfaces, then create junctions with it 
	
		nodes are lists of interfaces, each of these list will end in a call to `junction`
		nodes can contain 
		- interfaces (or objects than can be casted into)
		- couples of the following format: (key, mix)
			key - hashable objects used to implicitely define an interface
			mix - float used to set the interface center
	'''
	indev

	
def interfaces_center(pts, *loops):
	center = vec3(0)
	total = 0
	for loop in loops:
		for i in range(len(loop)):
			l = length(pts[loop[i-1]]+pts[loop[i]])
			center += (pts[loop[i-1]] + pts[loop[i]]) * l/2
			total += l
	return center / total if total else center

def convexhull(pts):
	import scipy.spatial
	if len(pts) == 3:
		return Mesh(pts, [(0,1,2),(0,2,1)])
	else:
		hull = scipy.spatial.ConvexHull(glmarray(pts))
		return Mesh(pts, [tuple(v) for v in hull.simplices])
	

def trijoin(pts, ptgts, div):
	mesh = Mesh(groups=['blend'])
	segts = div+1
	for i in range(3):
		for j in range(segts):
			x = j/segts
			mesh.points.append(interpol2((pts[i-1], ptgts[i-1][0]), (pts[i], ptgts[i][1]), x))
	l = len(mesh.points)
	for i in range(3):
		c = i*segts
		for j in range(segts//2):
			mkquad(mesh, ((c-j-1)%l, (c-j)%l, (c+j)%l, (c+j+1)%l))
	return mesh

def curvematch(line1, line2) -> '[(int,int)]':
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
	''' simple straight surface created from matching couples of line1 and line2 using mesh indices for lines '''
	group = len(mesh.groups)
	mesh.groups.append('blend')
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
			
def matchexisting(line1, line2) -> '[(vec3,vec3)]':
	''' create couple of points using curvematch '''
	return  ( (line1.points[i1], line2.points[i2]) 
				for i1,i2 in curvematch(line1, line2))

def matchclosest(line1, line2) -> '[(vec3, vec3)]':
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

def dividematch(match, resolution=None) -> '[(vec3, vec3)]':
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


def blend(*interfaces, center=None, generate='straight', match='length', resolution=None) -> 'Mesh':
	''' create a direct blended surface between unclosed lines
	'''	
	match = list(match)
	# get the discretisation
	div = 0
	for i in range(1, len(match)):
		a,d = match[i-1]
		b,c = match[i]
		v = normalize((a+b) - (d+c))
		angle = anglebt(a-b - project(a-b,v), d-c - project(d-c,v))
		dist = min(distance(a,b), distance(d,c))
		div = max(div, settings.curve_resolution(dist, angle, resolution))
	
	return blenditer(match, div, interpol1)

def blendpair(i0, i1, div, generate='straight', match='length'):
	''' blend between a pair of interfaces 
		
		match   'length', 'closest'
	'''
	if len(i0).loops != 1 or len(i1.loops) != 1:
		raise ValueError('interfaces must have one loop only')
	
	if generate == 'straight':
		infos = zip(i0.loop, reversed(i1.loop))
		return blenditer(infos, div, interpol1)
	elif generate == 'tangent':
		def infos():
			for p0,p1 in zip(i0.loop, reversed(i1.loop)):
				yield (i0.pts[p0], i0.tangents[p0]), (i1.pts[p1], i1.tangents[p1])
		return blenditer(infos(), div, interpol2)

def blenditer(parameters, div, interpol) -> Mesh:
	''' create a blended surface using the matching parameters and the given interpolation 
		parameters is an iterable of tuples of arguments for the interpolation function
		interpol receive the elements iterated and the interpolation position at the end
	'''
	segts = div+2
	mesh = Mesh(groups=['blend'])
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
			mkquad(mesh, (s, s+1, s+segts+1, s+segts))
	mesh.mergeclose()
	return mesh
		
		
def mktri(mesh, pts, track=0):
	mesh.faces.append(pts)
	mesh.tracks.append(track)

def mkquad(mesh, pts, track=0):
	if (	distance(mesh.points[pts[0]], mesh.points[pts[2]]) 
		<=	distance(mesh.points[pts[1]], mesh.points[pts[3]]) ):
		mesh.faces.append((pts[:-1]))
		mesh.faces.append((pts[3], pts[0], pts[2]))
	else:
		mesh.faces.append((pts[0], pts[1], pts[3]))
		mesh.faces.append((pts[2], pts[3], pts[1]))
	mesh.tracks.append(track)
	mesh.tracks.append(track)


def stretch(mesh, curve=0.1, locked=()):
	''' stretching smoothing of the surface until reaching a certain curve (rad/m)
		locked can be a set of points that won't be moved
		
		We advise having a well subdivided surface (use `subdivide` before if necessary)
	'''
	conn = connef(mesh.faces)
	edges = set(edgekey(*e)  for e in conn)
	pts = mesh.points
	def current_curve(a,b):
		c = arrangeface(mesh.faces[conn[(a,b)]], a)[2]
		d = arrangeface(mesh.faces[conn[(b,a)]], b)[2]
		z = normalise(b-a)
		cy = noproject(c-a, z)
		dy = noproject(d-a, z)
		cyl = length(cy)
		dyl = length(dy)
		return cross(dy, dy) / (cyl*dyl) / (cyl+dyl)
	
	curve = {e: current_curve(*e)	for e in edges}	# curve around each edge
	indev

