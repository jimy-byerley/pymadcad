# This file is part of pymadcad,  distributed under license LGPL v3
'''
This module focuses on automated envelope generations, based on interface outlines.

the user has only to define the interface outlines or surfaces to join, and the algorithm makes the surface. No more pain to imagine some fancy geometries.
	
formal definitions
------------------
	
:interface:   a surface or an outline (a loop) with associated exterior normals.
:node:        a group of interfaces meant to be attached together by a blended surface.

In order to generate envelopes, this module asks for cutting all the surfaces to join into 'nodes'. The algorithm decides how to join shortly all the outlines in a node. Once splited in nodes, you only need to generate node junctions for each, and concatenate the resulting meshes.

details
-------

The blended surfaces are created between interfaces, linked as the points of a convex polyedron of the interfaces directions to the node center.

	
example
-------

.. code::

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

to come in a next version

.. code::
	
	# create junction for each iterable of interface, if some are not interfaces, they are used as placeholder objects for auto-determined interfaces
	>> multijunction(
			(surf1, surf2, 42, surf5),
			(42, surf3, surf4),
			generate='straight',
			)
'''

from .mesh import Mesh, Wire, Web, wire, connef, edgekey, suites, arrangeface, mkquad, numpy_to_typedlist, typedlist_to_numpy
from .mathutils import *
from . import settings
from . import generation

from .nprint import nprint


def get_interfaces(objs, tangents, weights):
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
		args = [None, tangents, weights, None]
		if isinstance(obj, tuple):
			for i,o in enumerate(obj):	args[i] = o
		else:
			args[0] = obj
		e_pts, e_tangents, e_weights, e_loops = get_interface(*args)
		# concatenate
		l = len(v_pts)
		v_pts.extend(e_pts)
		v_tangents.extend(e_tangents)
		v_weights.extend(e_weights)
		loops.extend([i+l  for i in loop] for loop in e_loops)
	return v_pts, v_tangents, v_weights, loops
		
def get_interface(base, tangents, weights, loops):
	''' collects the data definig interfaces for junctions and blendings out of one object 
		This function has no real interest for the enduser.
	'''
	# get the interface outline to connect to the others
	if loops is None:
		if not isinstance(base, (Wire, Web, Mesh)) and hasattr(base, 'mesh'):	
			base = base.mesh()
		if not isinstance(base, (Wire, Web, Mesh)):
			raise TypeError('expected one of Wire,Web,Mesh   and not {}'.format(type(base).__name__))
		
		base.strippoints()
		points = base.points
		if isinstance(base, Wire):		loops = [base.indices]
		elif isinstance(base, Web):		loops = suites(base.edges)
		elif isinstance(base, Mesh): 
			loops = suites(base.outlines_oriented())
	else:
		if not isinstance(base, list):
			raise TypeError('if loops are provided, points must be a list')
		points = base
	
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
	if not isinstance(weights, list):
		weights = [weights] * len(points)
	return points, tangents, weights, loops



def junction(*args, center=None, tangents='normal', weight=1., match='length', resolution=None):
	''' join several outlines with a blended surface
		
		tangents:	
			'straight'	no interpolation, straight lines
			'normal'	interpolated surface starts normal to the interfaces
			'tangent'	interpolated surface starts tangent to the interfaces
		
		weight:
			factor applied on the tangents before passing to `interpol2` or `intri_smooth`
			the resulting tangent is computed in point `a` as `weight * distance(a,b) * normalize(tangent[a])`
			
		match:
			'length'	share the outline between 2 matched points to assign the same length to both sides
			'corner'	split the curve to share around a corner
			
		center:		
			position of the center of the junction node used to determine connexion between interfaces
			can be usefull for particularly weird and ambiguous interfaces
			
		.. note::
			match method 'corner' is not yet implemented
	'''
	pts, tangents, weights, loops = get_interfaces(args, tangents, weight)
	if len(loops) == 0:
		return Mesh()
	if len(loops) == 1:	
		return blendloop(
					(pts, tangents, weights, loops), 
					center, resolution=resolution)
	if len(loops) == 2:
		c0 = interfaces_center(pts, loops[0])
		c1 = interfaces_center(pts, loops[1])
		z = c1 - c0
		p = pts[imax( length2(noproject(pts[i]-c0, z))   for i in loops[0] )]
		x = noproject(p - c0, z)
		i0 = imax( dot(pts[i], x)  for i in loops[0])
		i1 = imax( dot(pts[i], x)  for i in loops[1])
		return blendpair(
					(pts, tangents, weights, (
						loops[0][i0:]+loops[0][1:i0+1],
						loops[1][i1:]+loops[1][1:i1+1],
						)),
					resolution=resolution)
	
	# determine center and convex hull of centers
	if not center:
		center = interfaces_center(pts, *loops)
	node = convexhull([	normalize(interfaces_center(pts, interf)-center) 
						for interf in loops])
	# fix convexhull faces orientations
	for i,f in enumerate(node.faces):
		if dot(node.facenormal(f), sum(node.facepoints(i))) < -1e-9:
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
			for a,b in match_length(Wire(pts,lb), Wire(pts,list(reversed(la)))):
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
			
		result += generation.dividedtriangle(lambda u,v: intri_smooth(ptri, ptgts, u,v), div)
	
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
		return Mesh(pts, numpy_to_typedlist(
			scipy.spatial.ConvexHull(
				typedlist_to_numpy(pts, 'f8'), 
				qhull_options='QJ Pp',
				).simplices, uvec3))
	


def match_length(line1, line2) -> '[(int, int)]':
	''' yield couples of point indices where the curved absciss are the closest '''
	yield line1.indices[0], line2.indices[0]
	l1, l2 = line1.length(), line2.length()
	i1, i2 = 1, 1
	x1, x2 = 0, 0
	while i1 < len(line1.indices) and i2 < len(line2.indices):
		p1 = distance(line1.points[line1.indices[i1-1]], line1.points[line1.indices[i1]]) / l1
		p2 = distance(line2.points[line2.indices[i2-1]], line2.points[line2.indices[i2]]) / l2
		x1p = x1+0.5*p1
		x2p = x2+0.5*p2
		if x1 <= x2p and x2p <= x1+p1    and    x2 <= x1p and x1p <= x2+p2:
			i1 += 1; x1 += p1
			i2 += 1; x2 += p2
		elif x1p < x2p:	
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


def match_closest(line1, line2) -> '[(int, int)]':
	''' yield couples of points by cutting each line at the curvilign absciss of the points of the other '''
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

def dividematch_length(line1, line2, resolution=None, associated=None) -> '[(vec3, vec3)]':
	''' insert additional couples to ensure smoothness '''
	indev

def dividematch_closest(w0:Wire, w1:Wire, resolution=None, associated=None) -> (Wire, Wire):
	indev

def blend(interfaces, generate='straight', match='length', resolution=None) -> 'Mesh':
	''' create a direct blended surface between unclosed lines '''	
	pts, tangents, weights, loops = get_interfaces(args, tangents, weight)
	
	indev
	
def blendloop(interface, center=None, tangents='tangent', weight=1., resolution=None) -> Mesh:
	''' blend inside a loop interface 
	 
		see `junction` for the parameters.
	'''
	pts, tangents, weights, loops = get_interfaces([interface], tangents, weight)
	if len(loops) > 1:	
		raise ValueError('interface must have one loop only')
	loop = loops[0]
	
	# get center and normal at center
	if not center:
		center = interfaces_center(pts, *loops) + sum(weights[p]*tangents[p] for p in loop) / len(loop)
	normal = normalize(sum(normalize(center-pts[p])	for p in loop))
	if not isfinite(normal):	normal = vec3(0)
	
	# generatestraight
	match = [None] * len(loop)
	div = 0
	for i,p in enumerate(loop):
		match[i] = m = (	(pts[p], distance(center, pts[p])*weights[p]*tangents[p]), 
							(center, noproject(pts[p]-center, normal))	)
		div = max(div, settings.curve_resolution(distance(m[0][0],m[1][0]), anglebt(m[0][1],m[1][1]), resolution))
	return blenditer(match, div, interpol2)


def blendpair(*interfaces, match='length', tangents='tangent', weight=1., resolution=None) -> Mesh:
	''' blend between a pair of interfaces 
		
		match:   
			'length', 'closest'
			refer to `match_*` in this module 
	 
		see `junction` for the other parameters.
	'''
	pts, tangents, weights, loops = get_interfaces(interfaces, tangents, weight)
	if len(loops) != 2:	
		raise ValueError('interface must have exactly 2 loops, got '+str(len(loops)))
	
	if match == 'length':		method = match_length
	elif match == 'closest':	method = match_closest
	else:
		raise ValueError('matching method {} not implemented'.format(match))
	
	#if loops[1][0] == loops[1][-1]:		loops[1] = synchronize(Wire(pts, loops[0]), Wire(pts, loops[1])).indices
	#elif loops[0][0] == loops[0][-1]:	loops[0] = synchronize(Wire(pts, loops[1]), Wire(pts, loops[0])).indices
	
	match = list(method(Wire(pts, loops[0]), Wire(pts, list(reversed(loops[1])))))
	# get the discretisation
	div = 0
	for i in range(1, len(match)):
		a,d = match[i-1][0], match[i-1][1]
		b,c = match[i][0], match[i][1]
		m = (pts[a] + pts[b] + pts[c] + pts[d]) /4
		ta = pts[a] -m
		tb = pts[b] -m
		tc = pts[c] -m
		td = pts[d] -m
		div = max(
				div,
				settings.curve_resolution(
						distance(pts[a],pts[c]), 
						anglebt(ta, -tc) + anglebt(ta, tangents[a]) + anglebt(tc, tangents[c]), 
						resolution),
				settings.curve_resolution(
						distance(pts[b],pts[d]), 
						anglebt(tb, -td) + anglebt(tb, tangents[b]) + anglebt(td, tangents[d]), 
						resolution),
				)
	def infos():
		for p0, p1 in match:
			d = distance(pts[p0], pts[p1])
			yield ((pts[p0], d*weights[p0]*tangents[p0]), (pts[p1], d*weights[p1]*tangents[p1]))
	return blenditer(infos(), div, interpol2)

def blenditer(parameters, div, interpol) -> Mesh:
	''' create a blended surface using the matching parameters and the given interpolation 
		parameters is an iterable of tuples of arguments for the interpolation function
		interpol receive the elements iterated and the interpolation position at the end
	'''
	segts = div+2
	mesh = Mesh(groups=[None])
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


def synchronize(ref:Wire, loop:Wire) -> Wire:
	''' assuming loop is a closed Wire, it will change its start point to match the startpoint of ref 
	'''
	r0, r1 = 0, int(wire_atlength(ref, ref.length()/2))	# lookup points on ref
	l0 = l1 = 0	# lookup points on loop: start and middle
	s0 = s1 = 0	# offset distance of lookup points on loop
	half = wire_atlength(loop, loop.length()/2)
	l1 = int(half)
	s1 = distance(loop[l1-1], loop[l1]) * (half - l1)
	
	# take the l0,l1 couple that minimize the squared distance between matching lookup points
	best = distance2(ref[r0], loop[l0]) + distance2(ref[r1], loop[l1])
	candidate = l0
	for l0 in range(1,len(loop)):
		# increment l0
		incr = distance(loop[l0-1], loop[l0])
		s0 += incr
		# increment l1
		while True:
			incr = distance(loop[l1], loop[(l1+1)%len(loop)])
			if abs(s1-s0+incr) > abs(s1-s0):
				break
			s1 += incr
			l1 = (l1+1) % len(loop)
		# search for a minimum
		score = distance2(loop[l0], ref[r0]) + distance2(loop[l1], ref[r1])
		if score < best:
			candidate = l0
			best = score
	l0 = candidate
	
	# circulate the loop
	end = loop.indices[l0:]
	if loop[0] == loop[-1]:		end.pop()
	return Wire(loop.points, end + loop.indices[:l0+1])

def wire_atlength(wire, length):
	''' return the index of the wire point at the given length.
	
		the returned value is float whose integer part is the index and floating part is the 
		ratio of the next segment to use to reach the exact desired length.
	''' 
	s = 0
	for i in range(1,len(wire)):
		d = distance(wire[i-1], wire[i])
		s += d
		if s > length:
			return i - (length-s)/d
	
def join(mesh, line1, line2):
	''' simple straight surface created from matching couples of line1 and line2 using mesh indices for lines '''
	group = len(mesh.groups)
	mesh.groups.append(None)
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
		
def trijoin(pts, ptgts, div):
	''' simple straight surface created between 3 points, interpolation is only on the sides '''
	mesh = Mesh(groups=[None])
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
