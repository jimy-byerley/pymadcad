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
	
	
	

class Interpolated2(object):
	''' interpolated curve passing through the given points (3rd degree bezier spline) 

		the tangent in each point is determined by the direction between adjacent points
		the point weights is how flattened is the curve close to the point tangents
	'''
	__slots__ = 'points', 'weights', 'resolution'
	def __init__(self, points, weights=None, resolution=None):
		self.points = points
		self.weights = weights or [1] * len(self.points)
		self.resolution = resolution
		
	def mesh(self, resolution=None):
		pts = self.points
		if not pts:		return Wire()
		
		# get tangent to each point
		tas = [self.weights[i-1] * 0.5 * (pts[i]-pts[i-2])
					for i in range(2,len(pts))]
		tbs = [self.weights[i-1] * 0.5 * (pts[i-2]-pts[i])	
					for i in range(2,len(pts))]
		tas.insert(0, tbs[0] - 2*project(tbs[0], pts[1]-pts[0]))
		tbs.append(tas[-1] - 2*project(tas[-1], pts[-2]-pts[-1]))
		
		# stack points to curve
		curve = []
		for i in range(len(pts)-1):
			a,b = pts[i], pts[i+1]
			#ta,tb = tas[i], tbs[i]
			ta = pts[i+1]-pts[max(0,i-1)]
			tb = pts[i]-pts[min(len(pts)-1, i+2)]
			# tangent to the curve in its inflexion point
			mid = 0.75*(b-a) + 0.25*(ta-tb)
			# get resolution
			div = 1 + 2*max(settings.curve_resolution(
							length(ta), 
							anglebt(ta, mid),
							self.resolution or resolution),
						settings.curve_resolution(
							length(tb), 
							anglebt(tb, -mid),
							self.resolution or resolution) )
			
			# append the points for this segment
			for i in range(div+1):
				curve.append(interpol2((a,ta), (b,tb), i/(div+1)))
		
		curve.append(b)
		return mesh.Wire(curve, groups=['spline'])
		
	def box(self):
		return boundingbox(self.points)
		
	def __repr__(self):
		return 'Interpolated({}, {})'.format(self.points, self.weights)

	def display(self, scene):
		return displays.SplineDisplay(scene, glmarray(self.points), glmarray(self.mesh().points))
		
from .generation import subdivide
		
class Wire2(object):	
	__slots__ = 'points'
	def __init__(self, points=()):
		self.points = points
		
	def mesh(self, resolution=None):
		if not resolution: 
			resolution = settings.primitives['curve_resolution']
		if resolution[0] == 'div':
			return subdivide(wire(self.points), resolution[1])
		else:
			return wire(self.points)
	
	
def mapcurve(wire: Wire, a: vec3, b: vec3, rotate=True, scale=True, match='length') -> Wire:
	''' remap the given wire from its original ending points to the new points `a` and `b`
		
		- The original wire extremities provide the *original line*
		- The given points `a` and `b` provide the *new line*
		
		This functions works on the difference of each point to its match on the original line.
		
		Parameters:
			rotate:   if True, the differences will be rotated by the angle between the original line and the new one
			scale:    if True, the difference will be scaled by the length ratio between the original line and the new one
			match:    method to match the wire onto the line between its extremities
	'''
	absciss = globals()['absciss_'+match](wire)
	c, d = wire[0], wire[-1]
	
	if rotate and scale:
		transform = quat(d-c, b-a)
	elif rotate:
		transform = quat(normalize(d-c), normalize(b-a))
	elif scale:
		transform = length(b-a) / (length(d-c) or nan)
	else:
		transform = 1
	
	return Wire(
				points=( mix(a, b, x) + transform * (p - mix(c, d, x))
						for p,x in zip(wire, absciss)),
				tracks=wire.tracks,
				groups=wire.groups,
				)

	
def absciss_length(wire) -> '[float]':
	''' create an influence index depending on the curvilign absciss '''
	total = wire.length()
	absciss = [0] * len(wire)
	path = 0
	for i in range(1,len(wire)):
		path += distance(wire[i], wire[i-1])
		absciss[i] = path/total
	return absciss
	
def absciss_distance(wire) -> '[float]':
	''' create an influence index depending on the distance to the line between the wire extremities '''
	origin, direction = wire[0], wire[-1]-wire[0]
	return [ dot(p-origin, direction) / dot(direction,direction)   for p in wire ]

def absciss_index(wire) -> '[float]':
	''' create an influence index depending on the index in the wire '''
	return [ i/(len(wire)-1)   for i in range(len(wire)) ]


from .primitives import Interpolated
from . import mesh
from . import settings

	
def swipe(wires: list, primitive:type=Wire2, interlace=0.4, resolution=None) -> Mesh:
	''' create a blended surface between the given curves.
	
		This function uses `match_length()` to build successive tuples of points each for building a primitive.
		The result is a blended surface whose:
		- regularity accross wires depend on the specified primitive
		- regularity accross matched tuples depend on the specified primitive and input wire regularities
		
		The resulting surface can therefore be derivable in the 'across wire' direction, but not in the wire proper direction.
		
		Parameters:
		
			wires:	            a list of wires accross which the surface will be blended. Those wires will bo sort of iso lines in the resulting surface.
			primitive (type):   the `Primitive` type used to blend the surface. It must accept a list of points as unique argument.
			interlace (float):  interlacing ratio threashold as definied in `deinterlace()`
			resolution:         surface resolution. this parameter is passed to each primitive
	'''
	
	# propagate interlacement forward
	result = Mesh()
	interlaced = list(wires)
	for i in range(1, len(interlaced)):
		interlaced[i] = deinterlace(
					wires[i], 
					match_length(wire(interlaced[i-1]), wire(wires[i])) [1],
					interlace,
					len(interlaced[i-1]),
					)
	# propagate interlacement backward			
	for i in reversed(range(1, len(interlaced))):
		interlaced[i-1] = deinterlace(
					wires[i-1], 
					match_length(wire(interlaced[i]), wire(wires[i-1])) [1],
					1,
					len(interlaced[i]),
					)
		
	remove_doubles_zip(interlaced)
	
	# compute subdivisions where the surface torsion is too important
	steps = len(interlaced[0])
	div_across = [0] * (steps-1)
	div_along = [0] * (steps-1)
	for i in range(1, len(interlaced)):
		for j in range(1, steps):
			acrossv = mix(
					interlaced[i][j] - interlaced[i-1][j], 
					interlaced[i][j-1] - interlaced[i-1][j-1],
					0.5)
			across = settings.curve_resolution(
				length(acrossv),
				anglebt(
					normalize(noproject(interlaced[i-1][j] - interlaced[i-1][j-1], acrossv)),
					normalize(noproject(interlaced[i][j] - interlaced[i][j-1], acrossv)),
					),
				resolution,
				)
			alongv = mix(
					interlaced[i-1][j] - interlaced[i-1][j-1], 
					interlaced[i][j] - interlaced[i][j-1],
					0.5)
			along = settings.curve_resolution(
				length(alongv),
				anglebt(
					normalize(noproject(interlaced[i][j] - interlaced[i-1][j], alongv)), 
					normalize(noproject(interlaced[i][j-1] - interlaced[i-1][j-1], alongv)),
					),
				resolution,
				)
			div_across[j-1] = max(div_across[j-1], across)
			div_along[j-1] = max(div_along[j-1], along)
	print('div_across', div_across)
	print('div_along', div_along)
	
	# subdivide the input curves
	for i,div in enumerate(div_along):
		for curve in interlaced:
			curve[i+1:i+1] = [
				mix(curve[i], curve[i+1],  j/(div+1))
				for j in range(1,div)]
			div_across[i+1:i+1] = [div_across[i]] * div
	
	# generate profiles and join them
	steps = len(interlaced[0])
	
	#resolution = ('div',3)
	def generate(i): 
		prim = primitive([ curve[i]  for curve in interlaced])
		profile = wire(prim .mesh(resolution=resolution))
		requested = max(div_across[max(0,i-1)], div_across[min(steps-2,i)])
		if len(profile) < (requested+1)*len(interlaced):
			profile = wire(prim .mesh(resolution=('div',div_across[i-1])))
		return profile
	
	last = generate(0)
	for i in range(1,steps):
		new = generate(i)
		result += linkpair(last, new, match='length')
		last = new
	
	result.mergegroups()
	return result .finish()
	
def linkpair(line1, line2, match='length'):
	''' create triangles between the two curves, with no interpolation nor smoothing '''
	result = Mesh(groups=[None])
	_matched = globals()['match_'+match](line1, line2)
	matched = (
		deinterlace(line1, _matched[0], 1, len(line1), merge=False),
		deinterlace(line2, _matched[1], 1, len(line2), merge=False),
		)
	assert len(matched[0]) == len(matched[1])
	#remove_doubles_zip(matched)
	#assert len(matched[0]) == len(matched[1])
	
	prec = 1e-10
	
	#if not distance(matched[1][-1], line2[-1]) <= prec:
		#from .rendering import show
		#show([line1, line2, matched[0], matched[1]], options={'display_points':True})
		#show([line1, line2, _matched[0], _matched[1]], options={'display_points':True})
	
	#print(len(line1), len(line2))
	
	#assert distance(_matched[0][0], line1[0]) <= prec
	#assert distance(_matched[0][-1], line1[-1]) <= prec
	#assert distance(_matched[1][0], line2[0]) <= prec
	#assert distance(_matched[1][-1], line2[-1]) <= prec
	
	#assert distance(matched[0][0], line1[0]) <= prec
	#assert distance(matched[0][-1], line1[-1]) <= prec
	#assert distance(matched[1][0], line2[0]) <= prec
	#assert distance(matched[1][-1], line2[-1]) <= prec
	
	result.points.append(matched[0][0])
	result.points.append(matched[1][0])
	a, b = 0, 1
	
	for i in range(1, len(matched[0])):
		l = len(result.points)
		if matched[0][i] == matched[0][i-1] and matched[1][i] == matched[1][i-1]:
			pass
		if matched[0][i] == matched[0][i-1]:
			result.points.append(matched[1][i])
			mktri(result, (a, b, l))
			a, b = a, l
		elif matched[1][i] == matched[1][i-1]:
			result.points.append(matched[0][i])
			mktri(result, (a, b, l))
			a, b = l, b
		else:
			result.points.append(matched[0][i])
			result.points.append(matched[1][i])
			mkquad(result, (a, b, l+1, l))
			a, b = l, l+1
	return result
	
def deinterlace(ref, matched, interlace=1, samples=None, preserve=True, merge=True):
	''' remove intermediate points when they are closer than the interlace fraction of the smalles edge between i and i-1 
	
		Returns a new list
	
		Parameters:
		
			ref:         original wire
			matched:     the wire after matching, hence suvdivided at random places
			interlace:   the minimum interlacement proportion allowed (the maximum value is 0.5)
			samples:     the minimum number of points to keep, overriding the interlacing ratio
			preserve:    if True, will prefer to keep the original points of `ref` instead of an intermediate and closest point to the matched curve
			merge:       if False, deinterlace will keep the orignal amount of points, juste replacing those to merge by the value of the merge target
	'''
	print('deinterlace')
	prec = ref.precision()
	
	# collect segments priority due to interlacement
	priority = [1] * (len(matched)-1)
	originals = [False] * len(matched)
	originals[0] = originals[-1] = True
	j = 0
	for i in range(1,len(ref)):
		if distance2(ref[i], ref[i-1]) <= prec:	continue
		while True:
			j += 1
			interlacement = distance(matched[j], matched[j-1]) / distance(ref[i], ref[i-1])
			priority[j-1] = interlacement
			
			if distance2(ref[i], matched[j]) <= prec:
				break
		originals[j] = True
			
	# select highest priority separations to keep
	keep = samples-1 if samples else 0
	mendatory = set(sorted(range(len(priority)), key=priority.__getitem__, reverse=True)[:keep])
	print('  priority', priority)
	print('  originals', originals)
	print('  mendatory', mendatory)
	
	# merge from a fresh start
	altered = []
	original = originals[0]
	batch = matched[0]
	weight = 1
	merged = 1
	dist = 0
	for i in range(len(priority)):
		dist += priority[i]
		# case where link must not be merged
		if dist > interlace or i in mendatory:	
			for j in range(merged):
				altered.append(batch / weight)
			original = originals[i+1] or not preserve
			batch = matched[i+1]
			weight = 1
			merged = 1
			dist = 0
		else:
			if not merge:	merged += 1
			# merge in case of same priority
			if original == originals[i+1]:
				batch += matched[i+1]
				weight += 1
			# merge in case of higher priority
			elif originals[i+1]:
				original = True
				batch = matched[i+1]
				weight = 1
	for j in range(merged):
		altered.append(batch / weight)
	#altered[0] = matched[0]
	#altered[-1] = matched[-1]
		
	print('  altered', len(altered))
	
	return altered
	
def deinterlace(ref, matched, interlace=1, samples=None, preserve=True, merge=True):
	''' remove intermediate points when they are closer than the interlace fraction of the smalles edge between i and i-1 
	
		Returns a new list
	
		Parameters:
		
			ref:         original wire
			matched:     the wire after matching, hence suvdivided at random places
			interlace:   the minimum interlacement proportion allowed (the maximum value is 0.5)
			samples:     the minimum number of points to keep, overriding the interlacing ratio
			preserve:    if True, will prefer to keep the original points of `ref` instead of an intermediate and closest point to the matched curve
			merge:       if False, deinterlace will keep the orignal amount of points, juste replacing those to merge by the value of the merge target
	'''
	prec = ref.precision()
	
	# collect segments priority due to interlacement
	priority = [0] * (len(matched)-1)
	originals = [False] * len(matched)
	originals[0] = originals[-1] = True
	j = 0
	for i in range(1,len(ref)):
		if distance2(ref[i], ref[i-1]) <= prec:	
			continue
		while True:
			j += 1
			priority[j-1] = distance(matched[j], matched[j-1]) / distance(ref[i], ref[i-1])
			if distance2(ref[i], matched[j]) <= prec:
				break
		originals[j] = True
		
	assert sum(originals) == len(ref)
	
	# fusionner les segments par ordre de priorité jusqu'a tomber sur des couples d'originaux
	order = SortedList(range(len(priority)), key=priority.__getitem__)	# order the points by their priority
	moved = deepcopy(matched)
	amount = [1]*len(matched)
	merges = list(range(len(matched)))
	
	c = 0
	while order:
		i = order.pop(0)
		print(priority[i])
		if priority[i] >= interlace or c+samples >= len(matched):
			break
		src, dst = i+1, merges[i]
		if originals[dst] and originals[src] and preserve:
			continue
		c += 1
		
		# select merge coefficient
		if   originals[dst]:	x = 0
		elif originals[src]:	x = 1
		else:					x = amount[src] / (amount[src] + amount[dst])
		# apply merge
		merges[src] = dst
		moved[dst] = mix(moved[dst], moved[src], x)
		amount[dst] += amount[src]
		# update the order
		if i+1 < len(priority) and i+1 in order:
			order.remove(i+1)
			priority[i+1] += priority[i]/2
			order.add(i+1)
		if i-1 >= 0 and i-1 in order:
			order.remove(i-1)
			priority[i-1] += priority[i]/2
			order.add(i-1)
		
	print('remain', order)
		
	if merge:
		return [moved[i]  for i in range(len(matched)) if merges[i] == i]
	else:
		for i in range(len(merges)):
			while merges[merges[i]] != merges[i]:
				merges[i] = merges[merges[i]]
		return [moved[i]  for i in merges]
	
from sortedcontainers import SortedList
	
def remove_doubles_zip(matched):
	''' make a zip iterator with matched, each time the tuple formed is the same as the previous one, it remove indices of each matched list 
	
		This is an in-place operation
	'''
	it = zip(*matched)
	last = next(it)
	k = 1
	for current in it:
		if current != last:
			for i, v in enumerate(current):
				matched[i][k] = v
			k += 1
	for m in matched:
		del m[k:]
		
def interpolate_matched(ref, values, matched):
	interpolated = []
	k = 1
	for i in range(len(matched)):
		if ref[k] == matched[i]:
			k += 1
		interpolated.append(mix(values[k-1], values[k], matched[i]-ref[k-1]))
	return interpolated
	


	
from .mesh import mktri, mkquad
	
def blendpair(line1, line2, match='length', tangents='straight', weight=1.):
	indev
	
	
def blendloop(loop, tangents='normal', weight=1., top=None, normal=None, homogenize=True):
	indev

		
def match_length(line1, line2, interlace1=True, interlace2=True) -> '[vec3], [vec3]':
	''' yield couples of point indices where the curved absciss are the closest '''
	match1 = typedlist([line1[0]])
	match2 = typedlist([line2[0]])
	l1, l2 = line1.length(), line2.length()
	i1, i2 = 1, 1
	x1, x2 = 0, 0
	while i1 < len(line1) and i2 < len(line2):
		p1 = distance(line1[i1-1], line1[i1]) / l1
		p2 = distance(line2[i2-1], line2[i2]) / l2
		
		#if p1/l1 <= NUMPREC:
			#i1 += 1
			#continue
		#if p2/l2 <= NUMPREC:
			#i2 += 1
			#continue
		
		x1p = x1+p1
		x2p = x2+p2
		#print('  ', x1p, x2p)
		if abs(x1p-x2p) <= 8*NUMPREC:
			if interlace1 or interlace2:
				match1.append(line1[i1])
				match2.append(line2[i2])
			x1, x2 = x1p, x2p
			i1 += 1
			i2 += 1
		elif x1p < x2p:
			if interlace1:
				match1.append(line1[i1])
				match2.append(mix(line2[i2-1], line2[i2], (x1p-x2)/p2))
			x1 = x1p
			i1 += 1
		elif x1p > x2p:
			if interlace2:
				match1.append(mix(line1[i1-1], line1[i1], (x2p-x1)/p1))
				match2.append(line2[i2])
			x2 = x2p
			i2 += 1
		else:
			raise AssertionError('this case should not happend')
	return match1, match2

def match_distance(line1, line2, interlace1=True, interlace2=True) -> '[vec3], [vec3]':
	''' yield couples of points by cutting each line at the curvilign absciss of the points of the other '''
	match1 = typedlist([line1[0]])
	match2 = typedlist([line2[0]])
	d1 = line1[1]-line1[0]
	d2 = line2[1]-line2[0]
	i1, i2 = 1, 1
	while i1 < len(line1) and i2 < len(line2):
		x1 = dot(line1[i1]-line2[i2-1], line2[i2]-line2[i2-1]) / length2(line2[i2]-line2[i2-1])
		x2 = dot(line2[i2]-line1[i1-1], line1[i1]-line1[i1-1]) / length2(line1[i1]-line1[i1-1])
		if x1 < 0 and x2 < 0  or  x1 > 1 and x2 > 1:
			match1.append(line1[i1])
			match2.append(line2[i2])
			i1 += 1
			i2 += 1
		if x1 <= 1:
			if interlace1:
				match1.append(line1[i1])
				match2.append(mix(line2[i2-1], line2[i2], x1))
			i1 += 1
		elif x2 <= 1:
			if interlace2:
				match1.append(mix(line1[i1-1], line1[i1], x2))
				match2.append(line2[i2])
			i2 += 1
		else:
			raise AssertionError('this case should not happend')
	return match1, match2
	
def match_index(line1, line2, interlace1, interlace2):
	indev
