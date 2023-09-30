# This file is part of pymadcad,  distributed under license LGPL v3

from math import inf
from .mathutils import *
from .mesh import Mesh, Web, Wire, MeshError, connpe, web
from .asso import Asso
from .nprint import nformat, nprint

from copy import copy
from operator import itemgetter


class TriangulationError(Exception):	pass


def triangulation(outline, normal=None, prec=NUMPREC):
	''' triangulate using the prefered method '''
	return triangulation_closest(outline, normal)
	
	
	

	
'''
	triangulation outline
	triangulation using only points from the given outline, creating quite good triangles at cost of efficiency
	O(n**2)
'''

def triangulation_outline(outline: Wire, normal=None, prec=None) -> Mesh:
	''' return a mesh with the triangles formed in the outline
		the returned mesh uses the same buffer of points than the input
		
		complexity:  O(n*k)  where k is the number of non convex points
	'''
	# get a normal in the right direction for loop winding
	if not normal:		normal = outline.normal()
	if prec is None:	prec = outline.precision()
	# the precision we will use in this function is a surface precision:  maximum edge length * minimum edge length
	prec = prec**2/NUMPREC 
	# project all points in the plane
	try:				proj = planeproject(outline, normal)
	except ValueError:	return Mesh()
	# reducing contour, indexing proj and outlines.indices
	hole = list(range(len(outline.indices)))
	if length2(outline[-1]-outline[0]) <= prec:		hole.pop()
	
	# set of remaining non-convexity points, indexing proj
	l = len(outline.indices)
	nonconvex = { i
					for i in hole
					if perpdot(proj[i]-proj[i-1], proj[(i+1)%l]-proj[i]) <= prec
					}
	
	def priority(u,v):
		''' priority criterion for 2D triangles, depending on its shape
		'''
		uv = length(u)*length(v)
		if not uv:	return 0
		return (dot(u,v) + uv) / uv**2
	
	def score(i):
		l = len(hole)
		o = proj[hole[ i ]]
		u = proj[hole[ (i+1)%l ]] - o
		v = proj[hole[ (i-1)%l ]] - o
		triangle = (hole[(i-1)%l], hole[i], hole[(i+1)%l])
		
		# check for badly oriented triangle
		if perpdot(u,v) < -prec:		
			return -inf
		# check for intersection with the rest
		elif perpdot(u,v) > prec:
			# check that there is not point of the outline inside the triangle
			for j in nonconvex:
				if j not in triangle:
					for k in range(3):
						a,b = proj[triangle[k]], proj[triangle[k-1]]
						s = perpdot(a-b, proj[j]-a)
						if k == 1:	s -= 2*prec
						if s <= -prec:
							break
					else:
						return -inf
		# flat triangles that change direction are forbidden, it must be considered as a +360° angle and not a 0° angle
		elif dot(u,v) >= prec:
			return -inf
		return priority(u,v)
	scores = [score(i) for i in range(len(hole))]
	
	triangles = typedlist(dtype=uvec3)
	while len(hole) > 2:
		l = len(hole)
		i = imax(scores)
		if scores[i] == -inf:
			raise TriangulationError("no more feasible triangles (algorithm failure or bad input outline)", [outline.indices[i] for i in hole])
		
		triangles.append(uvec3(
			outline.indices[hole[(i-1)%l]], 
			outline.indices[hole[i]], 
			outline.indices[hole[(i+1)%l]],
			))
		nonconvex.discard(hole.pop(i))
		scores.pop(i)
		l -= 1
		scores[(i-1)%l] = score((i-1)%l)
		scores[i%l] = score(i%l)
	
	return Mesh(outline.points, triangles)

def planeproject(pts, normal=None):
	''' project an outline in a plane, to get its points as vec2 '''
	x,y,z = guessbase(pts, normal)
	return typedlist(vec2(dot(p,x), dot(p,y))	for p in pts)

def guessbase(pts, normal=None, thres=10*NUMPREC):
	''' build a base in which the points will be in plane XY 
		thres is the precision threshold between axis for point selection
	'''
	if normal:
		return dirbase(normal)
	pts = iter(pts)
	try:
		o = next(pts)
		x = vec3(0)
		y = vec3(0)
		ol = max(glm.abs(o))
		xl = 0
		zl = 0
		while xl <= thres:
			p = next(pts)
			x = p-o
			xl = dot(x,x)/max(max(glm.abs(p)), ol)
		x = normalize(x)
		while zl <= thres:
			p = next(pts)
			y = p-o
			zl = length(cross(x,y))
		y = normalize(noproject(y,x))
		return x, y, cross(x,y)
	except StopIteration:
		raise ValueError('unable to extract 2 directions')

def convex_normal(web):
	area = vec3(0)
	pts = web.points
	c = pts[web.edges[0][0]]
	for e in web.edges:
		area += cross(pts[e[1]] - pts[e[0]],  c - pts[e[0]])
	return normalize(area)
	
	
def line_bridges(lines: Web, conn=None) -> 'Web':
	''' find what edges to insert in the given mesh to make all its loops connex.
		returns a Web of the bridging edges.
		
		complexity:  O(n**2 + n*k)
		with
			n = number of points in the lines
			k = average number of points per loop
	'''
	if conn is None:	conn = connpe(lines.edges)
	pts = lines.points
	
	bridges = []	# edges created
	# reached and remaining parts, both as edge indices and point indices
	reached_points = {}	# closest to each reached point
	reached_edges = {}	# closest to each reached edge
	remain_points = set(p 	 for e in lines.edges for p in e)
	remain_edges = set(range(len(lines.edges)))
	
	def propagate(start):
		''' propagate from the given start point to make connex points and edges as reached '''
		front = [start]
		while front:
			s = front.pop()
			if s in reached_points:	continue
			reached_points[s] = (inf, (s,s))
			remain_points.discard(s)
			for e in conn[s]:
				if e in reached_edges: continue
				reached_edges[e] = (inf, (s,s))
				remain_edges.discard(e)
				front.extend(lines.edges[e])
				
	def update_closest():
		# from points to edges
		for s, (score, best) in reached_points.items():
			if best[0] in reached_points:
				reached_points[s] = find_closest_point(s)
		# from edges to points
		for e, (score, best) in reached_edges.items():
			if best[0] in reached_points:
				reached_edges[e] = find_closest_edge(e)
	
	def find_closest_edge(ac):
		''' find the bridge to the closest point to the given edge
		'''
		best = None
		score = inf
		# minimum from edge to points
		ep = lines.edgepoints(ac)
		for ma in remain_points:
			d = distance_pe(pts[ma], ep)
			if d < score:
				e = lines.edges[ac]
				if distance2(pts[ma], pts[e[0]]) < distance2(pts[ma], pts[e[1]]):
					score, best = d, (ma, e[0])
				else:
					score, best = d, (ma, e[1])
		
		return score, best
		
	def find_closest_point(ac):
		''' find the bridge to the closest edge to the given point
		'''
		best = None
		score = inf
		# minimum from point to edges
		for ma in remain_edges:
			d = distance_pe(pts[ac], lines.edgepoints(ma))
			if d < score:
				e = lines.edges[ma]
				if distance2(pts[ac], pts[e[0]]) < distance2(pts[ac], pts[e[1]]):
					score, best = d, (e[0], ac)
				else:
					score, best = d, (e[1], ac)
						
		return score, best
	
	# main loop
	propagate(lines.edges[0][0])
	while remain_edges:
		update_closest()
		closest = min(
					min(reached_points.values(), key=itemgetter(0)),
					min(reached_edges.values(), key=itemgetter(0)),
					key=itemgetter(0)) [1]
		bridges.append(closest)
		bridges.append(tuple(reversed(closest)))
		propagate(closest[0])
		
	return Web(lines.points, bridges)
	

	
	
	
def flat_loops(lines: Web, normal=None) -> '[Wire]':
	''' collect the closed loops present in the given web, so that no loop overlap on the other ones.
	'''
	x,y,z = guessbase(lines.points, normal)
	pts = lines.points
	# create an oriented point to edge connectivity 
	conn = Asso()
	for i,e in enumerate(lines.edges):
		conn.add(e[0], i)
	
	used = [False]*len(lines.edges)	# edges used and so no to use anymore	
	# the start point for loop assembly must be in priority on an edge that has no double, because double edges that are not on a bridge would create independent loops out of an empty surface
	# create lists of edges that have a double and edges that have not
	doubled = []
	nondoubled = []
	for i, e in enumerate(lines.edges):
		double = False
		for j in conn[e[1]]:
			if lines.edges[j][1] == e[0]:
				double = True
		if double:	doubled.append(i)
		else:		nondoubled.append(i)
	# iterator of start points
	choice = (i  for i in (nondoubled + doubled)  if not used[i])
	
	loops = []
	
	while True:
		# find an unused edge
		end = next(choice, None)
		if end is None:
			break
		
		# assemble a loop
		loop = list(lines.edges[end])
		while True:
			# take the most inward next edge
			prev = normalize(pts[loop[-1]] - pts[loop[-2]])
			
			best = None
			score = -inf
			for edge in conn[loop[-1]]:
				if used[edge]:	continue
				e = lines.edges[edge]
				dir = normalize(pts[e[1]] - pts[e[0]])
				if isfinite(dir) and isfinite(prev):
					angle = atan2(dot(cross(prev,dir),z), dot(prev,dir))
				else:
					angle = -pi
				if pi-angle <= pi*NUMPREC:	angle -= 2*pi
				if angle > score:
					score, best = angle, edge
					
			
			# progress on the selected edge
			if best is None:
				raise TriangulationError("there is not only loops in that web", loop)
			used[best] = True
			if best == end:
				break
			
			# continue the loops
			loop.append(lines.edges[best][1])
			
			# the very particular case of loops with null surface, or with straight return to origin might intruduce serveral loops in the same suite, the following condition prevents it
			if loop[0] == loop[-1]:
				prev = normalize(pts[loop[-2]] - pts[loop[-1]])
				dir = normalize(pts[loop[1]] - pts[loop[0]])
				if acos(dot(prev,dir)) <= pi*NUMPREC:
					used[end] = True
					break
			
		loops.append(Wire(lines.points, loop))

	return loops

	
def triangulation_closest(outline, normal=None, prec=None):
	if isinstance(outline, Wire):
		return triangulation_outline(outline, normal, prec)
	else:
		outline = copy(web(outline)).mergegroups()
	x,y,z = dirbase(normal or convex_normal(outline))
	result = Mesh(outline.points)
	for loop in flat_loops(outline + line_bridges(outline), z):
		result += triangulation_outline(loop, z, prec)
	return result


	

import numpy.core as np
from .mesh import connexity
	
def sweepline_monotones(outline: Web, direction=0, prec=0) -> Mesh:
	'''
		complexity: O(n*log(n) + n*k)  with k = number of loops
	'''
	pts = outline.points
	print('direction', direction)
	
	# get a point on the given edge for the given x absciss
	def extend_edge(edge, x):
		a, b = pts[edge[-2]], pts[edge[-1]]
		try:
			assert a[direction] - prec <= x <= b[direction] + prec, "web is not closed or not manifold"
		except AssertionError:
			print(a[direction], x, b[direction])
			raise
		v = b - a
		return (x - a[direction]) / v[direction] * v + a
	
	def extendable_edge(edge, x):
		return len(edge) > 1 and x - pts[edge[-2]][direction] >= -prec
		
	def between_edges(prev, next, p):
		x = p[direction]
		a, b = extend_edge(mono[0], x), extend_edge(mono[1], x)
		return distance2(a, p) + distance2(b, p) < distance2(a, b)
		
	def reorder(edge):
		if pts[edge[0]][direction] < pts[edge[1]][direction]:
			return edge[0], edge[1], 1
		else:
			return edge[1], edge[0], 0
		
	# sort edges along direction (and forwardness for degenerated edges)
	def key(edge):
		p, n, _ = reorder(edge)
		v = pts[n] - pts[p]
		return (
			pts[p][direction], 
			abs(v[direction-1] - v[direction-2]) / (v[direction] + prec),
			)
	ordered = sorted(outline.edges, key=key)
	
	# set of points being at the start of an edge, helping fixing degenerated cases of edges at the same place
	starts = set(reorder(edge)[0]  for edge in outline.edges)
	
	print(ordered)
	monotonics = []
	finished = []
	for k, edge in enumerate(ordered):
		p, n, orientation = reorder(edge)
		x = pts[p][direction]
		
		for i in reversed(range(len(monotonics))):
			mono = monotonics[i]
			# close current monotonic
			if (mono[0][-1] == mono[1][-1] 
					and x >= pts[mono[0][-1]][direction] - prec):
				print('close', mono)
				finished.append(monotonics.pop(i))
		
		# process merges abd closings so the new point can be processed with a clean set of monotonics
		for i in reversed(range(1, len(monotonics))):
			prev = monotonics[i-1]
			next = monotonics[i]
			# merge two monotonics
			if (prev[1][-1] == next[0][-1] 
					and prev[1][-1] not in starts 
					and x >= pts[prev[1][-1]][direction] - prec):
				print('merge', prev, next)
				# find a helper point
				for j in range(k, len(ordered)):
					h, _, _ = reorder(ordered[j])
					hx = pts[h][direction]
					print(h)
					# assert pts[prev[0][-1]][direction] - hx >= -prec and pts[next[1][-1]][direction] - hx >= -prec 
					if h == prev[0][-1] or h == next[1][-1] or between_edges(prev[0], next[1], pts[h]):
						helper = h
						break
				else:
					raise AssertionError
				
				prev[1].append(helper)
				next[0].append(helper)
			
		for i, mono in enumerate(monotonics):
			found = True
			print('*', mono, p, n)
			
			# extend first side
			if p == mono[0][-1] and orientation == 1:
				print('extend prev')
				mono[0].append(n)
			# extend second side
			elif p == mono[1][-1] and orientation == 0:
				print('extend next')
				mono[1].append(n)
			# fins whether the current monotonic is concerned by this new edge
			elif extendable_edge(mono[0], x) and extendable_edge(mono[1], x) and between_edges(mono[0], mono[1], pts[p]):
				# split monotonic
				if orientation:
					monotonics[i:i+1] = ([mono[0][-2:], [p]], [[p, n], mono[1][-2:]])
				else:
					monotonics[i:i+1] = ([mono[0][-2:], [p, n]], [[p], mono[1][-2:]])
				mono[0][-1] = p
				mono[1][-1] = p
				finished.append(mono)
				print('split', monotonics)
				break
			else:
				continue
			
			break
		# create monotonic
		else:
			if orientation:
				monotonics.append([[p, n], [p]])
			else:
				monotonics.append([[p], [p, n]])
			
	# - [x] TODO: fix degenerated case of 2 edges at the same place
	# - [ ] TODO: fix degenerated case of a null-length edge
	# - [ ] TODO: fix degenerated case of 2 monotonics with the exact same flat front, with one getting challenged by an edge of the other
	# - [x] TODO: fix degenerated case of empty loop
	# - [ ] TODO: fix degenerated case of split point on an edge
	
	# assert not monotonics, "web is not closed or not manifold"
	# print(monotonics)
	finished.extend(monotonics)
	return finished
	
def interclass(a, b, key=lambda x: x):
	result = []
	ia, ib = iter(a), iter(b)
	va, vb = next(ia, None), next(ib, None)
	ka, kb = key(va), key(vb)
	if (va and vb) is not None:
		while True:
			if ka <= kb:
				result.append(va)
				va = next(ia, None)
				if va is None:	break
				ka = key(va)
			else:
				result.append(vb)
				vb = next(ib, None)
				if vb is None:  break
				kb = key(vb)		
	while va is not None:
		result.append(va)
		va = next(ia, None)
	while vb is not None:
		result.append(vb)
		vb = next(ib, None)
	return result
		
		
	
def triangulation_monotone(sidea, sideb, direction, normal, prec=0) -> Mesh:
	''' ugly triangulation of a monotone outline
		the outline is divided into `sida` and `sideb`, each being a wire monotonic in `direction`
		
		complexity: O(n)
		
		Arguments:
			indev
	'''
	points = sidea.points
	if sideb.points is not points:
		l = len(points)
		points += sidea
		sideb = Wire(points, [i+l  for i in sideb.indices])
	
	ordered = interclass(
					[(p,0)  for p in sidea.indices], 
					[(p,1) for p in sideb.indices], 
				key=lambda p: (
					points[p[0]][direction],  # sorted along direction
					points[p[0]][direction-1] + points[p[0]][direction-2],  # in case of ambiguity
					))
	stack = [ordered[0], ordered[1]]
	triangles = []
	
	print(sidea.indices, sideb.indices)
	print(ordered)
	
	for i in range(2, len(ordered)):
		if ordered[i][1] != stack[-1][1]:
			print(ordered[i], 'foreign', stack)
			while len(stack) > 1:
				j = stack.pop()
				k = stack[-1]
				if ordered[i][1]:  
					j, k = k, j
				triangles.append(uvec3(ordered[i][0], j[0], k[0]))
				print(' ', uvec3(ordered[i][0], j[0], k[0]))
			stack.clear()
			stack.append(ordered[i-1])
			stack.append(ordered[i])
		else:
			print(ordered[i], 'neigh', stack)
			while len(stack) > 1:
				j = stack.pop()
				k = stack[-1]
				if ordered[i][1]:  
					e = j[0], k[0]
				else:
					e = k[0], j[0]
				if dot(
						cross(
							points[e[0]] - points[ordered[i][0]], 
							points[e[1]] - points[ordered[i][0]]), 
						normal,
						) < -prec:
					print(' not ', uvec3(ordered[i][0], e[0], e[1]))
					stack.append(j)
					break
				triangles.append(uvec3(ordered[i][0], e[0], e[1]))
				print(' ', uvec3(ordered[i][0], e[0], e[1]))
			stack.append(ordered[i])
	return Mesh(points, triangles)

	
def retriangulate(mesh) -> Mesh:
	closest = {}  # best match for each point
	# propagate in one curse (whatever it is)
	# propagate the opposite
	from .rendering import show
	show([mesh])
	indev

def triangulation_sweepline(outline: Web, normal=None, prec=None) -> Mesh:
	pts = outline.points
	if prec is None:
		prec = outline.precision()*8
	# use the average normal if not provided
	if normal is None:	
		normal = convex_normal(outline)
	# choose one of the coordinates as sweepline direction, the coordinate with widest range
	direction = np.argmax(glm.abs(boundingbox(
		pts[i]  for e in outline.edges for i in e
		).width))
	# triangulate monotone parts
	result = Mesh()
	for (sidea, sideb) in sweepline_monotones(outline, direction, prec):
		result += triangulation_monotone(Wire(pts, sidea), Wire(pts, sideb), direction, normal, prec)
	return result

def triangulation(outline: Web, normal=None, prec=None) -> Mesh:
	# return retriangulate(triangulation_sweepline(outline, normal, prec))
	return triangulation_sweepline(outline, normal, prec)
	
