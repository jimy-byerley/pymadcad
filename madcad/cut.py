# This file is part of pymadcad,  distributed under license LGPL v3
''' This module provides the chamfer and bevel functions, and the associated tools.

	`multicut`, `chamfer`, and `bevel` are built in the same cutting algorithm. It cuts the mesh faces by propagation from the given edges. The cutting planes are determined by an offset vector from the original primitive (point or edge). 
	
	Most of the time you don't need to set the offset yourself. It can be automatically calculated by several methods, depending on the shape you want to get. Those methods are called `cutters` and are executed using `planeoffsets`_.
'''

from .mathutils import *
from .mesh import *
from . import generation as gt
from . import hashing
from . import text
from . import settings
from .triangulation import triangulation_outline
from .blending import blenditer, match_length

from numbers import Number
from functools import singledispatch

__all__ = [	'chamfer', 'bevel', 'multicut',
			'mesh_cut', 'web_cut', 'planeoffsets',
			'tangentend', 'tangentcorner', 'tangentjunction',
			'cutter_width', 'cutter_distance', 'cutter_depth', 'cutter_radius',
			]
			
			
# ---- common cut methods ----

@singledispatch
def multicut(mesh, indices, cutter):
	''' cut a Mesh/Web/Wire around the given edges/points, using the given cutter '''
	raise TypeError('wrong argument type: {}'.format(type(mesh)))

@singledispatch
def chamfer(mesh, indices, cutter):
	''' chamfer a Mesh/Web/Wire around the given edges/points, using the given cutter '''
	raise TypeError('wrong argument type: {}'.format(type(mesh)))
	
@singledispatch
def bevel(mesh, indices, cutter):
	''' bevel a Mesh/Web/Wire around the given edges/points, using the given cutter '''
	raise TypeError('wrong argument type: {}'.format(type(mesh)))




# ---- cut methods -----

def cutter_width(width, fn1, fn2):
	''' plane offset for a cut based on the width of the bevel '''
	n = normalize(fn1+fn2)
	s = dot(fn1,n)
	return -width/2 * sqrt(1/s**2 - 1) * n

def cutter_distance(depth, fn1, fn2):
	''' plane offset for a cut based on the distance along the side faces '''
	return -depth * normalize(fn1+fn2)

def cutter_depth(dist, fn1, fn2):
	''' plane offset for a cut based on the distance to the cutted edge '''
	return -dist * cross(normalize(cross(fn1,fn2)), fn1-fn2)

def cutter_radius(depth, fn1, fn2):
	''' plane offset for a cut based on the angle between faces '''
	n = normalize(fn1 + fn2)
	s = dot(fn1,n)
	return -depth * (1/s - s) * n




# ----- mesh operations ------

def planeoffsets(mesh, edges, cutter):
	''' compute the offsets for cutting planes using the given method 
		cutter is a tuple or a function
		
			- function(fn1,fn2) -> offset 		
				fn1, fn2 are the adjacents face normals
				offset is the distance from segment to plane times the normal to the plane
				
			- ('method', distance) 				
				the method is the string name of the method (a function named 'cutter_'+method existing in this module)
				distance depends on the method and is the numeric parameter of the method
	'''
	if isinstance(cutter, dict):
		return cutter
	else:
		cutter = interpretcutter(cutter)
		# get adjacent faces' normals to lines
		offsets = {e: [None,None]  for e in edges}
		for f in mesh.faces:
			for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
				if e in offsets:					offsets[e][0] = mesh.facenormal(f)
				elif (e[1],e[0]) in offsets:		offsets[(e[1],e[0])][1] = mesh.facenormal(f)
		
		# compute offsets (displacement on depth)
		for e, adjacents in offsets.items():
			offset = cutter(*adjacents)
			if dot(cross(*adjacents), mesh.points[e[0]]-mesh.points[e[1]]) > 0:
				offset = -offset
			offsets[e] = offset
		
		return offsets

def interpretcutter(cutter):
	if isinstance(cutter, tuple):
		func = globals()['cutter_'+cutter[0]]
		arg = cutter[1]
		return lambda fn1,fn2: func(arg, fn1, fn2)
	elif callable(cutter):
		return cutter
	else:
		raise TypeError("cutter must be a callable or a tuple (name, param)")

		
def mesh_cut(mesh, start, cutplane, stops, conn, prec, removal, cutghost=True):
	''' propagation cut for an edge 
		
		:start:		the edge or point to start propagation from
		:cutplane:	the plane cutting the faces. Its normal must be oriented toward the propagation area.
		:stops:     the planes stopping the propagation. Their normal must be oriented toward the propagation area.
		:removal:   the set in wich the function will put the indices of faces inside
		:cutghost:  whether the function should propagate on faces already marked for removal (previously or during the propagation)
	'''
	pts = mesh.points
	ptmgr = hashing.PointSet(prec, manage=pts)
	stops = list(filter(lambda e:e, stops))
	# find the intersections axis between the planes
	axis = [intersection_plane_plane(cutplane, stop)	for stop in stops]
	
	# prepare propagation
	if isinstance(start, tuple):
		front = [start, (start[1],start[0])]
	elif isinstance(start, Number):
		front = [e	for e in conn if start in e]
	else:
		raise TypeError('wrong start: {}, cut can only start from a point or an edge'.format(start))
	intersections = set()
	seen = set()
	while front:
		# propagation affairs
		frontedge = front.pop()
		
		if frontedge not in conn:	continue
		fi = conn[frontedge]
		if fi in seen:	continue
		f = mesh.faces[fi]
		
		# do not process empty faces (its a topology singularity), but propagate
		if faceheight(mesh, fi) <= prec:
			seen.add(fi)
			for j in range(3):
				front.append((f[j],f[j-1]))
			continue
		
		# find the intersection of the triangle with the common axis to the cutplane and the stop plane
		fpts = mesh.facepoints(fi)
		p = None
		for a in axis:
			# if the intersection is on a triangle's edge, p is None
			p = intersection_axis_face(a, fpts, prec)
			if p:	break
		if p:
			# mark cutplane change
			# empty faces can be generated, but they are necessary to keep the connectivity working
			pi = ptmgr.add(p)
			l = len(mesh.faces)
			mesh.faces[fi] = (f[0], f[1], pi)
			mesh.faces.append((f[1], f[2], pi))
			mesh.faces.append((f[2], f[0], pi))
			mesh.tracks.append(mesh.tracks[fi])
			mesh.tracks.append(mesh.tracks[fi])
			registerface(mesh, conn, fi)
			registerface(mesh, conn, l)
			registerface(mesh, conn, l+1)
			front.append(frontedge)
			continue
		
		# mark this face as processed
		seen.add(fi)
		
		# point side for propagation
		# True if the point is over or on the cutplane
		goodside = [False]*3
		for j,pi in enumerate(f):
			goodside[j] = 	dot(pts[pi]-cutplane[0], cutplane[1]) >= -prec
		
		# abort conditions
		if not any(goodside):
			continue
		outside = any( all( dot(pts[pi]-stop[0], stop[1]) <= -prec
							for pi in f)
						for stop in stops)
		if outside:
			continue
		
		# True if the point is over or on for all stop planes
		goodx = [False]*3
		for j,pi in enumerate(f):
			goodx[j] = all(dot(pts[pi]-stop[0], stop[1]) >= -prec  	for stop in stops)
			
		
		# intersections of triangle's edges with the plane
		cut = [None]*3
		for j in range(3):
			cut[j] = intersection_edge_plane((pts[f[j]], pts[f[j-2]]), cutplane, prec)
			
		
		# don't intersect if the 2 points are at the corner of the triangle
		for j in range(3):
			if cut[j-1] and cut[j] and distance(cut[j-1], cut[j]) <= prec:	
				cut[j] = None
	
		if all(goodside) and any(goodx):
			removal.add(fi)
		
		if fi not in removal or cutghost:	# NOTE: this is a ugly trick to make the current multicut implementation work with the same function for corners and edges
			# cut the face
			for j in range(3):
				if cut[j] and cut[j-1]:
					# cut only if the intersection segment is in the delimited area
					if any(	dot(cut[j-1]-stop[0], stop[1]) <= prec 
						and	dot(cut[j]  -stop[0], stop[1]) <= prec
							for stop in stops):
						continue
					
					# cut the face (create face even for non kept side, necessary for propagation)
					p1 = ptmgr.add(cut[j])
					p2 = ptmgr.add(cut[j-1])
					
					l = len(mesh.faces)
					mesh.faces[fi] = (p1, f[j-2], f[j-1])
					mesh.faces.append((p1, f[j-1], p2))
					mesh.faces.append((p1, p2, f[j-0]))
					mesh.tracks.append(mesh.tracks[fi])
					mesh.tracks.append(mesh.tracks[fi])
					registerface(mesh, conn, fi)
					registerface(mesh, conn, l)
					registerface(mesh, conn, l+1)
					seen.update((fi, l, l+1))
					# mark to remove the faces outside
					if dot(pts[f[j]]-cutplane[0], cutplane[1]) < 0:
						removal.add(fi)
						removal.add(l)
						intersections.add((p2,p1))
					else:
						removal.add(l+1)
						removal.discard(fi)
						intersections.add((p1,p2))
					break
		# propagate
		for j in range(3):
			front.append((f[j],f[j-1]))
	
	return intersections
			
	
def propagate(mesh, edges, conn, prec):
	front = list(edges)
	edges = set(edges)
	seen = set()
	count = 0
	while front:
		edge = front.pop()
		if edge not in conn:	continue
		fi = conn[edge]
		if fi in seen:	continue
		seen.add(fi)
		count += 1
		if faceheight(mesh, fi) <= prec:	continue
		
		a,b,c = arrangeface(mesh.faces[fi], edge[0])
		if c in edge:	continue
		if (b,c) not in edges:		front.append((c,b))
		if (c,a) not in edges:		front.append((a,c))
	return seen

	
@multicut.register(Mesh)
def mesh_multicut(mesh, edges, cutter, conn=None, prec=None, removal=None):
	''' general purpose edge cutting function.
		cut the given edges and the crossing corners, resolving the interference issues.
		
		cutter can either be a argument for planeoffsets or more directly a dict `{edgekey: offset vector}` for each passed edge.
		edges must be unoriented.
	'''
	# perpare working objects
	if conn is None:	conn = connef(mesh.faces)
	if prec is None:	prec = mesh.precision()
	if removal is None:	
		removal = set()
		final = True
	else:
		final = False
	if isinstance(edges, Web):	edges = edges.edges
	edges = [edgekey(*e)  for e in edges]
	offsets = planeoffsets(mesh, edges, cutter)
	
	normals = mesh.vertexnormals()
	juncconn = connpp(edges)
	pts = mesh.points
	
	debmesh = mesh
	separators = {}	# plans separateurs pour chaque arete
	for i, prox in juncconn.items():
		if len(prox) != 2:	continue
		o0 = offsets[edgekey(i,prox[0])]
		o1 = offsets[edgekey(i,prox[1])]
		axis = intersection_plane_plane((pts[prox[0]]+o0, o0), (pts[prox[1]]+o1, o1), pts[i])	# fail when the two planes have the same orientation
		if axis:
			n = normalize(cross(pts[i]-axis[0], axis[1]))
			if dot(n, pts[prox[0]]-pts[i]) < 0:		n = -n
			separators[(i,prox[0])] = (axis[0], n)
			separators[(i,prox[1])] = (axis[0],-n)
			
			## display cut planes in the mesh (debug)
			#grp = len(debmesh.groups)
			#debmesh.groups.append(None)
			#m = len(debmesh.points)
			#debmesh.points.append(pts[i])
			#debmesh.points.append(axis[0]+axis[1])
			#debmesh.points.append(axis[0]-axis[1])
			#debmesh.faces.append((m,m+1,m+2))
			#debmesh.tracks.append(grp)
			
		else:
			separators[(i,prox[0])] = None
			separators[(i,prox[1])] = None
	# fix the missing separators in case of straight lines
	for e,sep in separators.items():
		i,j = e
		while not sep:
			prox = juncconn[i]
			if len(prox) != 2:	raise Exception('unable to compute all separators')
			j,i = i, prox[0] if prox[0] != j else prox[1]
			separators[e] = sep = separators[(i,j)]
	
	corners = {}	# offset pour chaque coin
	# pour chaque jonction
	for junc, prox in juncconn.items():
		if len(prox) < 3:	continue
		# calculer le plan de distance max demandé par les aretes adjacentes
		offset = offsets.get(junc, 0.)
		# pour chaque arete adjacente (dans chaque sens)
		for b in prox:
			for c in prox:
				a = junc
				if b == c or b == a or c == a:	continue
				# calculer la distance
				ab = normalize(pts[b]-pts[a])
				ac = normalize(pts[c]-pts[a])
				n = cross(ab, ac)
				nb = cross(n, ab)
				nc = cross(ac, n)
				ob = offsets[edgekey(a,b)]
				oc = offsets[edgekey(a,c)]
				ib = dot(ob,ob) / dot(ob, nb)
				ic = dot(oc,oc) / dot(oc, nc)
				s = dot(nb,ac)
				if not s:	continue
				abl = (ib*dot(ab,ac) + ic) / s
				offset = dot(abl*ab + ib*nb, normals[a])
		# assignation du plan de coupe
		corners[junc] = offset*normals[junc]
		plane = (pts[junc] + corners[junc], -normals[junc])
		for p in prox:
			separators[(junc,p)] = plane
	
	outlines = {}
	# couper les aretes
	for edge, offset in offsets.items():
		outlines[edge] = mesh_cut(mesh, edge, 
								(pts[edge[0]]+offset, -normalize(offset)), 
								(separators.get(edge), separators.get((edge[1], edge[0]))), 
								conn, prec, removal, True)
	# couper les sommets
	for corner,offset in corners.items():
		outlines[corner] = mesh_cut(mesh, corner, 
								(pts[corner]+offset, -normalize(offset)), 
								(),
								conn, prec, removal, False)
	if final:
		frontier = []
		for edges in outlines.values():
			frontier.extend(edges)
		finalize(mesh, outlines, propagate(mesh, frontier, conn, prec), prec)
		#finalize(mesh, outlines, removal, prec)	# NOTE this is faster but also leads to not removing all faces
	return outlines

def finalize(mesh, outlines, removal, prec):
	''' function finalizing the mesh for multicut
		removing faces and simplifying outlines
	'''
	# simplify cuts
	merges = {}
	for e,cuts in outlines.items():
		reindex = line_simplification(Web(mesh.points, cuts), prec)
		lines = []
		for a,b in cuts:
			c,d = reindex.get(a,a), reindex.get(b,b)
			if c != d:	lines.append((c,d))
		outlines[e] = lines
		merges.update(reindex)
	# apply merges, but keeping the empty faces, to keep the removal list valid
	for i,f in enumerate(mesh.faces):
		mesh.faces[i] = (
			merges.get(f[0],f[0]),
			merges.get(f[1],f[1]),
			merges.get(f[2],f[2]),
			)
	
	# delete inside faces and empty ones
	removefaces(mesh, lambda fi: fi in removal or faceheight(mesh,fi) <= prec)

def arrangeface(f, p):
	if   p == f[1]:	return f[1],f[2],f[0]
	elif p == f[2]:	return f[2],f[0],f[1]
	else:			return f

def registerface(mesh, conn, fi):
	f = mesh.faces[fi]
	if f[0] == f[1] or f[1] == f[2] or f[2] == f[0]:
		return
	for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
		conn[e] = fi
		
def unregisterface(mesh, conn, fi):
	f = mesh.faces[fi]
	for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
		if e in conn:	del conn[e]

def segmentsdict(line):
	segments = {}
	for i in range(len(line)-1):
		segments[(line[i],line[i+1])] = i
	return segments

def intersection_plane_plane(p0, p1, neigh=None):
	''' return the intersection axis between two planes '''
	if not neigh:	neigh = p0[0]
	d = cross(p0[1], p1[1])
	if length(d) <= NUMPREC:	return None
	return vec3(
		inverse(transpose(dmat3(p0[1], p1[1], d))) 
		* dvec3(dot(p0[0],p0[1]), dot(p1[0],p1[1]), dot(neigh,d))
		), normalize(d)

def intersection_edge_plane(edge, axis, prec):
	''' return the intersection point of an edge with a plane, or None if it doesn't exist 
		In case of an intersection at an extremity of the edge, return that edge point
	'''
	a, b = edge
	da = dot(axis[0]-a, axis[1])
	db = dot(axis[0]-b, axis[1])
	if abs(da) <= prec:		return a
	if abs(db) <= prec:		return b
	if da * db > 0:		return None		# the segment doesn't reach the plane
	edgedir = b-a
	proj = dot(edgedir, axis[1])
	if abs(proj) <= prec:	return None	# the segment is parallel to the plane
	return a + da * edgedir / proj

def intersection_axis_face(axis, face, prec):
	''' intersection between an axis and a triangle
		In case of intersection with an edge of the triangle, return None
	'''
	n = cross(face[1]-face[0], face[2]-face[0])
	unp = dot(n, axis[1])
	if abs(unp) <= prec:	return None
	p = axis[0] + dot(face[0]-axis[0], n) / unp * axis[1]
	for i in range(3):
		if dot(p - face[i], normalize(cross(n,face[i]-face[i-1]))) <= prec:
			return None
	return p



def removefaces(mesh, crit):
	''' remove faces whose indices are present in faces, (for huge amount, prefer pass faces as a set) '''
	newfaces = typedlist(dtype=uvec3)
	newtracks = typedlist(dtype='I')
	for i in range(len(mesh.faces)):
		if not crit(i):
			newfaces.append(mesh.faces[i])
			newtracks.append(mesh.tracks[i])
	mesh.faces = newfaces
	mesh.tracks = newtracks
	
def removeedges(mesh, crit):
	''' remove faces whose indices are present in faces, (for huge amount, prefer pass faces as a set) '''
	newedges = typedlist(dtype=uvec2)
	newtracks = typedlist(dtype='I')
	for i in range(len(mesh.edges)):
		if not crit(i):
			newedges.append(mesh.edges[i])
			newtracks.append(mesh.tracks[i])
	mesh.edges = newedges
	mesh.tracks = newtracks

#def facesurf(mesh, fi):
	#o,x,y = mesh.facepoints(fi)
	#return length(cross(x-o, y-o))

#def faceangle(mesh, fi):
	#o,x,y = mesh.facepoints(fi)
	#return length(cross(normalize(x-o), normalize(y-o)))

def faceheight(mesh, fi):
	f = mesh.facepoints(fi)
	m = inf
	for i in range(3):
		a,b = f[i]-f[i-1], f[i]-f[i-2]
		if not dot(b,b):	return 0
		m = min(m, dot(a,a) - dot(a,b)**2/dot(b,b))
	return sqrt(max(0,m))
	
def faceheight(mesh, fi):
	f = mesh.facepoints(fi)
	m = inf
	for i in range(3):
		l = length(f[i-2]-f[i])
		if not l:	return 0
		h = length(cross(f[i-2]-f[i], f[i-1]-f[i])) / l
		if h < m:	m = h
	return m



@chamfer.register(Mesh)
def mesh_chamfer(mesh, edges, cutter):
	''' create a chamfer on the given suite of points, create faces are planes.
		cutter is described in function planeoffsets()
	'''
	# cut faces
	segments = mesh_multicut(mesh, edges, cutter)
	
	# create junctions
	group = len(mesh.groups)
	mesh.groups.append(None)
	pts = mesh.points
	# edges for corners surfaces
	corners = {}
	def cornerreg(c, edge):
		if c not in corners:	corners[c] = []
		corners[c].append(edge)
	
	for e,s in segments.items():
		if not s:	continue
		# corner cuts
		if isinstance(e, Number):
			if e not in corners:	corners[e] = []
			corners[e].extend(s)
		# edge cuts
		else:
			# assemble the intersection segments in a single outline
			frags = suites(s)
			if len(frags) == 1:		
				lp = frags[0]
				cornerreg(e[0], (lp[0],lp[-1]))
			elif len(frags) == 2:	
				lp = frags[0] + frags[1]
				# get the sides in the edge direction
				l,r = frags
				if dot(pts[l[-1]]-pts[l[0]], pts[e[1]]-pts[e[0]]) < 0 or dot(pts[r[-1]]-pts[r[0]], pts[e[1]]-pts[e[0]]) > 0:
					l,r = r,l
				cornerreg(e[0], (l[0], r[-1]))
				cornerreg(e[1], (r[0], l[-1]))
			else:
				raise Exception('a cutted edge has more than 2 cutted sides')
			# triangulate cutted edge
			faces = triangulation_outline(Wire(mesh.points, lp)).faces
			mesh.faces.extend(faces)
			mesh.tracks.extend([group]*len(faces))
	
	for c,s in corners.items():
		# triangulate cutted corner
		faces = triangulation_outline(Wire(mesh.points, suites(s)[0])).faces
		mesh.faces.extend(faces)
		mesh.tracks.extend([group]*len(faces))

@bevel.register(Mesh)
def mesh_bevel(mesh, edges, cutter, resolution=None):
	''' create a chamfer on the given suite of points, create faces are planes.
		cutter is described in function planeoffsets()
	'''
	if isinstance(edges, Web):	edges = edges.edges
	
	edges = [edgekey(*e)	for e in edges]
	conn = connef(mesh.faces)
	enormals = {}
	for e in edges:
		enormals[e] = (
			mesh.facenormal(conn[e]),
			mesh.facenormal(conn[(e[1],e[0])]),
			)
	# cut faces
	segments = mesh_multicut(mesh, edges, cutter, conn)
	mesh.check()
	conn = connef(mesh.faces)
	pts = mesh.points
	
	# edges for corners surfaces
	corners = {}
	def cornerreg(c, edge):
		if c not in corners:	corners[c] = []
		corners[c].append(edge)
	
	# create junctions
	normals = {}	# face normal at each point
	junctions = []
	ends = []
	for e,s in segments.items():
		if not s:	continue
		# corner cuts
		if isinstance(e, Number):	
			if e not in corners:	corners[e] = []
			corners[e].extend(s)
		# edge cuts
		else:
			# assemble the intersection segments in a single outline
			frags = suites(s)
			
			# one loop:  bevel extremity
			if len(frags) == 1:
				lp = frags[0]
				# find a separation to put a junction between 2 sides
				side = noproject(pts[lp[0]]-pts[lp[-1]], normalize(pts[e[1]] - pts[e[0]]))
				for i in range(1,len(lp)):
					if dot(pts[lp[i-1]]-pts[e[0]], side) * dot(pts[lp[i]]-pts[e[0]], side) < 0:
						l,r = lp[:i], lp[i:]
						break
				cornerreg(e[0], (lp[0],lp[-1]))
				if (lp[i], lp[i-1]) in conn:
					ends.append((lp[i-1], lp[i]))
			# two parts: cutted edge
			elif len(frags) == 2:
				l,r = frags
				if dot(pts[l[-1]]-pts[l[0]], pts[e[1]]-pts[e[0]]) < 0 or dot(pts[r[-1]]-pts[r[0]], pts[e[1]]-pts[e[0]]) > 0:
					l,r = r,l
				cornerreg(e[0], (l[0], r[-1]))
				cornerreg(e[1], (r[0], l[-1]))
			else:
				raise Exception('a cutted edge has more than 2 cutted sides')
			
			# match the curves
			r.reverse()
			match = list(match_length(Wire(pts,l), Wire(pts,r)))
			# prepare tangents
			ll = Wire(pts,l).length()
			x = 0.
			for i in range(len(match)):
				if i:	x += distance(pts[match[i-1][0]], pts[match[i][0]]) / ll
				# get the tangents from the triangle between the cuts and the cutting edge
				l,r = match[i]
				o = interpol1(pts[e[1]], pts[e[0]], x)
				plane = cross(pts[r]-o, pts[l]-o)
				normals[l] = enormals[e][1] + normals.get(l, 0)
				normals[r] = enormals[e][0] + normals.get(r, 0)
			junctions.append(match)
		
	# normals neighbooring corners
	for e,s in corners.items():
		for a,b in s:
			if (b,a) in conn:
				n = mesh.facenormal(conn[(b,a)])
				normals[b] = normals.get(b, 0) + n
				normals[a] = normals.get(a, 0) + n
	# normals neighbooring cut ends
	for a,b in ends:
		n = mesh.facenormal(conn[(b,a)])
		normals[a] = noproject(normals[a], n)
		normals[b] = noproject(normals[b], n)
	# normalize all contributions
	for k,n in normals.items():
		normals[k] = normalize(n)
	
	# compute resolution based on adjacent faces to edges
	div = 0
	c = 0
	for match in junctions:
		for a,b in match:
			div += settings.curve_resolution(
						arclength(a,b,normals[a],normals[b]), 
						anglebt(normals[a], normals[b]),
						resolution)
			c += 1
	div //= c
	
	new = Mesh()
	# round cutted corner	
	for e,s in corners.items():
		if len(s) < 3:	continue
		# assemble the intersection segments in a single outline
		lp = suites(s)[0]
		if lp[-1] == lp[0]:	lp.pop()	# remove the redundant point if there is (not all the time due to some BUG)
		new += tangentcorner(pts, lp, normals, div)
	# fill gap between existing surface and round extremities
	for edge in ends:
		n = mesh.facenormal(conn[(edge[1],edge[0])])
		new += tangentend(pts, edge, normals, div)
	# round cutted edges
	for match in junctions:
		# put a junction
		new += tangentjunction(pts, match, normals, div)
	
	new.groups = [None]
	new.tracks = typedlist.full(0, len(new.faces), 'I')
	mesh += new
	mesh.mergeclose()

	
	
	
	
# ---- web operations -----
from .nprint import nprint
	
def web_cut(web, start, cutplane, conn, prec, removal):
	''' propagation cut for a point 
		
		:start:		the edge or point to start propagation from
		:cutplane:	the plane cutting the faces. Its normal must be oriented toward the propagation area.
	'''
	intersections = []
	
	# propagate from point to point
	seen = set()
	front = list(conn[start])
	while front:
		ei = front.pop()
		if ei in seen:	continue
		seen.add(ei)
		e = web.edges[ei]
		
		# look intersection with the cutplane
		ep = web.edgepoints(e)
		side = [dot(p-cutplane[0], cutplane[1])		for p in ep]
		if side[0]*side[1] <= prec:
			p = intersection_edge_plane(ep, cutplane, prec)
			
			pi = len(web.points)
			web.points.append(p)
			intersections.append(pi)
			if side[0] <= prec:
				if distance2(p,ep[0]) < prec**2:
					pi = e[0]
				web.edges[ei] = (pi, e[1]) 
				web.edges.append((e[0], pi))
				web.tracks.append(web.tracks[ei])
				conn.discard(e[0], ei)
				conn.add(e[0], len(web.edges)-1)
			else:
				if distance2(p,ep[1]) < prec**2:
					pi = e[1]
				web.edges[ei] = (e[0], pi)
				web.edges.append((pi, e[1]))
				web.tracks.append(web.tracks[ei])
				conn.discard(e[1], ei)
				conn.add(e[1], len(web.edges)-1)
		else:
			# propagate
			for p in e:
				front.extend(conn[p])
		
	removal.update(seen)
	return intersections

@multicut.register(Web)
def web_multicut(web, points, cutter, conn=None, prec=None, removal=None):
	if conn is None:	conn = connpe(web.edges)
	if prec is None:	prec = web.precision()
	if isinstance(points, Wire):	points = points.indices
	
	removal = set()
	cutter = interpretcutter(cutter)
	pts = web.points
	intersections = {}
	
	for pi in points:
		dirs = []
		for ni in conn[pi]:
			e = web.edges[ni]
			dirs.append(normalize( pts[pi] - pts[e[e[0] == pi]] ))
		if not dirs:  # the cut has removed everything connected
			continue
		normal = normalize(sum(dirs))
		c = sum(dot(normal,d) for d in dirs) / len(dirs)
		s = sqrt(max(0, 1-c**2))
		x,_,_ = dirbase(normal)
		offset = cutter(c*x + s*normal, -c*x + s*normal)
		if dot(offset, normal) > 0:	offset = -offset
		plane = (pts[pi]+offset, -normalize(offset))
		
		intersections[pi] = web_cut(web, pi, plane, conn, prec, removal)
	
	removeedges(web, lambda i: i in removal)	
	return intersections
	
@chamfer.register(Web)
def web_chamfer(web, points, cutter):
	holes = web_multicut(web, points, cutter)

	g = len(web.groups)
	web.groups.append(None)
	
	conn = connpe(web.edges)
	def link(e):
		if web.edges[next(conn[e[0]])][0] == e[0]:	return (e[1],e[0])
		else:                                  		return e
	
	for cuts in holes.values():
		if len(cuts) == 2:
			web.edges.append(link(tuple(cuts)))
			web.tracks.append(g)
		else:
			pi = len(web.points)
			web.points.append(sum(web.points[c] for c in cuts) / len(cuts))
			web.edges.extend( link((c,pi))  for c in cuts )
			web.tracks.extend( [g] * len(cuts) )
	
@bevel.register(Web)
def web_bevel(obj, points, cutter, resolution=None):
	holes = web_multicut(obj, points, cutter)

	conn = connpe(obj.edges)
	pts = obj.points
	div = 6
	
	g = len(obj.groups)
	obj.groups.append(None)
	
	def tangentat(p):
		''' find the tangent axis to the given cut point, 
			the third element is whether the original edge direction is opposite to the given tangent 
		'''
		e = obj.edges[next(conn[p])]
		if e[0] == p:	return pts[p], normalize(pts[p] - pts[e[1]]), True
		else:			return pts[p], normalize(pts[p] - pts[e[0]]), False
	
	for cuts in holes.values():
		if len(cuts) == 2:
			# place an arc between the two ends
			s0 = tangentat(cuts[0])
			s1 = tangentat(cuts[1])
			corner = web(tangentarc(s0[:2], s1[:2], resolution))
			if s0[2]:	corner = corner.flip()
			obj += corner
			
		else:
			# find an approximate sphere center
			tangents = [tangentat(i)  for i in cuts]
			normal = normalize(sum(t[1]  for t in tangents))
			med = sum(pts[i]    for i in cuts) / len(cuts)
			center = sum(pts[i] + unproject(med-pts[i], normal)   for i in cuts)
			radius = sum(distance(pts[i], center)   for i in cuts)
			
			# place summit
			pi = len(pts)
			top = center + normal*radius
			pts.append(top)
			# place arcs
			for t in tangents:
				u = (top, noproject(t[0]-top, normal))
				corner = web(tangentarc(t[:2], u, resolution))
				if t[2]:	corner = corner.flip()
				obj += corner


				
# ---- wire operations -----

@multicut.register(Wire)
def wire_multicut(wire, points, cutter):
	if isinstance(points, Wire):	points = points.indices
	prec = wire.precision()
	
	cutter = interpretcutter(cutter)
	if not wire.tracks:
		wire.tracks = typedlist.full(0, len(wire.indices), 'I')
	g = len(wire.groups)
	wire.groups.append(None)
	
	cuts = []
	
	for origin in points:
		# get point location in the wire
		if origin == wire.indices[0] or origin == wire.indices[-1]:
			raise MeshError('a chamfer cannot have only one side')
		index = wire.indices.index(origin)
		
		# compute cut plane
		t0, t1 = normalize(wire[index] - wire[index-1]),  normalize(wire[index+1] - wire[index])
		axis = cross(t0, t1)
		offset = cutter(normalize(cross(axis,t0)), normalize(cross(axis,t1)))
		if dot(offset, t0) > 0:		offset = -offset
		cutplane = (wire[index]+offset, -normalize(offset))
	
		l = len(wire)
		start, end = 0, l
		ps, pe = None, None
		# propagate backward
		i = index-1
		while i >= 0:
			p0, p1 = wire[i], wire[i+1]
			p = intersection_edge_plane((p0, p1), cutplane, prec)
			if p:
				ps = p
				start = i+1
				break
			i -= 1
		# propagate forward
		i = index+1
		while i < l:
			p0, p1 = wire[i-1], wire[i]
			p = intersection_edge_plane((p0, p1), cutplane, prec)
			if p:
				pe = p
				end = i
				break
			i += 1
		# remove fragment
		m = len(wire.points)
		wire.points.append(ps)
		wire.points.append(pe)
		wire.indices[start:end] = [m,m+1]
		wire.tracks[start:end-1] = [g]
		
		cuts.append((m,m+1))  # point interval including start and excluding end
	return cuts
		
@chamfer.register(Wire)
def wire_chamfer(wire, points, cutter):
	wire_multicut(wire, points, cutter)

@bevel.register(Wire)
def wire_bevel(wire, points, cutter, resolution=None):
	cuts = set(wire_multicut(wire, points, cutter))
	g = len(wire.groups)-1
	wire.groups[g] = None
	
	i = 0
	while i < len(wire.indices)-2:
		i0, i1 = wire.indices[i], wire.indices[i+1]
		if (i0,i1) in cuts:
			p0 = wire.points[i0]
			p1 = wire.points[i1]
			t0 = normalize(p0 - wire[i-1])
			t1 = normalize(p1 - wire[i+2])
			l = len(wire.points)
			wire.points.extend( tangentarc((p0,t0), (p1,t1), resolution) )
			wire.indices[i:i+2] = range(l, len(wire.points))
			wire.tracks[i:i+1] = [g] * (len(wire.points)-l-1)
		i += 1


		
		
# --- mesh tangent generation functions ----

def tangentarc(s0, s1, resolution=None):
	''' create a tangent curve to both given axis. 
		return the curve as a list of points 
	'''
	p0, t0 = s0
	p1, t1 = s1
	z = cross(t0,t1)
	factor = arclength(p0, p1, cross(z,t0), cross(t1,z))
	s0 = (p0, factor*t0)
	s1 = (p1, factor*t1)
	div = settings.curve_resolution(factor, anglebt(t0,-t1), resolution)
	return [ interpol2(s0, s1, j/(div+1))   for j in range(div+2) ]

def tangentend(points, edge, normals, div):
	''' join a tangent surface resulting of `tangentcorner` or `tangentjunction` to a straight edge e 
		normals is the same dict as for tangentcorner and tangentjunction
	'''
	l, r = edge
	pl, pr = points[l], points[r]
	dist = arclength(points[l], points[r], normals[l], normals[r])
	e = points[r]-points[l]
	tl = normalize(noproject( e, normals[l])) * dist
	tr = normalize(noproject(-e, normals[r])) * dist
	
	def infos():
		for i in range(div+2):
			x = i/(div+1)
			yield interpol2((pl,tl), (pr,tr), x),  interpol1(pl,pr, x)
	return blenditer(infos(), 0, interpol1)
	

def tangentcorner(pts, lp, normals, div):
	''' create a rounded surface tangent to the loop given
		`normals` is a dict {point: normal}
	'''
	new = Mesh()
	# compute the common points to all triangular regions
	n = normalize(sum(normals[i] for i in lp))
	med = sum(pts[i] for i in lp) / len(lp)
	center = sum(pts[i] + unproject(med-pts[i], normals[i])	for i in lp) / len(lp)
	radius = sum(distance(pts[i], center) for i in lp) / len(lp)
	# place a central point
	p = len(pts)
	pts.append(center + n*radius)
	normals[p] = n
	# triangulate
	for i in range(len(lp)):
		a,b,c = lp[i-1], lp[i], p
		new += gt.icosurface(
					(pts[a],pts[b],pts[c]), 
					(normals[a],normals[b],normals[c]),
					resolution=('div',div))
	return new

def tangentjunction(points, match, normals, div):
	''' create a surface joining the given couples of points, tangent to the two sides
		`normals` is a dict {point: normal}
	'''
	def infos():
		# estimate normals
		for l,r in match:
			dist = arclength(points[l], points[r], normals[l], normals[r])
			e = points[r]-points[l]
			tl = normalize(noproject( e, normals[l])) * dist
			tr = normalize(noproject(-e, normals[r])) * dist
			yield (points[r], tr), (points[l], tl)
	
	return blenditer(infos(), div, interpol2)
