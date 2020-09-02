# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import vec3, mat3, dmat3, dvec3, dot, cross, project, noproject, anglebt, sqrt, cos, acos, asin, spline, interpol1, normalize, distance, length, inverse, transpose, NUMPREC, COMPREC
from .mesh import Mesh, Web, Wire, lineedges, connef, connpp, edgekey, suites, line_simplification
from . import generation as gt
from . import text
from . import settings

__all__ = [	'chamfer', 'bevel', 'beveltgt',
			'cut', 'multicut', 'planeoffsets',
			'cutter_width', 'cutter_distance', 'cutter_depth', 'cutter_angle',
			]

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

def cutter_angle(depth, fn1, fn2):
	''' plane offset for a cut based on the angle between faces '''
	s = dot(fn1,n)
	return -depth/2 * (1/s - s) * n


# ----- algorithm ------
from nprint import nprint

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

		
def cut(mesh, start, cutplane, stops, conn, prec, removal, cutghost):
	''' propagation cut for an edge '''
	pts = mesh.points
	stops = list(filter(lambda e:e, stops))
	# find the intersections axis between the planes
	axis = [intersection_plane_plane(cutplane, stop)	for stop in stops]
	
	# prepare propagation
	if isinstance(start, tuple):
		front = [start, (start[1],start[0])]
	elif isinstance(start, int):
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
		
		# find the intersection of the triangle with the common axis to the two cutplanes (current and next)
		fpts = mesh.facepoints(fi)
		p = None
		for a in axis:
			p = intersection_axis_face(a, fpts)
			if p:	break
		if p and distance(p, pts[f[0]]) > prec and distance(p, pts[f[1]]) > prec and distance(p, pts[f[2]]) > prec:
			# mark cutplane change
			# empty faces can be generated, but they are necessary to keep the connectivity working
			pi = mesh.usepointat(p, prec)
			l = len(mesh.faces)
			unregisterface(mesh, conn, fi)				
			mesh.faces[fi] = (f[0], f[1], pi)
			mesh.faces.append((f[1], f[2], pi))
			mesh.faces.append((f[2], f[0], pi))
			mesh.tracks.append(mesh.tracks[fi])
			mesh.tracks.append(mesh.tracks[fi])
			registerface(mesh, conn, fi)
			registerface(mesh, conn, l)
			registerface(mesh, conn, l+1)
			front.append(frontedge)
		else:
			# mark this face as processed
			seen.add(fi)
			
			#scn3D.add(text.Text(
				#(pts[f[0]] + pts[f[1]] + pts[f[2]]) /3,
				#'  '+str(fi),
				#8,
				#color=(0.1, 1, 0.4),
				#align=('left', 'center'),
				#))
			
			# point side for propagation
			goodside = [False]*3
			for j,pi in enumerate(f):
				goodside[j] = 	dot(pts[pi]-cutplane[0], cutplane[1]) > -prec
			goodx = [False]*3
			for j,pi in enumerate(f):
				goodx[j] = all(dot(pts[pi]-stop[0], stop[1]) >= -prec  	for stop in stops)
			
			if not any(goodside):
				continue
			
			# intersections of triangle's edges with the plane
			cutted = False
			ghostface = fi in removal and cutghost
			cut = [None]*3
			for j,e in enumerate(((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))):
				cut[j] = intersection_edge_plane(cutplane, (pts[e[0]], pts[e[1]]), prec)
				
			for j in range(3):
				if cut[j-1] and cut[j] and distance(cut[j-1], cut[j]) < prec:	
					cut[j] = None
		
			if all(goodside) and any(goodx):
				removal.add(fi)
			
			# cut the face
			for j in range(3):
				if cut[j] and cut[j-1]:
					cutted = True
					if ghostface:	break	# NOTE: this is a ugly trick to make the current multicut implementation work with the same function for corners and edges
					# cut only if the intersection segment is in the delimited area
					if any(	dot(cut[j-1]-stop[0], stop[1]) < prec 
						and	dot(cut[j]  -stop[0], stop[1]) < prec
							for stop in stops):
						continue
					
					# cut the face (create face even for non kept side, necessary for propagation)
					p1 = mesh.usepointat(cut[j], prec)
					p2 = mesh.usepointat(cut[j-1], prec)
					unregisterface(mesh, conn, fi)
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
					removal.discard(fi)
					# remove the faces outside
					if dot(pts[f[j]]-cutplane[0], cutplane[1]) < 0:
						removal.add(fi)
						removal.add(l)
						intersections.add((p2,p1))
					else:
						removal.add(l+1)
						intersections.add((p1,p2))
					break
			# propagate if the face has been cutted or has a corner in the area
			if cutted or (goodside[0] and goodx[0] or goodside[1] and goodx[1] or goodside[2] and goodx[2]):
				for j in range(3):
					front.append((f[j],f[j-1]))
	
	return intersections


	
def multicut(mesh, edges, cutter, conn=None, prec=None, removal=None):
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
			n = cross(pts[i]-axis[0], axis[1])
			if dot(n, pts[prox[0]]-pts[i]) < 0:		n = -n
			separators[(i,prox[0])] = (axis[0], n)
			separators[(i,prox[1])] = (axis[0],-n)
			'''
			# display cut planes in the mesh (debug)
			grp = len(debmesh.groups)
			debmesh.groups.append(None)
			m = len(debmesh.points)
			debmesh.points.append(pts[i])
			debmesh.points.append(axis[0]+axis[1])
			debmesh.points.append(axis[0]-axis[1])
			debmesh.faces.append((m,m+1,m+2))
			debmesh.tracks.append(grp)
			'''
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
		outlines[edge] = cut(mesh, edge, 
								(pts[edge[0]]+offset, -normalize(offset)), 
								(separators.get(edge), separators.get((edge[1], edge[0]))), 
								conn, prec, removal, False)
	# couper les sommets
	for corner,offset in corners.items():
		outlines[corner] = cut(mesh, corner, 
								(pts[corner]+offset, -normalize(offset)), 
								(),
								conn, prec, removal, True)
	if final:
		finalize(mesh, outlines, removal, prec)
	return outlines

def finalize(mesh, outlines, removal, prec):
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

def intersection_edge_plane(axis, edge, prec):
	''' return intersection point of an edge with a plane, or None if it doesn't exist '''
	dist = dot(axis[0]-edge[0], axis[1])
	compl = dot(axis[0]-edge[1], axis[1])
	if abs(dist) <= prec:		return edge[0]
	if abs(compl) <= prec:	return edge[1]
	if dist * compl > 0:	return None		# the segment doesn't reach the plane
	edgedir = edge[1]-edge[0]
	if abs(dot(edgedir, axis[1])) <= NUMPREC:	return None	# the segment is parallel to the plane
	edgedir = normalize(edgedir)
	return edge[0] + dist * edgedir / dot(edgedir, axis[1])

def intersection_axis_face(axis, face):
	''' return the intersection point between an axis and a triangle, or None if it doesn't exist '''
	coords = inverse(dmat3(face[1]-face[0], face[2]-face[0], axis[1])) * dvec3(axis[0] - face[0])
	if 0 < coords[0] and 0 < coords[1] and coords[0]+coords[1] < 1 :
		#return axis[0] - axis[1]*coords[2]	# can lead to huge precision loss when the triangle is thin
		return face[0] + coords[0]*(face[1]-face[0]) + coords[1]*(face[2]-face[0])
	else:
		return None


def removefaces(mesh, crit):
	''' remove faces whose indices are present in faces, (for huge amount, prefer pass faces as a set) '''
	newfaces = []
	newtracks = []
	for i in range(len(mesh.faces)):
		if not crit(i):
			newfaces.append(mesh.faces[i])
			newtracks.append(mesh.tracks[i])
	mesh.faces = newfaces
	mesh.tracks = newtracks

#def facesurf(mesh, fi):
	#o,x,y = mesh.facepoints(fi)
	#return length(cross(x-o, y-o))

#def faceangle(mesh, fi):
	#o,x,y = mesh.facepoints(fi)
	#return length(cross(normalize(x-o), normalize(y-o)))
	
from math import inf
def faceheight(mesh, fi):
	f = mesh.facepoints(fi)
	m = inf
	for i in range(3):
		a,b = f[i]-f[i-1], f[i]-f[i-2]
		if not dot(b,b):	return 0
		m = min(m, dot(a,a) - dot(a,b)**2/dot(b,b))
	return sqrt(max(0,m))


# ----- user functions ------

from .triangulation import triangulation_outline
from .generation import icosurface, junctioniter, curvematch

def chamfer(mesh, edges, cutter):
	''' create a chamfer on the given suite of points, create faces are planes.
		cutter is described in function planeoffsets()
	'''
	# cut faces
	segments = multicut(mesh, edges, cutter)
	
	# create junctions
	group = len(mesh.groups)
	mesh.groups.append('junction')
	juncs = {}
	for e,s in segments.items():
		if not s:	continue
		# assemble the intersection segments in a single outline
		lines = suites(s)
		if len(lines) == 1:		
			lp = lines[0]
			juncs[lp[0]] = lp[-1]
		elif len(lines) == 2:	
			lp = lines[0] + lines[1]
			juncs[lines[0][0]] = lines[1][-1]
			juncs[lines[1][0]] = lines[0][-1]
		else:
			lp = lines.pop()
			lines = {l[0]: l for l in lines}
			while juncs[lp[-1]] != lp[0]:
				lp.extend(lines[juncs[lp[-1]]])
		# triangulate
		faces = triangulation_outline(Wire(mesh.points, lp)).faces
		mesh.faces.extend(faces)
		mesh.tracks.extend([group]*len(faces))

from nprint import nprint, nformat

def bevel(mesh, edges, cutter, resolution=None):
	''' create a chamfer on the given suite of points, create faces are planes.
		cutter is described in function planeoffsets()
	'''
	edges = [edgekey(*e)	for e in edges]
	conn = connef(mesh.faces)
	enormals = {}
	for e in edges:
		enormals[e] = (
			mesh.facenormal(conn[e]),
			mesh.facenormal(conn[(e[1],e[0])]),
			)
	# cut faces
	segments = multicut(mesh, edges, cutter, conn)
	conn = connef(mesh.faces)
	pts = mesh.points
	resolution = ('div', 4)
	
	# create junctions
	group = len(mesh.groups)
	juncs = {}
	tangents = {}	# tangent at each point
	normals = {}	# face normal at each point
	junctions = []
	corners = []
	for e,s in segments.items():
		if not s or isinstance(e, int):	continue
		# assemble the intersection segments in a single outline
		frags = suites(s)
		
		# one loop:  bevel extremity
		if len(frags) == 1:
			lp = frags[0]
			# find a separation to put a junction between 2 sides
			side = pts[lp[0]]-pts[lp[-1]]
			for i in range(1,len(lp)):
				if dot(pts[lp[i-1]]-pts[e[0]], side) * dot(pts[lp[i]]-pts[e[0]], side) < 0:
					left, right = lp[:i], lp[i:]
					break
		# two parts: cutted edge
		elif len(frags) == 2:
			left,right = frags
		
		# cutted edges (extremity or not)
		if len(frags) <= 2:
			# identify the two sides
			if dot(pts[e[1]]-pts[e[0]], pts[right[1]]-pts[right[0]]) < 0:
				left,right = right,left
			
			juncs[left[0]] = right[-1], (e[1],e[0])
			juncs[right[0]] = left[-1], e
			# match the curves
			right.reverse()
			match = list(curvematch(Wire(pts,left), Wire(pts,right)))
			# prepare tangents
			ll = Wire(pts,left).length()
			x = 0.
			for i in range(len(match)):
				if i:	x += distance(pts[match[i-1][0]], pts[match[i][0]]) / ll
				# get the tangents from the triangle between the cuts and the cutting edge
				l,r = match[i]
				o = interpol1(pts[e[1]], pts[e[0]], x)
				plane = cross(pts[r]-o, pts[l]-o)
				tangents[l] = normalize(cross(plane, enormals[e][0])) + tangents.get(l, 0)
				tangents[r] = normalize(cross(enormals[e][1], plane)) + tangents.get(r, 0)
			junctions.append(match)
		else:
			print('edge bevel issue', e, frags)
	
	# corner
	for e,s in segments.items():
		if not s or not isinstance(e,int):	continue
		# assemble the intersection segments in a single outline
		frags = suites(s)
		
		# assemble a loop by merging cuts with juncs
		lp = frags.pop()
		frags = {l[0]: l for l in frags}	# line for each start
		holes = []	# holes in loop, there is junctions there
		while True:
			step = juncs[lp[-1]]
			holes.append((lp[-1], *step))
			if step[0] == lp[0]:	break
			last = frags[step[0]][-1]
			lp.extend(frags[step[0]])
		# prepare normals
		for i in range(1,len(lp)):
			f = conn.get((lp[i], lp[i-1]))
			if not f:	continue
			n = mesh.facenormal(f)
			normals[lp[i]] = normals.get(lp[i], 0) + n
			normals[lp[i-1]] = normals.get(lp[i-1], 0) + n
		for k,n in normals.items():
			normals[k] = normalize(n)
		# set neighboring tangents
		for a,b, edge in holes:
			tangents[a] = normalize(cross(normals[a], pts[edge[1]]-pts[edge[0]]))
			tangents[b] = normalize(cross(normals[b], pts[edge[0]]-pts[edge[1]]))
		corners.append(lp)
	
	new = Mesh()					
	for match in junctions:
		# put a junction
		new += tangentjunction(pts, match, tangents, resolution[1], interpol2)
	for lp in corners:
		# compute the common points to all triangular regions
		p = len(pts)
		center = sum(pts[i] for i in lp) / len(lp)
		normals[p] = n = normalize(sum(normals[i] for i in lp))
		radius = sum(dot(pts[i]-center, normals[i]) for i in lp) / len(lp)/2
		pts.append(center + n*radius)
		# triangulate
		for i in range(len(lp)):
			a,b,c = lp[i-1], lp[i], p
			new += icosurface(
						(pts[a],pts[b],pts[c]), 
						(normals[a],normals[b],normals[c]),
						resolution=resolution)
	
	new.groups = ['junction']
	new.tracks = [0] * len(new.faces)
	mesh += new
	mesh.mergeclose()

from functools import reduce
from .mathutils import interpol2, reflect
				
def beveltgt(mesh, line, cutter, interpol=spline, resolution=None):
	''' create a round profile on the given suite of points, create faces form cylindric surfaces.
		tangents to cuted faces will be used for interpolation
		cutter is described in function planeoffsets()
		
		WARNING: the tangency to cuted faces can lead to weired geometries in case of concave shape
	'''
	# cut faces
	conn = connef(mesh.faces)
	segments = cut(mesh, line, planeoffsets(mesh, line, cutter), conn)
	conn = connef(mesh.faces)
	group = Mesh(groups=['junction'])
	tangents = {}
	tmatch = []
	
	for (a,b),s in zip(lineedges(line), segments):
		if not s:	continue
		
		dir = mesh.points[a] - mesh.points[b]
		
		lps = suites(s)
		if len(lps) == 1:
			mesh += gt.flatsurface(Wire(mesh.points, lps[0]))
		else:
			left,right = lps
			if dot(dir, mesh.points[left[1]] - mesh.points[left[0]]) > 0:
				left,right = right,left
			right.reverse()
			ll, lr = Wire(mesh.points, left).length(), Wire(mesh.points, right).length()
			match = list(gt.curvematch(Wire(mesh.points, left), Wire(mesh.points, right)))
			tmatch.extend(match)
			
			# create parameters for the round profiles
			x = 0
			for i in range(len(match)-1):
				e = (match[i+1][0], match[i][0])
				if e in conn:	tl = mesh.facenormal(conn[e])
				else:			tl = None
				e = (match[i][1], match[i+1][1])
				if e in conn:	tr = mesh.facenormal(conn[e])
				else:			tr = None
				x += distance(mesh.points[match[i+1][0]], mesh.points[match[i][0]]) / ll
				
				for j in (0,1):
					l,r = match[i+j]
					o = interpol1(mesh.points[a], mesh.points[b], x)
					plane = normalize(cross(mesh.points[r]-o, mesh.points[l]-o))
					if tl:	tangents[l] = normalize(cross(plane, tl)) + tangents.get(l, 0)
					if tr:	tangents[r] = normalize(cross(tr, plane)) + tangents.get(r, 0)
	
	mesh += tangentjunction(mesh.points, tmatch, tangents, resolution, interpol)
	

def isedge(o):
	return isinstance(o,tuple) and len(o) == 2

def tangentjunction(points, match, tangents, div, interpol=spline):
	''' create a surface between interpolated curves for each match '''
	# estimate normals
	for l,r in match:
		v = points[l]-points[r]
		a = anglebt(tangents[l], tangents[r])
		dist = distance(points[l], points[r]) / sqrt(2-2*cos(a)) * a
		tangents[l] = tl = normalize(tangents[l]) * dist
		tangents[r] = tr = normalize(tangents[r]) * dist
	
	return gt.junctioniter(
			( ((points[r], tangents[r]), (points[l], tangents[l])) for l,r in match),
			div, interpol)
