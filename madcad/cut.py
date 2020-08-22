# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import vec3, mat3, dmat3, dvec3, dot, cross, project, noproject, anglebt, sqrt, cos, acos, asin, spline, interpol1, normalize, distance, length, inverse, transpose, NUMPREC, COMPREC
from .mesh import Mesh, Web, Wire, lineedges, connef, connpp, edgekey, suites, line_simplification
from . import generation as gt
from . import text
from . import settings

__all__ = [	'chamfer', 'bevel', 'beveltgt', 'tangentjunction', 
			'cut', 'separators', 'planeoffsets',
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

# ----- user functions ------

def chamfer(mesh, line, cutter):
	''' create a chamfer on the given suite of points, create faces are planes.
		cutter is described in function planeoffsets()
	'''
	# cut faces
	segments = cut(mesh, line, planeoffsets(mesh, line, cutter))
	
	# create junctions
	group = len(mesh.groups)
	mesh.groups.append('junction')
	for i,s in enumerate(segments):
		if s:
			lp = []
			for part in suites(s):
				lp.extend(part)
			faces = gt.flatsurface(Wire(mesh.points, lp)).faces
			mesh.faces.extend(faces)
			mesh.tracks.extend([group]*len(faces))

def chamfer(mesh, edges, cutter):
	''' create a chamfer on the given suite of points, create faces are planes.
		cutter is described in function planeoffsets()
	'''
	normals = mesh.vertexnormals()
	corners = {}
	offs = []
	lines = suites(edges)
	for line in lines:
		offsets = planeoffsets(mesh, line, cutter)
		offs.append(offsets)
		corners[line[0]]  = min(corners.get(line[0], 0), dot(offsets[0], offsets[0]) /dot(offsets[0],  normals[line[0]]))
		corners[line[-1]] = min(corners.get(line[-1],0), dot(offsets[-1],offsets[-1])/dot(offsets[-1], normals[line[-1]]))
	# cut faces
	segments = []
	for point,offset in corners.items():
		segments.extend(cut_corner(mesh, point, offset*normals[point]))
	for line, offsets in zip(lines, offs):
		segments.extend(cut(mesh, line, offsets))
	
	# create junctions
	#group = len(mesh.groups)
	#mesh.groups.append('junction')
	#for i,s in enumerate(segments):
		#if s:
			#lp = []
			#for part in suites(s):
				#lp.extend(part)
			#faces = gt.flatsurface(Wire(mesh.points, lp)).faces
			#mesh.faces.extend(faces)
			#mesh.tracks.extend([group]*len(faces))

				
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
			
def bevel(mesh, line, cutter, interpol=spline, resolution=None):
	''' create a smooth interpolated profile at the given suite of points
		tangents to line's adjacents faces will be used for interpolation
		cutter is described in function planeoffsets()
		
		WARNING: to use tangents from line adjacents faces can lead to matter add in case of concave shape
	'''
	# cut faces
	conn = connef(mesh.faces)
	normals = []
	for e in lineedges(line):
		normals.append((
			mesh.facenormal(conn[(e[1],e[0])]),
			mesh.facenormal(conn[e]),
			))
	
	segments = cut(mesh, line, planeoffsets(mesh, line, cutter), conn)
	tangents = {}
	tmatch = []
	
	for (a,b),s,(nl,nr) in zip(lineedges(line), segments, normals):
		if not s:	continue
		
		lps = suites(s)
		if len(lps) == 1:
			# TODO
			pass
			#mesh += gt.flatsurface(Wire(mesh.points, lps[0]))
		elif len(lps) == 2:
			# match left and right lines
			left,right = lps
			if dot(mesh.points[a] - mesh.points[b], mesh.points[left[1]] - mesh.points[left[0]]) > 0:
				left,right = right,left
			right.reverse()
			match = list(gt.curvematch(Wire(mesh.points, left), 
									   Wire(mesh.points, right) ))
			tmatch.extend(match)
			
			# create parameters for the round profiles
			ll, lr = Wire(mesh.points, left).length(), Wire(mesh.points, right).length()
			x = 0.
			for i in range(len(match)):
				if i:	x += distance(mesh.points[match[i-1][0]], mesh.points[match[i][0]]) / ll
				l,r = match[i]
				o = interpol1(mesh.points[a], mesh.points[b], x)
				plane = normalize(cross(mesh.points[r]-o, mesh.points[l]-o))
				tangents[l] = normalize(cross(plane, nl)) + tangents.get(l, 0)
				tangents[r] = normalize(cross(nr, plane)) + tangents.get(r, 0)
		else:
			print('error in bevel: bevel sides are', lps)
	
	mesh += tangentjunction(mesh.points, tmatch, tangents, resolution, interpol)

def tangentjunction(points, match, tangents, resolution=None, interpol=spline):
	''' create a surface between interpolated curves for each match '''
	group = Mesh(groups=['junction'])
	# determine the number of segments
	div = 0
	for l,r in match:
		v = points[l]-points[r]
		a = anglebt(tangents[l], tangents[r])
		dist = distance(points[l], points[r]) / sqrt(2-2*cos(a)) * a
		tangents[l] = tl = normalize(tangents[l]) * dist
		tangents[r] = tr = normalize(tangents[r]) * dist
		angle = min(1, acos(dot(tl,tr)))
		dist = length(points[l]-points[r])
		ldiv = settings.curve_resolution(dist, angle, resolution)
		if ldiv > div:
			div = ldiv
	
	return gt.junctioniter(
			( ((points[r], tangents[r]), (points[l], tangents[l])) for l,r in match),
			div, interpol)


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
	cutter = interpretcutter(cutter)
	# get adjacent faces' normals to lines
	offsets = {edgekey(*e): [None,None]  for e in edges}
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

def separators(mesh, line, offsets):
	''' separations between planes.
		planes are determined by the offsets to the lines
		segments are in format (origin, segt normal, axis direction)
	'''
	
	segments = []	# planes intersections  (origin, plane normal, axis direction)
	l = len(line)-1
	# there is an additional cut segment if the line is a loop
	for i in range(1, l if line[0]!=line[-1] else l+1):
		n1, n2 = offsets[(i-1)%l], offsets[i%l]
		# compute the intersection with the two neigh cut planes
		d = cross(n1, n2)
		if length(d) > mesh.precision():
			d = normalize(d)
			p = mesh.points[line[i]]
			intersect = vec3(inverse(transpose(dmat3(n1, n2, d))) * dvec3(
							dot(p+n1, n1), 
							dot(p+n2, n2), 
							dot(p, d)))
			n = normalize(cross(d, intersect-p))
			if dot(n, mesh.points[line[i]]-mesh.points[line[i-1]]) > 0:		n = -n
			segments.append((intersect, n, d))
		else:
			segments.append(None)
		'''
		# display cut planes in the mesh (debug)
		grp = len(debmesh.groups)
		debmesh.groups.append(None)
		m = len(debmesh.points)
		d = normalize(d)
		debmesh.points.append(p)
		debmesh.points.append(intersect+d)
		debmesh.points.append(intersect-d)
		debmesh.faces.append((m,m+1,m+2))
		debmesh.tracks.append(grp)
		'''
	
	# complete segments that cannot be computed locally (straight suite of points for example)
	for i in range(1,len(segments)):
		if not segments[i]:		segments[i] = segments[i-1]
	for i in reversed(range(len(segments)-1)):
		if not segments[i]:		segments[i] = segments[i+1]
	
	return segments


def cut_line(mesh, line, offsets, conn=None, prec=None):
	''' cut the mesh faces by planes, determined by the offsets to the lines '''
	
	removal = set()		# faces to remove that match no replacements
	result = []				# intersection segments for each offset
	if prec is None:	prec = mesh.precision()
	if conn is None:	conn = connef(mesh.faces)
	# segments planes and normals
	segments = separators(mesh, line, offsets)
	circular = line[0] == line[-1]
	
	# cut at each plane
	for i in range(1,len(edge)):
		# take segment limiting planes
		if circular:
			s1 = segments[i-2]
			s2 = segments[i-1]
		else:
			s1 = segments[i-2] if i >= 2 else None
			s2 = segments[i-1] if i <= len(segments) else None
			
		# propagate until cut
		results.append(cut_edge(mesh, edge, offsets[i-1], s1, s2, conn, prec))
			
	# simplify cuts
	merges = {}
	for i,grp in enumerate(result):
		reindex = line_simplification(Web(mesh.points, grp), prec)
		lines = []
		for a,b in grp:
			c,d = reindex.get(a,a), reindex.get(b,b)
			if c != d:	lines.append((c,d))
		result[i] = lines
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
	
	return result

		
def cut_edge(mesh, edge, offset, s1, s2, conn, prec, removal):
	''' propagation cut for an edge '''
	pts = mesh.points
	cutplane = (pts[edge[0]]+offset, -normalize(offset))
	# find the intersections axis between the planes
	i1 = intersection_plane_plane(cutplane, s1)	if s1 else None
	i2 = intersection_plane_plane(cutplane, s2)	if s2 else None
	
	# prepare propagation
	seen = set()
	front = [edge, (edge[1],edge[0])]
	intersections = set()
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
		p = None
		if s2:				p = intersection_axis_face(i2, mesh.facepoints(fi))
		if s1 and not p:	p = intersection_axis_face(i1, mesh.facepoints(fi))
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
				p = pts[pi]
				goodx[j] = ( (not s1 or dot(p-s1[0], s1[1]) >= -prec)
							and (not s2 or dot(p-s2[0],  s2[1]) >= -prec) )
			
			if not (goodside[0] or goodside[1] or goodside[2]):
				continue
			
			# intersections of triangle's edges with the plane
			cut = [None]*3
			for j,e in enumerate(((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))):
				cut[j] = intersection_edge_plane(cutplane, (pts[e[0]], pts[e[1]]), prec)
			for j in range(3):
				if cut[j-1] and cut[j] and distance(cut[j-1], cut[j]) < prec:	cut[j] = None
			
			if goodside[0] and goodside[1] and goodside[2] and (goodx[0] or goodx[1] or goodx[2]):
				removal.add(fi)
			
			cutted = False
			# cut the face
			for j in range(3):
				if cut[j] and cut[j-1]:
					cutted = True
					# cut only if the intersection segment is in the delimited area
					if s1 and dot(cut[j-1]-s1[0], s1[1]) < prec and dot(cut[j]-s1[0], s1[1]) < prec:	continue
					if s2 and dot(cut[j-1]-s2[0], s2[1]) < prec and dot(cut[j]-s2[0], s2[1]) < prec:	continue
					
					# cut the face (create face even for non kept side, necessary for propagation)
					removal.discard(fi)
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
	
def cut_corner(mesh, point, offset, conn, prec, removal):
	''' cut a mesh by the plane at an offset from the specified point (existing point in the mesh) '''
	pts = mesh.points
	cutplane = (pts[point]+offset, -normalize(offset))
	
	# prepare propagation
	toremove = {}
	intersections = []
	edge = next(filter(lambda e: point in e, conn))
	seen = set()
	front = [edge, (edge[1], edge[0])]
	while front:
		# unstack
		edge = front.pop()
		if edge in seen:		continue
		seen.add(edge)
		if edge not in conn:	
			front.append((edge[1], edge[0]))
			continue
		
		fi = conn[edge]
		f = mesh.faces[fi]
		a,b = edge
			
		# find intersection with the offset plane
		pt = intersection_edge_plane(cutplane, (pts[a], pts[b]), prec)
		if pt and distance(pt, pts[a]) > prec and distance(pt, pts[b]) > prec:
			pi = mesh.usepointat(pt)
			c = None
			for c in f:
				if c != a and c != b:	break
			
			# cut face
			mesh.faces[fi] = (a,pi,c)
			mesh.faces.append((pi,b,c))
			mesh.tracks.append(mesh.tracks[fi])
			registerface(mesh, conn, fi)
			registerface(mesh, conn, len(mesh.faces)-1)
			
			# register intersection and the face to remove
			r,k = len(mesh.faces)-1, fi
			if (	dot(pts[a]-cutplane[0], cutplane[1]) > prec 
				or	dot(pts[b]-cutplane[0], cutplane[1]) < -prec):
				r,k = k,r
			if fi in toremove:	intersections.append((toremove[fi], pi))
			if k in toremove:	del toremove[k]
			toremove[r] = pi
		else:
			# faces on the way but not intersected are inside
			toremove[fi] = None
		
		# propagate
		goodside = [dot(pts[f[i]]-cutplane[0], cutplane[1]) > prec		for i in range(3)]
		for i in range(3):
			if goodside[i-1] or goodside[i]:
				front.append((f[i], f[i-1]))
	
	# delete inside faces and empty ones
	#removefaces(mesh, removal.__contains__)
	removal.update(toremove)
	return intersections

def cut(mesh, edges, offsets, conn=None, prec=None, removal=None):
	''' general purpose edge cutting function.
		cut the given edges and the crossing corners, resolving the interference issues.
		
		offsets is `{edgekey: offset vector}` for each passed edge
	'''
	# perpare working objects
	if conn is None:	conn = connef(mesh.faces)
	if prec is None:	prec = mesh.precision()
	if removal is None:	
		removal = set()
		final = True
	else:
		final = False
	
	normals = mesh.vertexnormals()
	juncconn = connpp(edges)
	pts = mesh.points
	nprint('offsets', offsets)
	
	debmesh = mesh
	separators = {}	# plans separateurs pour chaque arete
	for i, prox in juncconn.items():
		if len(prox) != 2:	continue
		o0 = offsets[edgekey(i,prox[0])]
		o1 = offsets[edgekey(i,prox[1])]
		axis = intersection_plane_plane((pts[prox[0]]+o0, o0), (pts[prox[1]]+o1, o1))	# BUG: fail when the two planes have the same orientation
		if axis:
			n = cross(axis[1], pts[i]-axis[0])
			separators[(i,prox[0])] = (axis[0], n)
			separators[(i,prox[1])] = (axis[0],-n)
		
			# display cut planes in the mesh (debug)
			grp = len(debmesh.groups)
			debmesh.groups.append(None)
			m = len(debmesh.points)
			debmesh.points.append(pts[i])
			debmesh.points.append(axis[0]+axis[1])
			debmesh.points.append(axis[0]-axis[1])
			debmesh.faces.append((m,m+1,m+2))
			debmesh.tracks.append(grp)
	
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
				ab = pts[b]-pts[a]
				ac = pts[c]-pts[a]
				n = cross(ab, ac)
				nb = normalize(cross(pts[b]-pts[a], n))
				nc = normalize(cross(pts[c]-pts[a], n))
				ib = dot(nb,nb) / dot(offsets[edgekey(a,b)], nb)
				ic = dot(nc,nc) / dot(offsets[edgekey(a,c)], nc)
				s = dot(nb,ac)/length(ac)
				if not s:	continue
				ai = (ib*dot(ab,ac) + ic) / s
				offset = max(offset, ai * dot(normalize(ab), normals[a]))
		# assignation du plan de coupe
		corners[junc] = -offset*normals[junc]
		plane = (pts[junc] + corners[junc], -normals[junc])
		for p in prox:
			separators[(p,junc)] = plane
	#nprint('separators', separators)
	#nprint('corners', corners)
	
	outlines = []
	# couper les aretes
	for edge in offsets:
		outlines += cut_edge(mesh, edge, offsets[edge], separators.get(edge), separators.get((edge[1], edge[0])), 
								conn, prec, removal)
	# couper les sommets
	for corner,offset in corners.items():
		outlines += cut_corner(mesh, corner, offset, 
								conn, prec, removal)
	if final:
		removefaces(mesh, removal.__contains__)
	return outlines


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

def intersection_plane_plane(p0, p1):
	''' return the intersection axis between two planes '''
	d = cross(p0[1], p1[1])
	if length(d) <= NUMPREC:	return None
	return vec3(
		inverse(transpose(dmat3(p0[1], p1[1], d))) 
		* dvec3(dot(p0[0],p0[1]), dot(p1[0],p1[1]), dot(p0[0],d))
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
	
def faceheight(mesh, fi):
	f = mesh.facepoints(fi)
	heights = [length(noproject(f[i-2]-f[i], normalize(f[i-1]-f[i])))	for i in range(3)]
	return min(heights)


