from mathutils import vec3, mat3, dot, cross, cos, acos, normalize, distance, length, inverse, transpose, NUMPREC, COMPREC
import generation
import text

def chamfer(mesh, line, depth):
	# get adjacent faces' normals to lines
	adjacents = ({}, {})	# (left adjacents, right adjacents)
	segts = segmentsdict(line)
	for f in mesh.faces:
		for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
			if e in segts:					adjacents[0][segts[e]] = mesh.facenormal(f)
			elif (e[1],e[0]) in segts:		adjacents[1][segts[(e[1],e[0])]] = mesh.facenormal(f)
	
	# compute offsets (displacement on depth)
	offsets = []
	for i in range(len(line)-1):
		fn1,fn2 = adjacents[0][i], adjacents[1][i]
		# TODO: utiliser une fonction parametre
		# depth
		#offset = -depth * normalize(fn1+fn2)
		# or distance
		offset = -depth * cross(normalize(cross(fn1,fn2)), fn1-fn2)
		# or width
		#offset = -depth * normalize(fn1+fn2) * dot(fn1,fn2)
		
		if dot(cross(fn1, fn2), mesh.points[line[i+1]]-mesh.points[line[i]]) < 0:
			offset = -offset
		offsets.append(offset)
	
	# cut faces
	segments = cut(mesh, line, offsets)
	# TODO: simplifier les aretes resultantes
	
	# create junctions
	group = len(mesh.groups)
	mesh.groups.append('junc')
	for i,s in enumerate(segments):
		if s:
			lps = generation.makeloops(s)
			if len(lps) == 2:
				try:	generation.triangulate(mesh, lps[0]+lps[1], group)
				except Exception as err:	print(err)
			else:
				print('segment', i, 'edges are', lps)
				print('   ', s)

def cut(mesh, line, offsets):
	toremove = set()		# faces to remove that match no replacements
	result = []				# intersection segments for each offset

	# compute cut planes and their intersections
	#grp = len(mesh.groups)
	#mesh.groups.append(None)
	segments = []	# planes intersections  (origin, plane normal, axis direction)
	for i in range(1,len(line)-1):
		n1, n2 = offsets[i-1], offsets[i]
		d = cross(n1, n2)
		p = mesh.points[line[i]]
		intersect = inverse(transpose(mat3(n1, n2, d))) * vec3(
						dot(p+n1, n1), 
						dot(p+n2, n2), 
						dot(p, d))
		n = normalize(cross(d, intersect-p))
		if dot(n, mesh.points[line[i]]-mesh.points[line[i-1]]) > 0:		n = -n
		if length(d) > NUMPREC:
			segments.append((intersect, n, d))
		else:
			segments.append(None)
		
		#l = len(mesh.points)
		#d = normalize(d)
		#mesh.points.append(p)
		#mesh.points.append(intersect+d)
		#mesh.points.append(intersect-d)
		#mesh.faces.append((l,l+1,l+2))
		#mesh.tracks.append(grp)
	# complete segments that cannot be computed locally (straight suite of points for example)
	for i in range(1,len(segments)):
		if i and not segments[i]:		segments[i] = segments[i-1]
	for i in reversed(range(len(segments)-1)):
		if i and not segments[i]:		segments[i] = segments[i+1]
	# build connectivity
	conn = connectivity_edgeface(mesh.faces)
	
	# cut at each plane
	for i in range(1,len(line)):
		# propagate until cut
		cutplane = (mesh.points[line[i-1]]+offsets[i-1], -normalize(offsets[i-1]))
		#print('cutplane', cutplane)
		s1 = segments[i-2] if i >= 2 else None
		s2 = segments[i-1] if i <= len(segments) else None
		seen = set()
		front = [(line[i],line[i-1]), (line[i-1],line[i])]
		intersections = set()
		while front:
			frontedge = front.pop()
			if frontedge not in conn:	continue
			fi = conn[frontedge]
			if fi in seen:	continue
			
			f = mesh.faces[fi]
			#print(fi, 'for', frontedge)
			# find the intersection of the triangle with the common axis to the two cutplanes (current and next)
			p = intersection_axis_face((s2[0], s2[2]), mesh.facepoints(fi)) if s2 else None
			if p:
				
				# mark cutplane change
				unregisterface(mesh, conn, fi)
				pi = insertpoint(mesh, p)
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
				'''
				# mark cutplane change
				pi = insertpoint(mesh, p)
				l = len(mesh.faces)
				parts = [(f[0], f[1], pi), (f[1], f[2], pi), (f[2], f[0], pi)]
				# remove empty faces
				j = 0
				while j<len(parts):
					o = mesh.points[parts[j][0]]
					x = mesh.points[parts[j][1]] - o
					y = mesh.points[parts[j][2]] - o
					if length(cross(x,y)) < NUMPREC:	parts.pop(j)
					else:						j += 1
				# insert remaining faces
				if parts:
					mesh.faces[fi] = parts.pop()
					registerface(mesh, conn, fi)
				else:
					toremove.add(fi)
				if parts:	
					mesh.faces.append(parts.pop())
					mesh.tracks.append(mesh.tracks[fi])
					registerface(mesh, conn, l)
				if parts:
					mesh.faces.append(parts.pop())
					mesh.tracks.append(mesh.tracks[fi])
					registerface(mesh, conn, l+1)
				front.append(frontedge)
				'''
			else:
				# mark this face as processed
				seen.add(fi)
				
				#scn3D.add(text.Text(
					#(mesh.points[f[0]] + mesh.points[f[1]] + mesh.points[f[2]]) /3,
					#'  '+str(fi),
					#8,
					#color=(0.1, 1, 0.4),
					#align=('left', 'center'),
					#))
				
				# point side for propagation
				goodside = [False]*3
				for j,pi in enumerate(f):
					#goodside[j] = 	dot(mesh.points[pi]-cutplane[0], cutplane[1]) > NUMPREC 
					goodside[j] = 	dot(mesh.points[pi]-cutplane[0], cutplane[1]) > -NUMPREC
				goodx = [False]*3
				for j,pi in enumerate(f):
					p = mesh.points[pi]
					goodx[j] = ( (not s1 or dot(p-s1[0], -s1[1]) >= 0)
							 and (not s2 or dot(p-s2[0],  s2[1]) >= 0) )
				
				if goodside[0] or goodside[1] or goodside[2]:
					for j in range(3):
						front.append((f[j],f[j-1]))
				else:
					continue
				
				# intersections of triangle's edges with the plane
				cut = [None]*3
				for j,e in enumerate(((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))):
					cut[j] = intersection_edge_plane(cutplane, (mesh.points[e[0]], mesh.points[e[1]]))
				for j in range(3):
					if cut[j-1] and cut[j] and distance(cut[j-1], cut[j]) < NUMPREC:	cut[j] = None
				
				if goodside[0] and goodside[1] and goodside[2] and (goodx[0] or goodx[1] or goodx[2]):
					toremove.add(fi)
				
				# cut the face
				cutted = False
				for j in range(3):
					if cut[j] and cut[j-1]:
						cutted = True
						# cut only if the intersection segment is in the delimited area
						if s1 and dot(cut[j-1]-s1[0],-s1[1]) < NUMPREC and dot(cut[j]-s1[0],-s1[1]) < NUMPREC:	continue
						if s2 and dot(cut[j-1]-s2[0], s2[1]) < NUMPREC and dot(cut[j]-s2[0], s2[1]) < NUMPREC:	continue
						'''
						toremove.discard(fi)
						f = mesh.faces[fi]
						p1 = insertpoint(mesh, cut[j])
						p2 = insertpoint(mesh, cut[j-1])
						unregisterface(mesh, conn, fi)
						# add only the face that are outside the region delimited by planes
						if dot(mesh.points[f[j]]-cutplane[0], cutplane[1]) < 0 :
							mesh.faces[fi] = (p1, p2, f[j-0])
							registerface(mesh, conn, fi)
							seen.add(fi)
							intersections.add((p2,p1))
						else:
							l = len(mesh.faces)
							mesh.faces[fi] = (p1, f[j-2], f[j-1])
							mesh.faces.append((p1, f[j-1], p2))
							mesh.tracks.append(mesh.tracks[fi])
							registerface(mesh, conn, fi)
							registerface(mesh, conn, l)
							seen.update((fi, l))
							intersections.add((p1,p2))
						'''
						toremove.discard(fi)
						f = mesh.faces[fi]
						p1 = insertpoint(mesh, cut[j])
						p2 = insertpoint(mesh, cut[j-1])
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
						if dot(mesh.points[f[j]]-cutplane[0], cutplane[1]) < 0:
							toremove.add(fi)
							toremove.add(l)
							intersections.add((p2,p1))
						else:
							toremove.add(l+1)
							intersections.add((p1,p2))
						break
		result.append(intersections)
	# delete faces
	removefaces(mesh, toremove)
	
	return result

def insertpoint(mesh, pt):
	return mesh.usepointat(pt)

def connectivity_edgeface(faces):
	conn = {}
	for i,f in enumerate(faces):
		for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
			conn[e] = i
	return conn

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

# return intersection of an edge with a plane, or None
def intersection_edge_plane(axis, edge):
	dist = dot(axis[0]-edge[0], axis[1])
	compl = dot(axis[0]-edge[1], axis[1])
	if abs(dist) < NUMPREC:		return edge[0]
	if abs(compl) < NUMPREC:	return edge[1]
	if dist * compl > 0:	return None		# the segment doesn't reach the plane
	edgedir = edge[1]-edge[0]
	if abs(dot(edgedir, axis[1])) < NUMPREC:	return None	# the segment is parallel to the plane
	edgedir = normalize(edgedir)
	return edge[0] + dist * edgedir / dot(edgedir, axis[1])


def intersection_axis_face(axis, face):
	coords = inverse(mat3(face[1]-face[0], face[2]-face[0], axis[1])) * (axis[0] - face[0])
	if 0 <= coords[0] and 0 <= coords[1] and coords[0]+coords[1] <= 1 and (notint(coords[0]) or notint(coords[1])):
		#print('  coords', coords, notint(coords[0]), notint(coords[1]))
		return axis[0] - axis[1]*coords[2]
	else:
		return None
	
def notint(x):
	return x > 1e-5 and x < 1-1e-4

def removefaces(mesh, faces):
	''' remove faces whose indices are present in faces, (for huge amount, prefer pass faces as a set) '''
	newfaces = []
	newtracks = []
	for i in range(len(mesh.faces)):
		if i not in faces:
			newfaces.append(mesh.faces[i])
			newtracks.append(mesh.tracks[i])
	mesh.faces = newfaces
	mesh.tracks = newtracks


if __name__ == '__main__':
	
	# test intersections
	from generation import saddle, tube, outline, Outline, makeloops
	from primitives import Arc
	import sys
	import view, text
	from PyQt5.QtWidgets import QApplication
	from nprint import nprint
	from copy import deepcopy
	
	app = QApplication(sys.argv)
	main = scn3D = view.Scene()
	
	m = saddle(
			outline([vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], [0,1,2,3]),
			#outline([vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)]),
			#outline(Arc(vec3(0,1,-1),vec3(0,1.5,0),vec3(0,1,1))),
			outline([Arc(vec3(0,1,-1),vec3(0,1.3,-0.5),vec3(0,1,0)), Arc(vec3(0,1,0),vec3(0,0.7,0.5),vec3(0,1,1))]),
			)
	#m.options.update({'debug_display': True, 'debug_points': True })
	
	line = makeloops(list(m.group(1).outlines_unoriented() & m.group(2).outlines_unoriented()))[0]
	print('line', line)
	chamfer(m, line, 0.6)
	
	scn3D.add(m)
	
	main.show()
	sys.exit(app.exec())
