'''	
	Defines boolean operations for triangular meshes
'''
from copy import copy,deepcopy
from mathutils import vec3, mat3, dot, cross, inverse, normalize, length, distance, NUMPREC, COMPREC
from mesh import Mesh, edgekey

__all__ = ['intersect', 'boolean', 'intersection', 'union', 'difference']
		

def intersect(m1, m2):
	''' cut the faces of m1 and m2 at their intersections '''
	m3 = deepcopy(m1)
	intersectwith(m1, m2)
	intersectwith(m2, m3)

def intersection_coords(face_1, face_2, face_orig, edge_start, edge_end):
	''' intersection of an edge with a face, in face and edge coordinates (u,v,w)  
		u,v are the face coordinates (such as 0 <= u+v <= 1,  u,v positive   if the point is in the face)
		w such the edge coordinate (0 <= w <= 1,  if the point is in the edge)
	'''
	return inverse(mat3(face_1, face_2, edge_end-edge_start)) * (edge_end - face_orig)

def notint(x):
	return x > NUMPREC and x < COMPREC

def intersection_edge_face(face, edge):
	coords = intersection_coords(face[1]-face[0], face[2]-face[0], face[0], edge[1], edge[0])
	#if 0 < coords[0] and 0 < coords[1] and coords[0]+coords[1] < 1 and 0 < coords[2] and coords[2] < 1 and (notint(coords[0]) or notint(coords[1])):
	if 0 <= coords[0] and 0 <= coords[1] and coords[0]+coords[1] <= 1 and 0 <= coords[2] and coords[2] <= 1 and (notint(coords[0]) or notint(coords[1])):
		#print(coords, '\t\t', face, edge)
		return edge[0] + (edge[1]-edge[0])*coords[2]
	else:
		return None


def face_intersection_border(f1, f2):
	p1 = intersection_edge_face(f1, (f2[0], f2[1]))
	p2 = intersection_edge_face(f1, (f2[0], f2[2]))
	if   not p1 or (p1 and p2 and distance(p1,p2) < NUMPREC):	p1 = intersection_edge_face(f1, (f2[2], f2[1]))
	elif not p2 or (p1 and p2 and distance(p1,p2) < NUMPREC):	p2 = intersection_edge_face(f1, (f2[2], f2[1]))
	return p1, p2

def append_nonempty_face(mesh, face, group):
	p0 = mesh.points[face[0]]
	e1 = mesh.points[face[1]] - p0
	e2 = mesh.points[face[2]] - p0
	# test si la face et vide (aretes colinéaires)
	if length(cross(e1, e2)) > NUMPREC:
		mesh.faces.append(face)
		mesh.tracks.append(group)
	#else:
		#print('reject', face)

def faceintersection(mesh, fi, f2, f2i):
	pts = mesh.facepoints(fi)
	i1 = face_intersection_border(pts, f2)
	if not i1[0] or not i1[1] or distance(i1[0],i1[1]) < NUMPREC:
		i2 = face_intersection_border(f2, pts)
		if not (i1[0] or i2[0] or i2[1]) or not (i1[1] or i2[1] or i2[0]):
			msg = 'one only point'
		else:
			msg = 'one point from the second face'
		p1 = i1[0] or i2[0] or i2[1] or i1[1]
		p2 = i1[1] or i2[1] or i2[0] or i1[0]
		# fix problem of p1 == p2 with a p3 available elsewhere
		for p in (i1[1], i2[1], i2[0], i1[0]):
			if p and distance(p1,p) > NUMPREC:	
				p2 = p
				break
	else:
		msg = 'two intersections into face'
		p1, p2 = i1
	if p1 and p2:
		return (p1, p2)


def cutface(mesh, fi, edge):
	''' cut the face with an edge [vec3, vec3] '''
	pts = mesh.facepoints(fi)
	p1,p2 = edge
	# remove the former face
	fp = mesh.faces.pop(fi)
	group = mesh.tracks.pop(fi)
	# insert the intersection points
	p1i = mesh.usepointat(p1)
	p2i = mesh.usepointat(p2)
	
	
	n = cross(pts[1]-pts[0], pts[2]-pts[0])
	d = normalize(cross(p2-p1, n))	# projection direction orthogonal to the axis bt p1 and p2
	s = [dot(pt-p1, d) for pt in pts]	# signs of projections
	
	#print('config', fp, s[0]*s[2], s[1]*s[0], s[1]*s[2])
	
	# turn the face to get the proper configuration where fp[1] and fp[2] are apart of the crossing axis (p1, p2)
	# choose the edge at the middle of which the axis is the nearer
	if   s[2]*s[0] <= s[2]*s[1] and s[2]*s[0] <= s[0]*s[1]:	fp = (fp[1], fp[2], fp[0])
	elif s[1]*s[0] <= s[1]*s[2] and s[1]*s[0] <= s[0]*s[2]:	fp = (fp[2], fp[0], fp[1])

	# exchange the points of the intersection in order to avoid crossing triangle 
	p0 = mesh.points[fp[0]]
	a = cross(n, mesh.points[fp[2]] - mesh.points[fp[1]])	# perpendicular axis to base, oriented to triangle center
	if dot(p1-p2, a) < 0:	# p1 is nearer to the base than p2
		p2i,p1i = p1i,p2i
		
	#print('choice', fp, p1i, p2i)
	
	# the links created must have the same normal orientation than the former one
	for face in [
		(p1i, fp[0], fp[1]),
		(p1i, fp[1], p2i),
		(p2i, fp[1], fp[2]),
		(p2i, fp[2], p1i),
		(p1i, fp[2], fp[0]),
		]:
		append_nonempty_face(mesh, face, group)
	return (p1i, p2i)
			
def intersectwith(m1, m2):
	''' cut m1 at the intersection with m2 '''
	intersections = []
	c = 0
	# work with separated faces
	for f2 in range(len(m2.faces)):
		f1 = 0
		for _ in range(len(m1.faces)):	# m1 length varies over the iterations of m2
			face1, face2 = m1.faces[f1], m2.faces[f2]
			edge = faceintersection(m1, f1, m2.facepoints(f2), f2)
			if edge:
				cut = cutface(m1, f1, edge)
				intersections.append((*cut, f2))
				c += 1
				#print('cut', cut)
			else:
				f1 += 1
	return intersections

def booleanwith(m1, m2, side):
	''' execute the boolean operation only on m1 '''
	intersectwith(m1, m2)
	
	m1.mergedoubles()
	
	used = [False] * len(m1.faces)
	front = set()
	notto = set()

	# search for intersections
	for f2 in m2.faces:
		o = m2.points[f2[0]]
		x = m2.points[f2[1]] - o
		y = m2.points[f2[2]] - o
		n = cross(x,y)
		decomp = inverse(mat3(x, y, n))
		for i,f1 in enumerate(m1.faces):
			onplane = []
			summit = 0
			for p in f1:
				d = decomp * (m1.points[p] - o)
				if -NUMPREC <= d[0] and -NUMPREC <= d[1] and d[0] + d[1] <= 1+NUMPREC and abs(d[2]) < NUMPREC:
				#if 0 <= d[0] and 0 <= d[1] and d[0] + d[1] <= 1 and abs(d[2]) < NUMPREC:
					onplane.append(p)
				else:
					summit = p
			if len(onplane) == 2:
				proj = dot(n, m1.points[summit] - o)	* (-1 if side else 1)
				if proj > NUMPREC:
					used[i] = True
					notto.add(edgekey(onplane[0], onplane[1]))
					front.add(edgekey(onplane[0], summit))
					front.add(edgekey(onplane[1], summit))
	
	if not front:
		if side:	
			m1.faces = []
			m1.tracks = []
		return notto
	
	# display frontier
	#for edge in notto:
		#p = (m1.points[edge[0]] + m1.points[edge[1]]) /2
		#scn3D.objs.append(text.Text(tuple(p), str(edge), 9, (1, 1, 0)))
	
	# propagation
	front -= notto
	frontchanged = [True]
	def trypropagate(e1,e2):
		key = edgekey(e1,e2)
		if key not in notto:
			frontchanged[0] = True
			front.add(key)
	
	def propagate(fi,f):
		key = edgekey(f[0], f[1])
		if key in front:
			assert key not in notto
			assert not used[i]
			front.remove(key)
			trypropagate(f[1], f[2])
			trypropagate(f[2], f[0])
			used[fi] = True
			return True
		return False
	# propagation loop
	while frontchanged[0]:
		frontchanged[0] = False
		for i,face in enumerate(m1.faces):
			(used[i]
			or propagate(i, face)
			or propagate(i, (face[2], face[0], face[1]))
			or propagate(i, (face[1], face[2], face[0])))
	
	# selection of faces
	m1.faces =  [f for u,f in zip(used, m1.faces) if u]
	m1.tracks = [t for u,t in zip(used, m1.tracks) if u]
	
	
	# simplify the intersection (only at the intersection are useless points)
	frontier = list(notto)
	frontier.sort(key=lambda e: e[0])
	def processangles(process):
		for i,ei in enumerate(frontier):
			for j,ej in enumerate(frontier):
				if i == j:	continue
				if   ej[0] == ei[1]:	process(ei[0], ei[1], ej[1])
				elif ej[0] == ei[0]:	process(ei[1], ei[0], ej[1])
				elif ej[1] == ei[1]:	process(ei[0], ei[1], ej[0])
				elif ej[1] == ei[0]:	process(ei[1], ei[0], ej[0])
	
	# get the points to keep
	destinations = set()	# points to keep
	def getdests(a,b,c):
		o = m1.points[b]
		if length(cross(m1.points[a]-o, m1.points[c]-o)) > NUMPREC:
			destinations.add(b)
	processangles(getdests)
	
	# find the points to merge
	merges = {}
	def getmerges(a,b,c):
		o = m1.points[b]
		if length(cross(m1.points[a]-o, m1.points[c]-o)) < NUMPREC:
			dst = a if a in destinations else c
			if dst not in merges or merges[dst] != b:
				merges[b] = dst
				while merges[b] in merges:
					merges[b] = merges[merges[b]]
	processangles(getmerges)

	# display merges
	#for src,dst in merges.items():
		#scn3D.objs.append(text.Text(m1.points[src], str(src)+' -> '+str(dst), 9, (1, 1, 0)))
		#scn3D.objs.append(text.Text(m1.points[dst], str(dst), 9, (1, 0, 1)))
	
	m1.mergepoints(merges)
	
	return notto

def boolean(m1, m2, selector):
	''' execute boolean operation on volumes 
		selector decides which part of each mesh to keep
		- False keep the exterior part (part exclusive to the other mesh)
		- True keep the common part
	'''
	if selector[0] is not None:
		if selector[1] is not None:
			mc1, mc2 = deepcopy((m1,m2))
			booleanwith(mc1, m2, selector[0])
			booleanwith(mc2, m1, selector[1])
			if selector[0] and not selector[1]:
				mc1.faces = [(f[0], f[2], f[1]) for f in mc1.faces]
			if not selector[0] and selector[1]:
				mc2.faces = [(f[0], f[2], f[1]) for f in mc2.faces]
			return mc1 + mc2
		else:
			mc1 = deepcopy(m1)
			booleanwith(mc1, selector[0])
			return mc1
	elif selector[1] is not None:
		return boolean(m2, m1, (selector[1], selector[0]))

def union(a,b):			
	''' return a mesh for the union of the volumes. 
		It is a boolean with selector (False,False) 
	'''
	return boolean(a,b,(False, False))
def intersection(a,b):	
	''' return a mesh for the common volume. 
		It is a boolean with selector (True, True) 
	'''
	return boolean(a,b,(True, True))
def difference(a,b):	
	''' return a mesh for the volume of a less the common volume with b
		It is a boolean with selector (False, True)
	'''
	return boolean(a,b,(False, True))

def usefulpts(mesh):
	''' return the points that are useless for the shape of the mesh (eg. those common to triangles of a flat face) '''
	normals = [None]*len(mesh.points)
	keep = [False]*len(mesh.points)
	for i,f in enumerate(mesh.faces):
		p = mesh.facepoints(i)
		n = normalize(cross(p[1]-p[0], p[2]-p[0]))
		for p in f:
			if normals[p] is None:
				normals[p] = n
			elif not keep[p] and distance(normals[p], n) > NUMPREC:
				keep[p] = True
	return keep



if __name__ == '__main__':
	from nprint import nprint
	from mathutils import mat4, rotate
	from time import time
	
	import view, text
	import sys
	from PyQt5.QtWidgets import QApplication
	
	app = QApplication(sys.argv)
	
	main = scn3D = view.Scene()
	
	
	m1 = Mesh(
		[
			vec3(1.0, -1.0, -1.0),
            vec3(1.0, -1.0, 1.0),
            vec3(-1.0, -1.0, 1.0),
            vec3(-1.0, -1.0, -1.0),
            vec3(1.0, 1.0, -1.0),
            vec3(1.0, 1.0, 1.0),
            vec3(-1.0, 1.0, 1.0),
            vec3(-1.0, 1.0, -1.0)],
		[
            (0, 1, 2),
            (0, 2, 3),
            (4, 7, 6),
            (4, 6, 5),
            (0, 4, 5),
            (0, 5, 1),
            (1, 5, 6),
            (1, 6, 2),
            (2, 6, 7),
            (2, 7, 3),
            (4, 0, 3),
            (4, 3, 7)],
		[	0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5	],
		[None] * 6,
		)
	m2 = deepcopy(m1)
	#
	#m2.translate(vec3(0.6, 0.3, 0.4))
	#m2.transform(rotate(mat4(1), 0.3, vec3(1,1,1)))
	#
	#m2.translate(vec3(0.6, 0.3, 0.4))
	#m2.translate(vec3(0.5, 0.3, 0.4))
	#m2.transform(rotate(mat4(1), 0.7, vec3(1,0,0)))
	#
	m2.translate(vec3(0.5, 0.3, 0.4))
	m2.transform(rotate(mat4(1), 0.7, vec3(1,1,0)))
	#
	#m2.translate(vec3(1.99, 0.52, 0.51))
	#
	#m2.translate(vec3(2, 0.5, 0.5))
	
	#m3 = deepcopy(m1)
	#intersectwith(m1, m2)
	#intersectwith(m2, m1)
	#intersectwith(m3, m2)
	#booleanwith(m1, m2, False)
	#booleanwith(m2, m3, True)
		
	#scn3D.objs.append(m2)
	#scn3D.objs.append(m1)
	start = time()
	m3 = boolean(m1, m2, (True, False))
	m3.removeunused()
	print('computation time:', time()-start)
	scn3D.objs.append(m3)
	#for i,pt in enumerate(usefulpts(m3)):
		#if not pt:
			#scn3D.objs.append(text.Text(m3.points[i]*1.05, str(i), 9))
	
	main.show()
	sys.exit(app.exec())
	
