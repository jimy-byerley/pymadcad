'''	
	Defines boolean operations for triangular meshes
'''
from copy import copy,deepcopy
from mathutils import dvec3, dmat3, vec3, mat3, dot, cross, noproject, inverse, sqrt, normalize, length, distance, NUMPREC, COMPREC
from mesh import Mesh, edgekey, connef
from time import time

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
	return inverse(dmat3(face_1, face_2, edge_end-edge_start)) * dvec3(edge_end - face_orig)

def notint(x):
	return x > NUMPREC and x < COMPREC

def intersection_edge_face(face, edge):
	coords = intersection_coords(face[1]-face[0], face[2]-face[0], face[0], edge[1], edge[0])
	#if 0 < coords[0] and 0 < coords[1] and coords[0]+coords[1] < 1 and 0 < coords[2] and coords[2] < 1 and (notint(coords[0]) or notint(coords[1])):
	if 0 <= coords[0] and 0 <= coords[1] and coords[0]+coords[1] <= 1 and 0 <= coords[2] and coords[2] <= 1 and (notint(coords[0]) or notint(coords[1])):
		#print(coords, '\t\t', face, edge)
		#return edge[0] + (edge[1]-edge[0])*coords[2]
		return coords[0]*(face[1]-face[0]) + coords[1]*(face[2]-face[0]) + face[0]
	else:
		return None

def pierce_face(mesh, fi, p, prec, conn=None):
	f = mesh.faces[fi]
	l = len(mesh.faces)
	pi = mesh.usepointat(p, prec)
	mesh.faces[fi] = (f[0], f[1], pi)
	mesh.faces.append((f[1], f[2], pi))
	mesh.faces.append((f[2], f[0], pi))
	mesh.tracks.append(mesh.tracks[fi])
	mesh.tracks.append(mesh.tracks[fi])
	if conn:
		registerface(mesh, conn, fi)
		registerface(mesh, conn, l)
		registerface(mesh, conn, l+1)
	return pi

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

def facesurf(mesh, fi):
	o,x,y = mesh.facepoints(fi)
	return length(cross(x-o, y-o))

def registerface(mesh, conn, fi):
	f = mesh.faces[fi]
	for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
		conn[e] = fi

def intersectwith(m1, m2):
	prec = m1.precision()
	surfprec = prec * m1.maxnum()
	# cut m1 faces with intersections with m2 edges
	for e in m2.edges():
		for fi,f in enumerate(m1.faces):
			p = intersection_edge_face(m1.facepoints(fi), (m2.points[e[0]], m2.points[e[1]]))
			if p:	pierce_face(m1, fi, p, prec)
	# cut m1 edges with intersections with m2 faces
	for i in range(len(m2.faces)):
		for fi in range(len(m1.faces)):
			if facesurf(m1, fi) <= surfprec:	continue
			f = m1.faces[fi]
			fc = [fi,fi,fi]
			for ei,e in enumerate(((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))):
				p = intersection_edge_face(m2.facepoints(i), (m1.points[e[0]], m1.points[e[1]]))
				if p:
					pierce_face(m1, fc[ei], p, prec)
					if fc[ei] == fi:
						fc[1] = len(m1.faces)-2
						fc[2] = len(m1.faces)-1
	
	removefaces(m1, lambda fi: facesurf(m1,fi) <= surfprec)
	return m1

def booleanwith(m1, m2, side):
	''' execute the boolean operation only on m1 '''
	start = time()
	intersectwith(m1, m2)
	print('intersectwith', time()-start)
	start = time()
	prec = m1.precision()
	surfprec = prec * m1.maxnum()
	
	m1.mergeclose()	# TODO voir pourquoi
	
	used = [False] * len(m1.faces)
	front = set()
	notto = set()
	
	# search for intersections
	for f2 in m2.faces:
		o = m2.points[f2[0]]
		x = m2.points[f2[1]] - o
		y = m2.points[f2[2]] - o
		n = cross(x,y)
		decomp = inverse(dmat3(x, y, n))
		triprec = NUMPREC / (1-abs(dot(x,y))/length(x)/length(y))
		for i,f1 in enumerate(m1.faces):
			onplane = [False]*3
			for j in range(3):
				v = m1.points[f1[j]] - o
				d = decomp * dvec3(v)
				if -triprec <= d[0] and -triprec <= d[1] and d[0] + d[1] <= 1+triprec and abs(d[2]) <= prec:
					onplane[j] = True
			for j in range(3):
				if onplane[j] and onplane[j-1]:
					a,b,summit = f1[j-1], f1[j], f1[j-2]
					proj = dot(n, m1.points[summit] - o)	* (-1 if side else 1)
					if proj > prec:
						used[i] = True
						notto.add(edgekey(a, b))
						front.add(edgekey(a, summit))
						front.add(edgekey(b, summit))
	
	if not front:
		if side:	
			m1.faces = []
			m1.tracks = []
		return notto
	
	# display frontier
	#import text
	#for edge in notto:
		#p = (m1.points[edge[0]] + m1.points[edge[1]]) /2
		#scn3D.add(text.Text(p, str(edge), 9, (1, 1, 0)))
	
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
	#for i,u in enumerate(used):
		#if not u:
			#p = m1.facepoints(i)
			#scn3D.add(text.Text((p[0]+p[1]+p[2])/3, 'X', 9, (1,0,1)))
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
		if length(cross(m1.points[a]-o, m1.points[c]-o)) > prec:
			destinations.add(b)
	processangles(getdests)
	
	# find the points to merge
	merges = {}
	def getmerges(a,b,c):
		o = m1.points[b]
		if length(cross(m1.points[a]-o, m1.points[c]-o)) < prec:
			dst = a if a in destinations else c
			if dst not in merges or merges[dst] != b:
				merges[b] = dst
	processangles(getmerges)
	for k,v in merges.items():
		while v in merges:
			merges[k] = v = merges[merges[k]]

	# display merges
	#for src,dst in merges.items():
		#scn3D.objs.append(text.Text(m1.points[src], str(src)+' -> '+str(dst), 9, (1, 1, 0)))
		#scn3D.objs.append(text.Text(m1.points[dst], str(dst), 9, (1, 0, 1)))
	
	m1.mergepoints(merges)
	print('booleanwith', time()-start)
	
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
			booleanwith(mc1, m2, selector[0])
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
	#m2.transform(vec3(0.6, 0.3, 0.4))
	#m2.transform(rotate(mat4(1), 0.3, vec3(1,1,1)))
	#
	#m2.transform(vec3(0.6, 0.3, 0.4))
	#m2.transform(vec3(0.5, 0.3, 0.4))
	#m2.transform(rotate(mat4(1), 0.7, vec3(1,0,0)))
	#
	m2.transform(vec3(0.5, 0.3, 0.4))
	m2.transform(rotate(mat4(1), 0.7, vec3(1,1,0)))
	#
	#m2.transform(vec3(1.99, 0.52, 0.51))
	#
	#m2.transform(vec3(2, 0.5, 0.5))
	
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
	#m3.strippoints()
	print('computation time:', time()-start)
	
	print('face15', facesurf(m3, 15))
	
	assert m3.isvalid()
	#assert m3.isenvelope()
	
	# debug purpose
	#m3.options.update({'debug_display':True, 'debug_points':True, 'debug_faces':'indices'})
	#m2.options.update({'debug_display':True})
	#m3.groups = [None]*len(m3.faces)
	#m3.tracks = list(range(len(m3.faces)))
	# display
	scn3D.add(m3)
	
	main.show()
	sys.exit(app.exec())
	
