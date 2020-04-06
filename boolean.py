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

from mesh import connpp
import generation
def intersectwith(m1, m2):
	prec = m1.precision()
	#surfprec = prec * m1.maxnum()
	surfprec = prec
	cuts = {}
	
	# cut m1 faces with intersections with m2 edges
	conn2 = connef(m2.faces)
	for e in m2.edges():
		for fi,f in enumerate(m1.faces):
			p = intersection_edge_face(m1.facepoints(fi), (m2.points[e[0]], m2.points[e[1]]))
			if p:	
				pi = pierce_face(m1, fi, p, prec)
				i = conn2.get(e)
				if i is not None:
					if i in cuts:	cuts[i].append(pi)
					else:			cuts[i] = [pi]
				i = conn2.get((e[1],e[0]))
				if i is not None:
					if i in cuts:	cuts[i].append(pi)
					else:			cuts[i] = [pi]
	
	# cut m1 edges with intersections with m2 faces
	for i in range(len(m2.faces)):
		for fi in range(len(m1.faces)):
			if facesurf(m1, fi) <= surfprec:	continue
			f = m1.faces[fi]
			fc = [fi,fi,fi]
			for ei,e in enumerate(((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))):
				p = intersection_edge_face(m2.facepoints(i), (m1.points[e[0]], m1.points[e[1]]))
				if p:
					pi = pierce_face(m1, fc[ei], p, prec)
					if i in cuts:	cuts[i].append(pi)
					else:			cuts[i] = [pi]
					if fc[ei] == fi:
						fc[1] = len(m1.faces)-2
						fc[2] = len(m1.faces)-1
	
	removefaces(m1, lambda fi: facesurf(m1,fi) <= surfprec)
	
	# find edges at the intersection
	frontier = []
	conn1 = connpp(m1.faces)
	for f2,pts in cuts.items():
		for p in pts:
			#p = merges.get(p,p)
			if p in conn1:
				for neigh in conn1[p]:
					if neigh in pts:
						frontier.append((edgekey(p,neigh), f2))
	#print(generation.makeloops(frontier, oriented=False))
	
	return frontier

def booleanwith(m1, m2, side):
	''' execute the boolean operation only on m1 '''
	prec = m1.precision()
	
	start = time()
	frontier = intersectwith(m1, m2)
	print('intersectwith', time()-start)
	start = time()
	
	conn1 = connef(m1.faces)
	used = [False] * len(m1.faces)
	notto = set((e[0] for e in frontier))
	front = set()
	# get front and mark frontier faces as used
	for e,f2 in frontier:
		for edge in (e, (e[1],e[0])):
			if edge in conn1:
				fi = conn1[edge]
				for p in m1.faces[fi]:
					if p not in edge:	summit = p
				o = m1.points[edge[0]]
				proj = dot(m1.points[summit] - m1.points[edge[0]], m2.facenormal(f2))  * (-1 if side else 1)
				if proj > prec:
					used[fi] = True
					front.add(edgekey(edge[0], summit))
					front.add(edgekey(edge[1], summit))
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
	m1.mergepoints(line_simplification(m1, notto))

	print('booleanwith', time()-start)
	
	return notto

def line_simplification(mesh, line):	# TODO mettre dans un module en commun avec cut.py
	''' simplify the points that has no angle, merging these to some that have '''
	prec = mesh.precision()
	
	line = list(line)
	line.sort(key=lambda e: e[0])
	def processangles(process):
		for i,ei in enumerate(line):
			for j,ej in enumerate(line):
				if i == j:	continue
				if   ej[0] == ei[1]:	process(ei[0], ei[1], ej[1])
				elif ej[0] == ei[0]:	process(ei[1], ei[0], ej[1])
				elif ej[1] == ei[1]:	process(ei[0], ei[1], ej[0])
				elif ej[1] == ei[0]:	process(ei[1], ei[0], ej[0])
	
	# get the points to keep
	destinations = set()	# points to keep
	def getdests(a,b,c):
		o = mesh.points[b]
		if length(cross(mesh.points[a]-o, mesh.points[c]-o)) > prec:
			destinations.add(b)
	processangles(getdests)
	
	# find the points to merge
	merges = {}
	def getmerges(a,b,c):
		o = mesh.points[b]
		if length(cross(mesh.points[a]-o, mesh.points[c]-o)) < prec:
			dst = a if a in destinations else c
			if dst not in merges or merges[dst] != b:
				merges[b] = dst
				while dst in merges:
					merges[b] = dst = merges[dst]
	processangles(getmerges)
	for k,v in merges.items():
		while v in merges:
			merges[k] = v = merges[v]

	# display merges
	#for src,dst in merges.items():
		#scn3D.objs.append(text.Text(m1.points[src], str(src)+' -> '+str(dst), 9, (1, 1, 0)))
		#scn3D.objs.append(text.Text(m1.points[dst], str(dst), 9, (1, 0, 1)))
	
	return merges


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
	
	start = time()
	m3 = boolean(m1, m2, (True, False))
	m3.strippoints()
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
	#scn3D.add(m2)
	
	main.show()
	sys.exit(app.exec())
	
