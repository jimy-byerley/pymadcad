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
	#if NUMPREC < coords[0] and NUMPREC < coords[1] and coords[0]+coords[1] < 1 and 0 < coords[2] and coords[2] < 1 and (notint(coords[0]) or notint(coords[1])):
	#if -NUMPREC <= coords[0] and -NUMPREC <= coords[1] and coords[0]+coords[1] <= 1+NUMPREC and 0 <= coords[2] and coords[2] <= 1 and (notint(coords[0]) or notint(coords[1])):
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
	''' remove faces that satisfy a criterion '''
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
	
def faceheight(mesh, fi):
	f = mesh.facepoints(fi)
	heights = [length(noproject(f[i-2]-f[i], normalize(f[i-1]-f[i])))	for i in range(3)]
	return min(heights)

def registerface(mesh, conn, fi):
	f = mesh.faces[fi]
	for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
		conn[e] = fi

from mesh import connpp
import generation
def intersectwith(m1, m2, prec=None):
	if not prec:	prec = m1.precision()
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

import hashing

def facekeyo(a,b,c):
	if a < b and b < c:		return (a,b,c)
	elif a < b:				return (c,a,b)
	else:					return (b,c,a)
	
def intersectwith(m1, m2, prec=None):
	if not prec:	prec = m1.precision()
	#prec *= 4
	cellsize = hashing.meshcellsize(m1)
	print('cellsize', cellsize)
	
	cuts = {}	# cut points for each face from m2
	
	print('* hashing')
	conn2 = connef(m2.faces)
	prox1 = hashing.PositionMap(cellsize)
	for fi in range(len(m1.faces)):	
		#print('    ',fi)
		prox1.add(m1.facepoints(fi), fi)
	
	# cut m1 faces with intersections with m2 edges
	print('* edge vs faces')
	for e in m2.edges():
		neigh = list(set( prox1.get((m2.points[e[0]], m2.points[e[1]])) ))
		#print(e,neigh)
		for fi in neigh:
			p = intersection_edge_face(m1.facepoints(fi), (m2.points[e[0]], m2.points[e[1]]))
			if p:	
				pi = pierce_face(m1, fi, p, prec)
				# keep cuts and prox1 up to date
				l = len(m1.faces)
				neigh.append(l-1)
				neigh.append(l-2)
				prox1.add(m1.facepoints(l-1), l-1)
				prox1.add(m1.facepoints(l-2), l-2)
				i = conn2.get(e)
				if i is not None:
					if i in cuts:	cuts[i].append(pi)
					else:			cuts[i] = [pi]
				i = conn2.get((e[1],e[0]))
				if i is not None:
					if i in cuts:	cuts[i].append(pi)
					else:			cuts[i] = [pi]
		
	# cut m1 edges with intersections with m2 faces
	print('* faces vs faces')
	for i in range(len(m2.faces)):
		neigh = list(set(prox1.get(m2.facepoints(i))))
		#print(i,neigh)
		for fi in neigh:
			if faceheight(m1,fi) <= prec:	continue
			f = m1.faces[fi]
			fc = [fi,fi,fi]
			for ei,e in enumerate(((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))):
				p = intersection_edge_face(m2.facepoints(i), (m1.points[e[0]], m1.points[e[1]]))
				if p:
					pi = pierce_face(m1, fc[ei], p, prec)
					l = len(m1.faces)
					if fc[ei] == fi:
						fc[1] = l-2
						fc[2] = l-1
					# keep cuts and prox1 up to date
					prox1.add(m1.facepoints(l-1), l-1)
					prox1.add(m1.facepoints(l-2), l-2)
					if i in cuts:	cuts[i].append(pi)
					else:			cuts[i] = [pi]
	
	# set of faces, to detect identical opposed faces
	faces = set()
	for f in m1.faces:	faces.add(facekeyo(*f))
	# remove empty faces, or faces that has an identical opposed twin
	# empty faces are detected using triangle's height because triangle surface is bad when we got very ong empty triangles
	def crit(fi):
		a,b,c = m1.faces[fi]
		return faceheight(m1,fi) <= prec or facesurf(m1,fi) <= prec or facekeyo(a,c,b) in faces
	removefaces(m1, crit)
	
	# find edges at the intersection
	frontier = set()
	conn1 = connpp(m1.faces)
	for f2,pts in cuts.items():
		for p in pts:
			if p in conn1:
				for neigh in conn1[p]:
					if neigh in pts:
						frontier.add((edgekey(p,neigh), f2))
	#nprint(generation.makeloops([e for e,f2 in frontier], oriented=False))
	
	return frontier

def booleanwith(m1, m2, side, prec=None):
	''' execute the boolean operation only on m1 '''
	if not prec:	prec = m1.precision()
	# NOTE: le probleme arrive aussi sans hashage
	start = time()
	frontier = intersectwith(m1, m2, prec)
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
	from mesh import Web
	w = Web([1.01*p for p in m1.points], [e for e,f2 in frontier])
	w.options['color'] = (1,0.9,0.2)
	scn3D.add(w)
	
	# propagation
	front -= notto
	frontchanged = [True]
	def trypropagate(e1,e2):
		key = edgekey(e1,e2)
		if key not in notto:
			frontchanged[0] = True
			front.add(key)
	c = 0
	def propagate(fi,f):
		key = edgekey(f[0], f[1])
		if key in front:
			assert key not in notto
			front.remove(key)
			trypropagate(f[1], f[2])
			trypropagate(f[2], f[0])
			used[fi] = c
			return True
		return False
	# propagation loop
	while frontchanged[0]:
		c += 1
		frontchanged[0] = False
		for i,face in enumerate(m1.faces):
			(used[i]
			or propagate(i, face)
			or propagate(i, (face[2], face[0], face[1]))
			or propagate(i, (face[1], face[2], face[0])))
	
	# selection of faces
	import text
	for i,u in enumerate(used):
		if u:
			p = m1.facepoints(i)
			scn3D.add(text.Text((p[0]+p[1]+p[2])/3, str(u), 9, (1,0,1)))
	m1.faces =  [f for u,f in zip(used, m1.faces) if u]
	m1.tracks = [t for u,t in zip(used, m1.tracks) if u]
	
	# simplify the intersection (only at the intersection are useless points)
	m1.mergepoints(line_simplification(m1, notto, prec))
	
	print('booleanwith', time()-start)
	
	return notto
	
from mathutils import anglebt

def line_simplification(mesh, line, prec=None):	# TODO mettre dans un module en commun avec cut.py
	''' simplify the points that has no angle, merging these to some that have '''
	if not prec:	prec = mesh.precision()
	print('simplify', prec)
	
	line = list(line)
	def processangles(process):
		for i,ei in enumerate(line):
			for j,ej in enumerate(line):
				if j >= i:	break
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
	
	merges = {}
	effect = True
	while effect:
		effect = False
		for i,e in enumerate(line):
			for a,b in (e, (e[1],e[0])):
				if not (a in merges or a in destinations) and (b in destinations or b in merges):
					merges[a] = b if b in destinations else merges[b]
					effect = True
					break
	
	# display merges
	#import text
	#for src,dst in merges.items():
		#scn3D.add(text.Text(mesh.points[src], str(src)+' -> '+str(dst), 9, (1, 1, 0)))
		#scn3D.add(text.Text(mesh.points[dst], str(dst), 9, (1, 0, 1)))
	
	return merges


def boolean(m1, m2, selector, prec=None):
	''' execute boolean operation on volumes 
		selector decides which part of each mesh to keep
		- False keep the exterior part (part exclusive to the other mesh)
		- True keep the common part
	'''
	if not prec:	prec = max(m1.precision(), m2.precision())
	
	if selector[0] is not None:
		if selector[1] is not None:
			mc1, mc2 = deepcopy((m1,m2))
			booleanwith(mc1, m2, selector[0], prec)
			booleanwith(mc2, m1, selector[1], prec)
			if selector[0] and not selector[1]:
				mc1.faces = [(f[0], f[2], f[1]) for f in mc1.faces]
			if not selector[0] and selector[1]:
				mc2.faces = [(f[0], f[2], f[1]) for f in mc2.faces]
			res = mc1 + mc2
			res.mergeclose()
			return res
		else:
			mc1 = deepcopy(m1)
			booleanwith(mc1, m2, selector[0], prec)
			return mc1
	elif selector[1] is not None:
		return boolean(m2, m1, (selector[1], selector[0]))

def union(a,b):			
	''' return a mesh for the union of the volumes. 
		It is a boolean with selector (False,False) 
	'''
	return boolean(a,b, (False,False))
def intersection(a,b):	
	''' return a mesh for the common volume. 
		It is a boolean with selector (True, True) 
	'''
	return boolean(a,b, (True,True))
def difference(a,b):	
	''' return a mesh for the volume of a less the common volume with b
		It is a boolean with selector (False, True)
	'''
	return boolean(a,b, (False,True))



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
	m3.mergeclose()
	m3.strippoints()
	print('computation time:', time()-start)
	
	#print('face15', facesurf(m3, 15))
	
	m3.check()
	#assert m3.isenvelope()
	
	# debug purpose
	m3.options.update({'debug_display':True, 'debug_points':True, 'debug_faces':'indices'})
	m2.options.update({'debug_display':True, 'debug_points':True})
	m3.groups = [None]*len(m3.faces)
	m3.tracks = list(range(len(m3.faces)))
	# display
	scn3D.add(m3)
	#scn3D.add(m2)
	
	#for k in list(_proxa.dict):
		#if k not in _proxb.dict:
			#del _proxa.dict[k]
	#_proxa.options.update({'color':(1,0.9,0.1)})
	#scn3D.add(_proxa)
	#scn3D.add(_proxb)
	
	main.show()
	sys.exit(app.exec())
	
