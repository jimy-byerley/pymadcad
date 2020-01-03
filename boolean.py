import numpy as np
from copy import copy,deepcopy
from mathutils import vec3, mat3, vec4, dot, cross, inverse, normalize, length, determinant, NUMPREC, COMPREC

class Mesh:
	def __init__(self, points, faces):
		self.points = points
		self.faces = faces
	
	def facepoints(self, index):
		f = self.faces[index]
		return self.points[f[0]], self.points[f[1]], self.points[f[2]]
	
	def translate(self, displt):
		for i in range(len(self.points)):
			self.points[i] += displt
	
	def transform(self, mat):
		for i in range(len(self.points)):
			self.points[i] = vec3(mat * vec4(self.points[i], 1))
	
	def mergedoubles(self, distance=NUMPREC):
		merges = {}
		for j in reversed(range(len(self.points))):
			for i in range(j):
				if length(self.points[i]-self.points[j]) < distance and i not in merges:
					merges[j] = i
					break
		self.mergepoints(merges)
		return merges
	
	def mergepoints(self, merges):
		for i in range(len(self.faces)):
			f = self.faces[i]
			self.faces[i] = (
				merges.get(f[0], f[0]),
				merges.get(f[1], f[1]),
				merges.get(f[2], f[2]),
				)
		
	def facenormals(self):
		facenormals = []
		for face in self.faces:
			p0 = self.points[face[0]]
			e1 = self.points[face[1]] - p0
			e2 = self.points[face[2]] - p0
			facenormals.append(normalize(cross(e1, e2)))
		return facenormals
	
	def display(self, scene):
		import view
		import text
		
		for i,p in enumerate(self.points):
			scene.add(text.Text(p, str(i), 9))
		
		fn = np.array([tuple(p) for p in self.facenormals()])
		points = np.array([tuple(p) for p in self.points], dtype=np.float32)		
		lines = []
		for i in range(0, 3*len(self.faces), 3):
			lines.append((i, i+1))
			lines.append((i+1, i+2))
			lines.append((i, i+2))
		
		return view.SolidDisplay(scene,
			points[np.array(self.faces, dtype=np.uint32)].reshape((len(self.faces)*3,3)),
			np.hstack((fn, fn, fn)).reshape((len(self.faces)*3,3)),
			faces = np.array(range(3*len(self.faces)), dtype=np.uint32).reshape(len(self.faces),3),
			lines = np.array(lines, dtype=np.uint32),
			)
		
		

def intersect(m1, m2):
	intersectwith(m1, m2)
	intersectwith(m2, m1)

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
	#if 0 <= coords[0] and 0 <= coords[1] and coords[0]+coords[1] <= 1 and 0 < coords[2] and coords[2] <= 1 and (notint(coords[0]) or notint(coords[1]) or notint(coords[2])):
	#if 0 <= coords[0] and 0 <= coords[1] and coords[0]+coords[1] <= 1 and 0 < coords[2] and coords[2] <= 1 and (notint(coords[0]) or notint(coords[1])):
	#if 0 <= coords[0] and 0 <= coords[1] and coords[0]+coords[1] <= 1 and 0 <= coords[2] and coords[2] <= 1:
	if 0 < coords[0] and 0 < coords[1] and coords[0]+coords[1] < 1 and 0 < coords[2] and coords[2] < 1 and (notint(coords[0]) or notint(coords[1])):
	#if -NUMPREC <= coords[0] and -NUMPREC <= coords[1] and coords[0]+coords[1] <= 1+NUMPREC and -NUMPREC <= coords[2] and coords[2] <= 1+NUMPREC and (notint(coords[0]) or notint(coords[1])):
		return edge[0] + (edge[1]-edge[0])*coords[2]
	else:
		return None


def intersection(f1, f2):
	p1 = intersection_edge_face(f1, (f2[0], f2[1]))
	p2 = intersection_edge_face(f1, (f2[0], f2[2]))
	if not p1:	p1 = intersection_edge_face(f1, (f2[2], f2[1]))
	if not p2:	p2 = intersection_edge_face(f1, (f2[2], f2[1]))
	return p1, p2

def append_nonempty_face(mesh, face):
	p0 = mesh.points[face[0]]
	e1 = mesh.points[face[1]] - p0
	e2 = mesh.points[face[2]] - p0
	# test si la face et vide (aretes colinéaires)
	if length(cross(e1, e2)) > NUMPREC:
		mesh.faces.append(face)

def faceintersection(mesh, fi, f2):
	pts = mesh.facepoints(fi)
	i1 = intersection(pts, f2)
	if not i1[0] or not i1[1]:
		i2 = intersection(f2, mesh.facepoints(fi))
		if not (i1[0] or i2[0] or i2[1]) or not (i1[1] or i2[1] or i2[0]):
			msg = 'one only point'
		else:
			msg = 'one point from the second face'
		p1 = i1[0] or i2[0] or i2[1] or i1[1]
		p2 = i1[1] or i2[1] or i2[0] or i1[0]
	else:
		msg = 'two intersections into face'
		p1, p2 = i1
	if p1:
		print(msg)
		return (p1, p2)

def cutface(mesh, fi, edge):
	pts = mesh.facepoints(fi)
	p1,p2 = edge
	# remove the former face
	fp = mesh.faces.pop(fi)
	# insert the intersection points
	p1i = len(mesh.points)
	p2i = p1i+1
	mesh.points.append(p1)
	mesh.points.append(p2)
	
	d = cross(p2-p1, cross(pts[1]-pts[0], pts[2]-pts[0]))	# projection direction orthogonal to the axis bt p1 and p2
	s = [dot(pt-p1, d) for pt in pts]	# signs of projections
	
	# turn the face to get the proper configuration where fp[1] and fp[2] are apart of the crossing axis (p1, p2)
	# choose the edge at the middle of which the axis is the nearer
	if   s[2]*s[0] <= s[2]*s[1] and s[2]*s[0] <= s[0]*s[1]:	fp = (fp[1], fp[2], fp[0])
	elif s[1]*s[0] <= s[1]*s[2] and s[1]*s[0] <= s[0]*s[2]:	fp = (fp[2], fp[0], fp[1])

	# exchange the points of the intersection in order to avoid crossing triangle 
	p0 = mesh.points[fp[0]]
	if length(p0-p1) > length(p0-p2):
		p2i,p1i = p1i,p2i
	
	# the links created must have the same normal orientation than the former one
	for face in [
		(p1i, fp[0], fp[1]),
		(p1i, fp[1], p2i),
		(p2i, fp[1], fp[2]),
		(p2i, fp[2], p1i),
		(p1i, fp[2], fp[0]),
		]:
		append_nonempty_face(mesh, face)
	return (p1i, p2i)
			
def intersectwith(m1, m2):
	intersections = []
	# work with separated faces
	for f2 in range(len(m2.faces)):
		f1 = 0
		for _ in range(len(m1.faces)):	# m1 length varies over the iterations of m2
			edge = faceintersection(m1, f1, m2.facepoints(f2))
			if edge:
				intersections.append((*cutface(m1, f1, edge), f2))
			else:
				f1 += 1
	return intersections

def edgekey(a,b):
	if a < b:	return (a,b)
	else:		return (b,a)

def booleanwith(m1, m2, side):
	r = intersectwith(m1, m2) + intersectwith(m1, m2)
	merges = m1.mergedoubles()
	intersections = {edgekey(merges.get(e1,e1), merges.get(e2,e2)): f 	for e1,e2,f in r}
	
	nprint('intersections', intersections)
	
	used = [False] * len(m1.faces)
	front = set()
	notto = set()
	
	def tryuseface(f1, s1):
		key = edgekey(s1[0], s1[1])
		f2 = intersections.get(key)
		if f2 is not None:
			s2 = m2.faces[f2]
			d1 = m1.points[s1[2]] - m1.points[s1[0]]	# direction of the non-frontier point
			n2 = cross(	# normal of the intersecting face
					m2.points[s2[1]] - m2.points[s2[0]], 
					m2.points[s2[2]] - m2.points[s2[0]])
			# select the face if it's at the desired side of the intersecting face
			if (dot(d1,n2) > 0) ^ side:
				used[f1] = True
				front.add(edgekey(s1[1], s1[2]))
				front.add(edgekey(s1[0], s1[2]))
			notto.add(key)
			return True
		return False
	
	# init the propagation
	for i,face in enumerate(m1.faces):
		(tryuseface(i, face)
		or tryuseface(i, (face[2], face[0], face[1]))
		or tryuseface(i, (face[1], face[2], face[0])))
	
	# complete the loops of faces
	extremes = set()
	for edge in notto:
		if edge[0] == edge[1]:	continue
		for pt in edge:
			if pt in extremes:	extremes.remove(pt)
			else:				extremes.add(pt)
	def trycomplete(e1, e2):
		if e1 in extremes and e2 in extremes:
			extremes.remove(e1)
			extremes.remove(e2)
			notto.add(edgekey(e1,e2))
			return True
	for face in m1.faces:
		(	trycomplete(face[0], face[1])
		or	trycomplete(face[1], face[2])
		or	trycomplete(face[2], face[0]))
		
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
	
	while frontchanged[0]:
		frontchanged[0] = False
		for i,face in enumerate(m1.faces):
			(used[i]
			or propagate(i, face)
			or propagate(i, (face[2], face[0], face[1]))
			or propagate(i, (face[1], face[2], face[0])))
	
	m1.faces = [f for u,f in zip(used, m1.faces) if u]

def boolean(m1, m2, selector):
	booleanwith(m1, m2, selector[0])
	booleanwith(m2, m1, selector[1])
	m1 + m2

if __name__ == '__main__':
	from nprint import nprint
	from mathutils import vec4, mat4, rotate
	
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
		)
	m2 = deepcopy(m1)
	m2.translate(vec3(0.6, 0.3, 0.4))
	m2.transform(rotate(mat4(1), 0.3, vec3(1,1,1)))
	#m2.transform(rotate(mat4(1), 0.7, vec3(1,0,0)))
	#m3 = deepcopy(m1)
	#intersectwith(m1, m2)
	#intersectwith(m2, m3)
	#intersectwith(m2, m3)
	#booleanwith(m1, m2, False)
	#booleanwith(m2, m3, False)
	
	import view, text
	import sys
	from PyQt5 import Qt
	from PyQt5.QtWidgets import QApplication
	
	app = QApplication(sys.argv)
	
	main = scn3D = view.Scene()
	
	scn3D.add(m2)
	scn3D.add(text.Text((2,0,0), 'coucou\n\ttout le monde', 10))
	
	main.show()
	sys.exit(app.exec())
	
