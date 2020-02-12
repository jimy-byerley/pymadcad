from mesh import Mesh, edgekey
from mathutils import vec3, cross, dot, normalize, sqrt, distance, length, NUMPREC
import math
from itertools import chain

# return the points intersectig the mesh, by propagation from line
def intersections(mesh, line, criterion):
	edges = mesh.edges()
	for i in range(1,len(line)):		
		edges.remove(edgekey(line[i-1], line[i]))
	front = [(o, p, None) for o,p in enumerate(line)]
	cutted = {}
	unsolved = True
	while unsolved:
		newfront = []
		unsolved = False
		for o,p,found in front:
			if found is not None:
				newfront.append((o,p,found))
			else:
				unsolved = True
				for edge in edges:
					if (edge[0] == p or edge[1] == p) and cutted.get(edge, (None,-1))[1] != o:
						print(edge)
						if edge[1] == p:	e = (edge[1], edge[0])
						else:				e = edge
						r = criterion(mesh, line, o, p, e[1])
						if r:
							if edge not in cutted or not cutted[edge][0] or distance(mesh.points[o], cutted[edge][0]) < distance(mesh.points[o], r[0]):
								newfront.append((o,p,r))
								cutted[edge] = (r[0], o)
						else:
							newfront.append((o,e[1],None))
							cutted[edge] = (None, o)
		front = newfront
	return front

def c_distance(dist, mesh, line, i, p, n):
	l = len(line)
	o = mesh.points[line[i]]
	c = mesh.points[n]
	# critere destructif
	if distance(c,o) < dist:
		return None
		
	# calcul des intersections avec les aretes adjacentes
	d1 = normalize(mesh.points[line[i-1]] - o) if i-1 >= 0 else None
	d2 = normalize(mesh.points[line[i+1]] - o) if i+1 < l else None
	d1 = d1 or d2
	d2 = d2 or d1
	#print('    ', i, d1, d2)
	o3 = mesh.points[p]
	d3 = normalize(c - o3)
	def body(d, dp):
		#print('params', (o,d), (o3,d3), dist)
		i = ap_froma((o,d), (o3,d3), dist)
		proj = length(i-o3) / length(c-o3)
		if 0 <= proj and proj <= 1:
			#if abs(distance_ap((o,d), i) - dist) > NUMPREC:
				#print('reproj err', p,n, distance_ap((o,d), i))
			return i, distance_ap((o,dp), i)
		else:
			return None, 0
	i1,dist1 = body(d1,d2)
	i2,dist2 = body(d2,d1)
	
	# selection de l'intersection la plus éloignée
	if not i1 or not i2:	return None
	if dist2 < dist1:
		#print('   take i1')
		return i1, '{}  {:.4g}  {:.4g}'.format(i, dist1, dist2)
	else:
		#print('   take i2')
		return i2, '{}  {:.4g}  {:.4g}'.format(i, dist2, dist1)

def propagintersect(mesh, line, process, compare):
	edges = mesh.edges()
	intersections = []
	interior = set()
	
	for i in range(len(line)-1):
	
		reached = {}	# dictionnary of states  edgekey: (intersection, reference segment)
		bold = (line[i], line[i+1])
		front = {line[i], line[i+1]}
	
		while front:
			newfront = set()
			for e in edges:
				if e not in reached:
					for edge in (e, (e[1],e[0])):
						if edge[0] in front:
							data = process(mesh, bold, edge)
							if data:
								reached[e] = data
							else:
								reached[e] = None
								newfront.add(edge[1])
			front = newfront
		nprint('reached', reached)
		# update propagations from other segments
		j = 0
		while j < len(intersections):
			ref,edge,data = intersections[j]
			if edge in reached:
				if reached[edge]:
					if compare(mesh, (line[ref], line[ref+1]), reached[edge], (line[i], line[i+1]), data):
						intersections[j] = (i,edge,reached[edge])
					del reached[edge]
				else:
					intersections.pop(j)
					continue
			j += 1
		# append this propagation
		for edge,data in reached.items():
			if edge not in interior:
				if data:	intersections.append((i,edge,data))
				else:		interior.add(edge)
	
	nprint('intersections', intersections)
	return [(ref,data)  for ref,edge,data in intersections]

def c_distance(radius):
	def process(mesh, bold, segt):
		return intersect_boldsegt(
				(mesh.points[bold[0]], mesh.points[bold[1]]), 
				(mesh.points[segt[0]], mesh.points[segt[1]]), 
				radius)
	def compare(mesh, bold1, pt1, bold2, pt2):
		d1 = distance_segt(
				(mesh.points[bold1[0]], mesh.points[bold1[1]]), 
				pt1)
		d2 = distance_segt(
				(mesh.points[bold1[0]], mesh.points[bold1[1]]), 
				pt2)
		print('  dists', d1, d2)
		return d1 > d2
	return process, compare

class cycle:
	def __init__(self, iterable):
		self.iterable = iterable
		self.index = -1
		self.again = True
	def __iter__(self):	return self
	def __next__(self):
		self.index += 1
		if self.index == len(iterable):
			if self.again:	self.index = 0
			else:			raise StopIteration()
		return self.iterable[self.index]
		
		

def propagintersect(mesh, lines):
	for face in mesh.faces:
		for i in range(3):
			edge = (face[i-2],face[i-1])
			if edge in lines:
				front.append((face[i], 
	return intersections


def intersect_boldsegt(bold, segt, radius):
	b1,b2 = bold
	s1,s2 = segt
	db,ds = normalize(b2-b1), normalize(s2-s1)
	
	# compute the intersection with the envelope of the boldsegt
	pt = ap_froma((b1,db), (s1,ds), radius)
	if dot(pt-b1,db) < 0:
		pt = ap_fromp(b1, (s1,ds), radius)
	elif dot(pt-b2,db) > 0:
		pt = ap_fromp(b2, (s1,ds), radius)
	
	# return if the intersection with the segt axis is on the segt
	proj = length(pt-s1) / length(s2-s1)
	if 0 <= proj and proj <= 1:
		return pt

def distance_segt(segt, pt):
	d1,d2,d3 = distance(segt[1],pt), distance(segt[1],pt), distance(*segt)
	if   d1**2 + d3**2 < d2**2:	return d1
	elif d2**2 + d3**3 < d1**2:	return d2
	else:	return distance_ap((segt[0], (segt[1]-segt[0])/d3), pt)
	
def ap_fromp(orig, axis, dist):
	''' axis point at distance from point
		gives the point of axis a2 that is at `dist` distance of axis a1 
	'''
	o = orig - axis[0]
	b = dot(o, axis[1])
	a = length(o - b*axis[1])
	return axis[0] + (sqrt(dist**2 - a**2) + b) * axis[1]	

def ap_froma(a1, a2, dist):
	''' axis point at distance from axis '''
	# compute a base adapted to the problem
	x = a1[1]
	z = normalize(cross(x, a2[1]))
	y = cross(z,x)
	o = a2[0]-a1[0]
	t = (sqrt(dist**2 - dot(o,z)**2) - dot(o,y)) / dot(a2[1],y)
	return a2[0] + a2[1]*t

def distance_aa(a1, a2):
	''' distance betwee axis and axis '''
	return abs(dot(
			normalize(cross(a1[1], a2[1])), 
			a1[0] - a2[0],
			))

def distance_ap(a, p):
	''' distance between axis and point '''
	v = p-a[0]
	return sqrt(length(v)**2 - dot(v,a[1])**2)
	#return length(v - dot(v,a[1])*a[1])



if __name__ == '__main__':
	import sys
	
	# test axispointat
	pt = ap_froma(
			(vec3(0,1,0), vec3(1,0,0)),
			(vec3(0,0,1), vec3(0,1,0)),
			sqrt(2),
			)
	assert pt == vec3(0,2,1), pt
	d = distance_ap((vec3(0,1,0), vec3(1,0,0)), pt)
	assert abs(d - sqrt(2)) < NUMPREC, d
	
	pt = ap_froma(
			(vec3(0,0,0), vec3(1,0,0)),
			(vec3(0,0,0), normalize(vec3(1,0.5,0))),
			2,
			)
	assert pt == vec3(4,2,0), pt
	
	a1 = (vec3( 0, 0, 0 ), vec3( 0, -0.707107, 0.707107 ))
	a2 = (vec3( -1, 1, 0 ), vec3( -1, 0, 0 ))
	ptd = distance_ap(a1, ap_froma(a1, a2, 1.5))
	assert abs(ptd - 1.5) < NUMPREC, ptd
	
	# test intersections
	from generation import saddle, tube, outline, Outline, makeloops
	from primitives import Arc
	import view, text
	from PyQt5.QtWidgets import QApplication
	from nprint import nprint
	
	app = QApplication(sys.argv)
	main = scn3D = view.Scene()
	
	m = saddle(
			Outline([vec3(-2,1,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], [0,1,2,3]),
			#outline([vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)]),
			outline(Arc(vec3(0,1,-1),vec3(0,0.5,0),vec3(0,1,1))),
			)
	m.options.update({'debug_display': True, 'debug_points': True })
	scn3D.objs.append(m)
	line = makeloops(list(m.group(1).outlines_unoriented() & m.group(2).outlines_unoriented()))[0]
	print('line', line)
	for ref, pt in propagintersect(m, line, *c_distance(1.7)):
		scn3D.objs.append(text.Text(pt + vec3(0,-0.1,0), str(ref), color=(0,1,0.2)))
	
	main.show()
	sys.exit(app.exec())
