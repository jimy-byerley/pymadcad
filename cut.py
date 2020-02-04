from mesh import Mesh, edgekey
from mathutils import vec3,cross,dot,normalize,sqrt

# return the points intersectig the mesh, by propagation from line
def intersections(mesh, line, criterion):
	edges = mesh.edges()
	front = [(o, p, None) for o,p in enumerate(line+reversed(line[1:-1]))]
	forbidden = set((line[i-1], line[i]) for i in range(len(line)))
	unsolved = True
	while unsolved:
		newfront = []
		unsolved = False
		for o,p,found in front:
			if found is not None:
				newfront.append(o,p,found)
			else:
				unsolved = True
				for edge in edges:
					if edge[0] == p: 
						e = edgekey(*edge)
						if e not in forbidden:
							r = criterion(mesh, line, o, p, edge[1])
							if r:	newfront.append(o,p,r)
							else:	newfront.append(o,edge[1],None)
							#forbidden.append(e)
		front = newfront
	return front

def c_distance(dist, mesh, line, i, p, n):
	o = mesh.points[line[i]]
	d1 = mesh.points[line[i-1]] - o
	d2 = mesh.points[line[(i+1)%len(line)]] - o
	c = mesh.points[n]
	if distance(c,o) < dist or distance(c,o) < dist:
		return None
	
	o3 = mesh.points[p]
	d3 = c - o2
	c = axispointat((o,d1), (o3,d3), dist)
	if distance(c,o) >= dist and distance(c,o) >= dist:
		return c
	else:
		c = axispointat((o,d2), (o3,d3), dist)
		if distance(c,o) >= dist and distance(c,o) >= dist:
			return c
	
def axispointat(a1, a2, dist):
	''' gives the point of axis a2 that is at `dist` distance of axis a1 '''
	# compute a base adapted to the problem
	x = a1[1]
	z = normalize(cross(x, a2[1]))
	y = cross(z,x)
	# project d (a2 direction), o (difference of origins)
	d = dot(a2[1],x), dot(a2[1],y)
	o = a2[0]-a1[0]
	# simplifications
	oo = dot(o,o)
	o = dot(o,x), dot(o,y)
	do = d[0]*o[0] + d[1]*o[1]
	dd = d[0]*d[0] + d[1]*d[1]
	# compute the participation of the a2 axis direction
	x = (- do + sqrt(do*do - dd*(oo-dist**2)))  /  dd
	return a2[0] + a2[1]*x

def axisdistance(a1, a2):
	''' gives the distance between 2 axis '''
	return abs(dot(
			normalize(cross(a1[1], a2[1])), 
			a1[0] - a2[0],
			))

if __name__ == '__main__':
	# test axispointat
	assert axispointat(
			(vec3(0,0,0), vec3(1,0,0)),
			(vec3(0,-1,1), vec3(0,1,0)),
			sqrt(2),
			) == vec3(0,1,1)
	
	# test intersections
	from generation import saddle, tube, outline, Outline
	from primitives import Arc
	import view, text
	import sys
	from PyQt5.QtWidgets import QApplication
	from nprint import nprint
	
	app = QApplication(sys.argv)
	main = scn3D = view.Scene()
	
	m = saddle(
			Outline([vec3(-2,1,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], [0,1,2,3]),
			#outline([vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)]),
			outline(Arc(vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1))),
			)
	m.options.update({'debug_display': True, 'debug_points': True })
	scn3D.objs.append(m)
	line = outline(list(m.group(1).outlines_unoriented() & m.group(2).outlines_unoriented()))
	for o,p,i in intersections(m, line, lambda *args: c_distance(1, *args)):
		scn3D.objs.append(text.Text(i * 0.05, str(o)))
	
	main.show()
	sys.exit(app.exec())
