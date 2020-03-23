import math
from mathutils import vec3, anglebt, cross, dot, length, distance, normalize, noproject
from mesh import Mesh, Web, edgekey, connpp, connef

__all__ = ['select', 'stopangle', 'crossover', 'straight', 'short', 'SelExpr']

def select(mesh, edge, stopleft=None, stopright=False, conn=None, web=None) -> Web:
	''' Select edges by propagation across group outlines, until the stop criterion is verified
		If a criterion is None, the propagation select everything it reachs.
		If stopright is not specified (False), stopleft is used for both directions.
		
		example:
			select(m, (12,37), stopangle(pi/2) | crossover)
	'''
	# manage arguments
	if isinstance(mesh, Mesh) and web is None:
		web = mesh.groupoutlines()
	elif isinstance(mesh, Web):
		web = mesh
		mesh = None
	if conn is None:
		conn = connpp(web.lines)
	if stopleft is None:		stopleft = lambda *args: False
	if stopright is None:		stopright = lambda *args: False
	elif stopright is False:	stopright = stopleft
	
	assert edge[0] in conn and edge[1] in conn[edge[0]], "the given edge doesn't exist"
	# selection by propagation on each side
	shared = {'mesh':mesh, 'web':web, 'conn':conn}
	return Web(web.points, list(
			  selectside(shared, edge, stopleft) 
			| selectside(shared, (edge[1], edge[0]), stopright)
			))

def selectside(shared, edge, stop):
	''' selection by propagation until stop criterion '''
	front = [edge]
	seen = set()
	conn = shared['conn']
	while front:
		last,curr = edge = front.pop()
		e = edgekey(*edge)
		if e in seen:	continue
		seen.add(e)
		connected = conn[curr]
		if len(connected) > 1:
			for next in connected:
				if not stop(shared,last,curr,next):
					front.append((curr,next))
		else:
			front.append((curr,connected[0]))
	return seen


class SelExpr(object):
	''' boolean expression for topology condition, used to define stop expressions for select() '''
	__slots__ = 'func',
	
	def __init__(self, func):
		self.func = func
	
	def __and__(self, other):	return SelExpr(lambda *args: self.func(*args) and other.func(*args))
	def __or__(self, other):	return SelExpr(lambda *args: self.func(*args) or other.func(*args))
	def __xor__(self, other):	return SelExpr(lambda *args: bool(self.func(*args) ^ other.func(*args)))
	def __not__(self, other):	return SelExpr(lambda *args: not self.func(*args))
	def __call__(self, *args):	return self.func(*args)

# --- stop conditions ---

def stopangle(maxangle):
	''' stop when angle between consecutive edges is bigger than maxangle '''
	def stop(shared,last,curr,next): 
		points = shared['web'].points
		return anglebt(points[curr]-points[last], points[next]-points[curr]) >= maxangle
	return SelExpr(stop)

@SelExpr
def crossover(shared,last,curr,next): 
	return len(shared['conn'][curr]) > 2

@SelExpr
def straight(shared,last,curr,next):
	points = shared['web'].points
	start = points[curr] - points[last]
	other = min(shared['conn'][curr], key=lambda o: anglebt(start, points[o] - points[curr]))
	return other != next

@SelExpr
def short(shared,last,curr,next):
	points = shared['web'].points
	start = points[curr] - points[last]
	best = None
	score = 0
	for o in shared['conn'][curr]:
		if o == last:	continue
		angle = anglebt(start, points[o] - points[curr])
		if angle > score:
			score = angle
			best = o
	return best != next

def faceangle(minangle):
	''' stop when angle between adjacent faces is strictly lower than minangle '''
	def stop(shared,last,curr,next):
		mesh = shared['mesh']
		if 'connef' not in shared:
			shared['connef'] = connef = connef(context['mesh'])
		else:
			connef = shared['connef']
		if (curr,next) in connef and (next,curr) in connef:
			nl = mesh.facenormal(connef[(curr,next)])
			nr = mesh.facenormal(connef[(next,curr)])
			return anglebt(nl, nr) <= minangle
		else:
			return True
	return SelExpr(stop)
	
# --- edge selection from mesh ---

distance_pp = distance

def distance_pa(pt, axis):
	return length(noproject(pt-axis[0], axis[1]))

def distance_pe(pt, edge):
	dir = edge[1]-edge[0]
	l = length(dir)
	x = dot(pt-edge[0], dir)/l**2
	if   x < 0:	return distance(pt,edge[0])
	elif x > 1:	return distance(pt,edge[1])
	else:
		v = pt-edge[0]
		return length(v - v*dir)/l

def distance_aa(a1, a2):
	return dot(a1[0]-a2[0], normalize(cross(a1[1], a2[1])))

def distance_ae(axis, edge):
	d = normalize(edge[1]-edge[0])
	y = noproject(d, d)
	s1 = dot(edge[0]-axis[0], y)
	s2 = dot(edge[1]-axis[0], y)
	if s1*s2 < 0:
		z = normalize(cross(axis[1], d))
		return dot(edge[0]-axis[1], z)
	elif abs(s1) < abs(s2):
		return distance_pa(edge[0], axis)
	else:
		return distance_pa(edge[1], axis)

def edgenear(web, obj):
	if isinstance(obj,vec3):	dist = lambda e: distance_pe(obj,e)
	elif isinstance(obj,tuple):	dist = lambda e: distance_ae(obj,e)
	else:
		raise TypeError("obj must be a point or an axis")
	if isinstance(web, Mesh):	web = web.groupoutlines()
	best = None
	score = math.inf
	for edge in web.lines:
		d = dist((web.points[edge[0]], web.points[edge[1]]))
		if d < score:
			score = d
			best = edge
	if isinstance(obj,tuple) and dot(web.points[best[1]]-web.points[best[0]], obj[1]) < 0:
		best = best[1],best[0]
	return best
	

if __name__ == '__main__':
	import sys
	from PyQt5.QtWidgets import QApplication
	import view
	
	m = Mesh(
		[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0),    vec3(2,-1,0), vec3(2,1,0),    vec3(-2,1,0), vec3(-2,-1,0)],
		[(0,1,2),(0,2,3), (4,0,3),(0,4,5),  (6,2,1),(2,6,7)],
		[0,0, 1,1, 2,2],
		)
	print('all\t', select(m, (0,1)).lines)
	print('crossover\t', select(m, (0,1), crossover).lines)
	print('stopangle\t', select(m, (0,1), stopangle(0.1)).lines)
	print('straight\t', select(m, (0,1), straight).lines)
	print('short\t', select(m, (0,1), short).lines)
	print('short | straight\t', select(m, (0,1), short | straight).lines)
	print('edgenear', edgenear(m, (vec3(0,2,0), vec3(1,1,0))))
	
	#m.options.update({'debug_display':True, 'debug_points':True})
	#app = QApplication(sys.argv)
	#scn3D = view.Scene()
	#scn3D.add(m)
	#scn3D.look(m.box())
	#scn3D.show()
	#sys.exit(app.exec())
