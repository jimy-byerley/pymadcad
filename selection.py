from mathutils import vec3, anglebt
from mesh import Mesh, Web, edgekey

__all__ = ['select', 'stopangle', 'crossover', 'straight', 'short', 'SelExpr']

def select(mesh, edge, stopleft=None, stopright=False, conn=None, web=None) -> Web:
	''' Select edges by propagation across group outlines, until the stop criterion is verified
		If a criterion is None, the propagation select everything it reachs.
		If stopright is not specified (False), stopleft is used for both directions.
		
		example:
			select(m, (12,37), angle(pi/2) | crossover)
	'''
	# manage arguments
	if isinstance(mesh, Mesh) and web is None:
		web = mesh.groupoutlines()
	elif isinstance(mesh, Web):
		web = mesh
		mesh = None
	if conn is None:
		conn = connectivity(web.lines)
	if stopleft is None:		stopleft = lambda *args: False
	if stopright is None:		stopright = lambda *args: False
	elif stopright is False:	stopright = stopleft
	# selection by propagation on each side
	return Web(web.points, list(
			  selectside(mesh, web, conn, edge, stopleft) 
			| selectside(mesh, web, conn, (edge[1], edge[0]), stopright)
			))

def selectside(mesh, web, conn, edge, stop):
	''' selection by propagation until stop criterion '''
	front = [edge]
	seen = set()
	while front:
		last,curr = edge = front.pop()
		e = edgekey(*edge)
		if e in seen:	continue
		seen.add(e)
		for next in conn[curr]:
			if not stop(mesh,web,conn,last,curr,next):
				front.append((curr,next))
	return seen

def connectivity(lines):
	''' point to point connectivity '''
	conn = {}
	for line in lines:
		for a,b in ((line[0],line[1]), (line[1],line[0])):
			if a not in conn:		conn[a] = [b]
			else:					conn[a].append(b)
	return conn
		


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
	return SelExpr(lambda mesh,web,conn,last,curr,next: anglebt(web.points[curr]-web.points[last], web.points[next]-web.points[curr]) > maxangle)

@SelExpr
def crossover(mesh,web,conn,last,curr,next): 
	return len(conn[curr]) > 2

@SelExpr
def straight(mesh,web,conn,last,curr,next):
	start = web.points[curr] - web.points[last]
	other = min(conn[curr], key=lambda o: anglebt(start, web.points[o] - web.points[curr]))
	return other != next

@SelExpr
def short(mesh,web,conn,last,curr,next):
	start = web.points[curr] - web.points[last]
	best = None
	score = 0
	for o in conn[curr]:
		if o == last:	continue
		angle = anglebt(start, web.points[o] - web.points[curr])
		if angle > score:
			score = angle
			best = o
	return best != next


if __name__ == '__main__':
	import sys
	from PyQt5.QtWidgets import QApplication
	import view
	
	m = Mesh(
		[vec3(1,1,0), vec3(-1,1,0), vec3(-1,-1,0), vec3(1,-1,0),    vec3(2,-1,0), vec3(2,1,0),   vec3(-2,1,0), vec3(-2,-1,0)],
		[(0,1,2),(0,2,3), (4,0,3),(0,4,5),  (6,2,1),(2,6,7)],
		[0,0, 1,1, 2,2],
		)
	print('all\t', select(m, (0,1)).lines)
	print('crossover\t', select(m, (0,1), crossover).lines)
	print('stopangle\t', select(m, (0,1), stopangle(0.1)).lines)
	print('straight\t', select(m, (0,1), straight).lines)
	print('short\t', select(m, (0,1), short).lines)
	print('short | straight\t', select(m, (0,1), short | straight).lines)
	
	m.options.update({'debug_display':True, 'debug_points':True})
	app = QApplication(sys.argv)
	scn3D = view.Scene()
	scn3D.add(m)
	scn3D.look(m.box())
	scn3D.show()
	sys.exit(app.exec())
