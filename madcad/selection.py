# This file is part of pymadcad,  distributed under license LGPL v3

import math
from .mathutils import (
		vec3, uvec2,
		anglebt, cross, dot, length, distance, normalize, noproject,
		distance_pp, distance_pa, distance_pe, distance_aa, distance_ae,
		)
from .mesh import Mesh, Web, edgekey, connpp, connef

__all__ = ['select', 'stopangle', 'crossover', 'straight', 'short', 'selexpr', 'edgenear']

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
		conn = connpp(web.edges)
	if stopleft is None:		stopleft = lambda *args: False
	if stopright is None:		stopright = lambda *args: False
	elif stopright is False:	stopright = stopleft
	
	if isinstance(edge, vec3):
		edge = edgenear(web, edge)
	
	assert edge[0] in conn and edge[1] in conn[edge[0]], "the given edge doesn't exist"
	# selection by propagation on each side
	shared = {'mesh':mesh, 'web':web, 'conn':conn}
	seen = set()
	selectside(shared, edge, stopleft, seen, True) 
	selectside(shared, edge, stopleft, seen, False) 
	return Web(web.points, list(seen))

def selectside(shared, edge, stop, seen=None, revert=None):
	''' selection by propagation until stop criterion '''
	front = [edge if not revert else uvec2(edge[1],edge[0])]
	if seen is None:	
		seen = set()
	else:				
		seen.discard(edge)
		seen.discard(uvec2(edge[1], edge[0]))
	conn = shared['conn']
	while front:
		last,curr = front.pop()
		if   uvec2(last,curr) in seen:		continue
		elif uvec2(curr,last) in seen:		continue
		if revert:	seen.add(uvec2(curr,last))
		else:		seen.add(uvec2(last,curr))
		connected = conn[curr]
		if len(connected) > 1:
			for next in connected:
				if not stop(shared,last,curr,next):
					front.append(uvec2(curr,next))
		else:
			front.append(uvec2(curr,connected[0]))
	return seen


class selexpr(object):
	''' boolean expression for topology condition, used to define stop expressions for select() '''
	__slots__ = 'func',
	
	def __init__(self, func):
		self.func = func
	
	def __and__(self, other):	return selexpr(lambda *args: self.func(*args) and other.func(*args))
	def __or__(self, other):	return selexpr(lambda *args: self.func(*args) or other.func(*args))
	def __xor__(self, other):	return selexpr(lambda *args: bool(self.func(*args) ^ other.func(*args)))
	def __not__(self, other):	return selexpr(lambda *args: not self.func(*args))
	def __call__(self, *args):	return self.func(*args)
	
	def __repr__(self):
		return 'selexpr({})'.format(self.func)

# --- stop conditions ---

def stopangle(maxangle):
	''' stop when angle between consecutive edges is bigger than maxangle '''
	def stop(shared,last,curr,next): 
		points = shared['web'].points
		return anglebt(points[curr]-points[last], points[next]-points[curr]) >= maxangle
	return selexpr(stop)

@selexpr
def crossover(shared,last,curr,next):
	return len(shared['conn'][curr]) > 2

@selexpr
def straight(shared,last,curr,next):
	points = shared['web'].points
	start = points[curr] - points[last]
	other = min(shared['conn'][curr], key=lambda o: anglebt(start, points[o] - points[curr]))
	return other != next

@selexpr
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
	return selexpr(stop)
	
# --- edge selection from mesh ---

def edgenear(web, obj):
	if isinstance(obj,vec3):	dist = lambda e: distance_pe(obj,e)
	elif isinstance(obj,tuple):	dist = lambda e: distance_ae(obj,e)
	else:
		raise TypeError("obj must be a point or an axis")
	if isinstance(web, Mesh):	web = web.groupoutlines()
	best = None
	score = math.inf
	for edge in web.edges:
		d = dist((web.points[edge[0]], web.points[edge[1]]))
		if d < score:
			score = d
			best = edge
	if isinstance(obj,tuple) and dot(web.points[best[1]]-web.points[best[0]], obj[1]) < 0:
		best = best[1],best[0]
	return best
