# This file is part of pymadcad,  distributed under license LGPL v3

from copy import copy, deepcopy
from random import random
import numpy as np
import numpy.lib.recfunctions as rfn
from array import array
from collections import OrderedDict, Counter
from numbers import Integral, Real
import math

from ..mathutils import *
from ..asso import Asso
from .. import hashing
from .. import settings


class MeshError(Exception):	
	''' inconsistent data in mesh '''
	pass


class NMesh(object):
	''' common methods for points container (typically Mesh, Web, Wire) '''
	
	# BEGIN --- data management ---
	
	def own(self, **kwargs) -> 'Self':
		''' return a copy of the current mesh, which attributes are referencing the original data or duplicates if demanded
		
			Example:
			
				>>> b = a.own(points=True, faces=False)
				>>> b.points is a.points
				False
				>>> b.faces is a.faces
				True
		'''
		new = copy(self)
		for name, required in kwargs.items():
			if required:
				setattr(new, name, deepcopy(getattr(self, name)))
		return new
	
	def option(self, **kwargs) -> 'self':
		''' update the internal options with the given dictionnary and the keywords arguments.
			This is only a shortcut to set options in a method style.
		'''
		self.options.update(kwargs)
		return self
	
	def transform(self, trans) -> 'Self':
		''' apply the transform to the points of the mesh, returning the new transformed mesh'''
		trans = transformer(trans)
		transformed = copy(self)
		transformed.points = typedlist((trans(p) for p in self.points), dtype=vec3)
		return transformed
			
	def mergeclose(self, limit=None) -> dict:
		''' merge points below the specified distance, or below the precision 
			return a dictionnary of points remapping  {src index: dst index}
			
			O(n) implementation thanks to hashing
		'''
		if limit is None:	limit = self.precision()
		
		merges = {}
		points = hashing.PointSet(limit)
		for i in range(len(self.points)):
			used = points.add(self.points[i])
			if used != i:	merges[i] = used
		self.points = points.points
		self.mergepoints(merges)
		return merges
	
	def stripgroups(self) -> list:
		''' remove groups that are used by no faces. return the reindex list. '''
		self.groups, self.tracks, reindex = striplist(self.groups, self.tracks)
		return reindex
	
	def mergegroups(self, defs=None, merges=None) -> 'self':
		''' merge the groups according to the merge dictionnary
			the new groups associated can be specified with defs
			the former unused groups are not removed from the buffer and the new ones are appended
			
			if merges is not provided, all groups are merged, and defs is the data associated to the only group after the merge
		'''
		if merges is None:	
			self.groups = [defs]
			self.tracks = typedlist.full(0, len(self.tracks), 'I')
		else:
			if defs:
				l = len(self.groups)
				self.groups.extend(defs)
			else:
				l = 0
			for i,t in enumerate(self.tracks):
				if t in merges:
					self.tracks[i] = merges[t]+l
		return self
	
	def finish(self) -> 'self':
		''' finish and clean the mesh 
			note that this operation can cost as much as other transformation operation
			job done
				- mergeclose
				- strippoints
				- stripgroups
				- check
		'''
		self.mergeclose()
		self.strippoints()
		self.stripgroups()
		self.check()
		return self
	
	# END BEGIN --- verification methods ---
		
	def isvalid(self):
		''' return true if the internal data is consistent (all indices referes to actual points and groups) '''
		try:				self.check()
		except MeshError:	return False
		else:				return True
	
	# END BEGIN --- selection methods ---
	
	def pointat(self, point: vec3, neigh=NUMPREC) -> int:
		''' return the index of the first point at the given location, or None '''
		for i,p in enumerate(self.points):
			if distance(p,point) <= neigh:	return i
	
	def pointnear(self, point: vec3) -> int:
		''' return the nearest point the the given location '''
		return min(	range(len(self.points)), 
					lambda i: distance(self.points[i], point))
	
	def qualify(self, *quals, select=None, replace=False) -> 'self':
		''' set a new qualifier for the given groups 
		
			Parameters:
				quals:				the qualifiers to enable for the selected mesh groups
				select (iterable):	if specified, only the groups having all those qualifiers will be added the new qualifiers
				replace (bool):		if True, the qualifiers in select will be removed
				
			Example:
			
				>>> pool = meshb.qualify('part-a') + meshb.qualify('part-b')
				>>> set(meshb.faces) == set(pool.group('part-b').faces)
				True
				
				>>> chamfer(mesh, ...).qualify('my-group', select='chamfer', replace=True)
				>>> mesh.group('my-group')
				<Mesh ...>
		'''
		if select is None:	it = range(len(self.groups))
		else:				it = self.qualified_groups(select)
		for i in it:
			group = self.groups[i]
			if group is None:
				self.groups[i] = group = {}
			if not hasattr(group, '__setitem__'):
				raise TypeError('cannot qualify a group that is not a dictionnary')
			if replace:
				for key in select:
					if key in group:
						del group[key]
			for key in quals:
				group[key] = None
	
		return self
		
	def qualified_indices(self, quals):
		''' yield the faces indices when their associated group are matching the requirements '''
		if isinstance(quals, int):
			yield from self.qualified_indices({quals})
		elif isinstance(quals, str):
			yield from self.qualified_indices([quals])
		
		elif not quals:
			yield from range(len(self.tracks))
		elif isinstance(next(iter(quals)), int):
			if not isinstance(quals, set):	quals = set(quals)
			for i,t in enumerate(self.tracks):
				if t in quals:
					yield i
		else:
			inclusive = None in quals
			seen = {}
			for i,t in enumerate(self.tracks):
				if t in seen:
					ok = seen[t]
				elif self.groups[t]:
					ok = seen[t] = all(k in self.groups[t]   for k in quals)
				else:
					ok = inclusive
				if ok:
					yield i
				
	def qualified_groups(self, quals):
		''' yield the groups indices when they are matching the requirements '''
		if isinstance(quals, int):
			yield quals
		elif isinstance(quals, str):
			yield from self.qualified_groups([quals])
		
		elif not quals:
			yield from range(len(self.groups))
		elif isinstance(next(iter(quals)), int):
			yield from quals
		else:
			inclusive = None in quals
			for i, group in enumerate(self.groups):
				if group and all(key in group   for key in quals) or inclusive:
					yield i
	
	# END BEGIN --- extraction methods ---
	
	def maxnum(self) -> float:
		''' maximum numeric value of the mesh, use this to get an hint on its size or to evaluate the numeric precision '''
		m = 0
		for p in self.points:
			for v in p:
				a = abs(v)
				if a > m:	m = a
		return m
	
	def precision(self, propag=3) -> float:
		''' numeric coordinate precision of operations on this mesh, allowed by the floating point precision '''
		return self.maxnum() * NUMPREC * (2**propag)
		
	def usepointat(self, point, neigh=NUMPREC) -> int:
		''' Return the index of the first point in the mesh at the location. If none is found, insert it and return the index '''
		i = self.pointat(point, neigh=neigh)
		if i is None:
			i = len(self.points)
			self.points.append(point)
		return i
					
	def box(self) -> Box:
		''' return the extreme coordinates of the mesh (vec3, vec3) '''
		if not self.points:		return Box()
		max = deepcopy(self.points[0])
		min = deepcopy(self.points[0])
		for pt in self.points:
			for i in range(3):
				if   pt[i] < min[i]:	min[i] = pt[i]
				elif pt[i] > max[i]:	max[i] = pt[i]
		return Box(min, max)
	
	def barycenter_points(self) -> vec3:
		''' barycenter of points used '''
		return sum(self.points[i]	for i in self.indices) / len(self.indices)
		
	# END





def numpy_to_typedlist(array: 'ndarray', dtype) -> 'typedlist':
	''' convert a numpy.ndarray into a typedlist with the given dtype, if the conversion is possible term to term '''
	ndtype = np.array(typedlist(dtype)).dtype
	if ndtype.fields:
		return typedlist(rfn.unstructured_to_structured(array).astype(ndtype, copy=False), dtype)
	else:
		return typedlist(array.astype(ndtype, copy=False), dtype)
	
def typedlist_to_numpy(array: 'typedlist', dtype) -> 'ndarray':
	''' convert a typedlist to a numpy.ndarray with the given dtype, if the conversion is possible term to term '''
	tmp = np.array(array, copy=False)
	if tmp.dtype.fields:
		return rfn.structured_to_unstructured(tmp, dtype)
	else:
		return tmp.astype(dtype)
		
def ensure_typedlist(obj, dtype):
	''' return a typedlist with the given dtype, create it from whatever is in obj if needed '''
	if isinstance(obj, typedlist) and obj.dtype == dtype:
		return obj
	else:
		return typedlist(obj, dtype)




# ------ connectivity tools -------
		

def edgekey(a,b):
	''' return a key for a non-directional edge '''
	if a < b:	return (a,b)
	else:		return (b,a)
	
def facekeyo(a,b,c):
	''' return a key for an oriented face '''
	if a < b and b < c:		return (a,b,c)
	elif a < b:				return (c,a,b)
	else:					return (b,c,a)
	
def arrangeface(f, p):
	''' return the face indices rotated the way the `p` is the first one '''
	if   p == f[1]:	return f[1],f[2],f[0]
	elif p == f[2]:	return f[2],f[0],f[1]
	else:			return f
	
def arrangeedge(e, p):
	if p == e[1]:	return e[1], e[0]
	else:			return e

def connpp(ngons):
	''' point to point connectivity 
		input is a list of ngons (tuple of 2 to n indices)
	'''
	conn = {}
	for loop in ngons:
		for i in range(len(loop)):
			for a,b in ((loop[i-1],loop[i]), (loop[i],loop[i-1])):
				if a not in conn:		conn[a] = [b]
				elif b not in conn[a]:	conn[a].append(b)
	return conn
	
def connef(faces):
	''' connectivity dictionnary, from oriented edge to face '''
	conn = {}
	for i,f in enumerate(faces):
		for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0])):
			conn[e] = i
	return conn
	
def connpe(edges):
	conn = Asso()
	for i,edge in enumerate(edges):
		for p in edge:
			conn.add(p,i)
	return conn


def connexity(links):
	''' return the number of links referencing each point as a dictionnary {point: num links} '''
	reach = {}
	for l in links:
		for p in l:
			reach[p] = reach.get(p,0) +1
	return reach

def suites(lines, oriented=True, cut=True, loop=False):
	''' return a list of the suites that can be formed with lines.
		lines is an iterable of edges
		
		Parameters:
			oriented:      specifies that (a,b) and (c,b) will not be assembled
			cut:           cut suites when they are crossing each others
		
		return a list of the sequences that can be formed
	'''
	lines = list(lines)
	# get contiguous suite of points
	suites = []
	while lines:
		suite = list(lines.pop())
		found = True
		while found:
			found = False
			for i,edge in enumerate(lines):
				if edge[-1] == suite[0]:		suite[0:1] = edge
				elif edge[0] == suite[-1]:		suite[-1:] = edge
				# for unoriented lines
				elif not oriented and edge[0] == suite[0]:		suite[0:1] = reversed(edge)
				elif not oriented and edge[-1] == suite[-1]:	suite[-1:] = reversed(edge)
				else:
					continue
				lines.pop(i)
				found = True
				break
			if loop and suite[-1] == suite[0]:	break
		suites.append(suite)
	# cut at suite intersections (sub suites or crossing suites)
	if cut:
		reach = {}
		for suite in suites:
			for p in suite:
				reach[p] = reach.get(p,0) + 1
		for suite in suites:
			for i in range(1,len(suite)-1):
				if reach[suite[i]] > 1:
					suites.append(suite[i:])
					suite[i+1:] = []
					break
	return suites



# ----- internal helpers ------


def reprarray(array, name):
	content = ', '.join((repr(e) for e in array))
	return '['+content+']'

def striplist(points, indices):
	used = [False] * len(points)
	for index in indices:
		if isinstance(index, int):	
			used[index] = True
		else:
			for p in index:
				used[p] = True
	optimized = typedlist(dtype=points.dtype)  if isinstance(points, typedlist) else []
	reindices = typedlist(dtype=indices.dtype)
	reindex = typedlist(dtype='i', reserve=len(points))
	for p, u in zip(points, used):
		if u:
			reindex.append(len(optimized))
			optimized.append(p)
		else:
			reindex.append(-1)
	for index in indices:
		if isinstance(index, int):	
			reindices.append(reindex[index])
		else:
			reindices.append([reindex[i]   for i in index])
	return optimized, reindices, reindex
