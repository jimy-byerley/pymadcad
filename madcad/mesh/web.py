from .container import *
from .wire import Wire


class Web(NMesh):
	''' set of bipoint edges, used to represent wires
		this definition is very close to the definition of Mesh, but with edges instead of triangles
		
		Attributes:
			points:     typedlist of vec3 for points
			edges:      typedlist of couples for edges, the couple is oriented (meanings of this depends on the usage)
			tracks:     typedlist of integers giving the group each line belong to
			groups:     custom information for each group
			options:	custom informations for the entire web
	'''
	__slots__ = 'points', 'edges', 'tracks', 'groups', 'options'

	# BEGIN --- special methods ---
	
	def __init__(self, points=None, edges=None, tracks=None, groups=None, options=None):
		self.points = ensure_typedlist(points, vec3)
		self.edges = ensure_typedlist(edges, uvec2)
		self.tracks = ensure_typedlist(tracks or typedlist.full(0, len(self.edges), 'I'), 'I')
		self.groups = groups if groups is not None else [None] * (max(self.tracks, default=-1)+1)
		self.options = options or {}
		
	def __add__(self, other):
		''' return a new mesh concatenating the faces and points of both meshes '''
		if isinstance(other, Web):
			r = Web(
				self.points if self.points is other.points else self.points[:], 
				self.edges[:], 
				self.tracks[:], 
				self.groups if self.groups is other.groups else self.groups[:],
				)
			r.__iadd__(other)
			return r
		else:
			return NotImplemented
			
	def __iadd__(self, other):
		''' append the faces and points of the other mesh '''
		if isinstance(other, Web):
			if self.points is other.points:
				self.edges.extend(other.edges)
			else:
				lp = len(self.points)
				self.points.extend(other.points)
				self.edges.extend(e+lp  for e in other.edges)
			if self.groups is other.groups:
				self.tracks.extend(other.tracks)
			else:
				lt = len(self.groups)
				self.groups.extend(other.groups)
				self.tracks.extend(t+lt   for t in other.tracks)
			return self
		else:
			return NotImplemented
			
	# END BEGIN ----- data management -----
	
	def strippoints(self) -> list:
		''' remove points that are used by no faces, return the reindex list.
			if used is provided, these points will be removed without usage verification
			
			return a table of the reindex made
		'''
		self.points, self.edges, reindex = striplist(self.points, self.edges)
		return reindex
	
	def mergepoints(self, merges) -> 'self':
		''' merge points with the merge dictionnary {src index: dst index}
			merged points are not removed from the buffer.
		'''
		j = 0
		for i,f in enumerate(self.edges):
			f = uvec2(
				merges.get(f[0], f[0]),
				merges.get(f[1], f[1]),
				)
			if not f[0] == f[1]:
				self.edges[j] = f
				self.tracks[j] = self.tracks[i]
				j += 1
		del self.edges[j:]
		del self.tracks[j:]
		return self
			
	
	# END BEGIN ----- mesh checks -------
	
	def isline(self):
		''' true if each point is used at most 2 times by edges '''
		reached = typedlist.full(0, len(self.points), 'I')
		for line in self.edges:
			for p in line:	reached[p] += 1
		for r in reached:
			if r > 2:	return False
		return True
	
	def isloop(self):
		''' true if the wire form a loop '''
		return len(self.extremities()) == 0
	
	def check(self):
		''' check that the internal data references are good (indices and list lengths) '''
		if not (isinstance(self.points, typedlist) and self.points.dtype == vec3):	raise MeshError("points must be a typedlist(dtype=vec3)")
		if not (isinstance(self.edges, typedlist) and self.edges.dtype == uvec2): 	raise MeshError("edges must be in a typedlist(dtype=uvec2)")
		if not (isinstance(self.tracks, typedlist) and self.tracks.dtype == 'I'): 	raise MeshError("tracks must be in a typedlist(dtype='I')")
		l = len(self.points)
		for line in self.edges:
			for p in line:
				if p >= l:	raise MeshError("some indices are greater than the number of points", line, l)
				if p < 0:	raise MeshError("point indices must be positive", line)
			if line[0] == line[1]:	raise MeshError("some edges use the same point multiple times", line)
		if len(self.edges) != len(self.tracks):	raise MeshError("tracks list doesn't match edge list length")
		if max(self.tracks, default=-1) >= len(self.groups): raise MeshError("some line group indices are greater than the number of groups", max(self.tracks, default=-1), len(self.groups))
	
	
	
	# END BEGIN ----- selection methods -------
	
	def groupnear(self, point: vec3) -> int:
		''' return group id if the edge the closest to the given point '''
		return self.tracks[self.edgenear(point)]
	
	def edgenear(self, point: vec3) -> int:
		''' return the index of the closest edge to the given point '''
		return min( range(len(self.edges)),
					lambda i: distance_pe(point, self.edgepoints(i)) )
	
	def group(self, quals) -> 'Self':
		''' extract a part of the mesh corresponding to the designated groups.
			
			Groups can be be given in either the following ways:
				- a set of group indices
					
					This can be useful to combine with other functions. However it can be difficult for a user script to keep track of which index correspond to which created group
					
				- an iterable of group qualifiers
				
					This is the best way to designate groups, and is meant to be used in combination with `self.qual()`.
					This mode selects every group having all the input qualifiers
					
			Example:
			
				>>> # create a mesh with only the given groups
				>>> mesh.group({1, 3, 8, 9})   
				<Mesh ...>
				
				>>> # create a mesh with all the groups having the following qualifiers
				>>> mesh.group(['extrusion', 'arc'])   
				<Mesh ...>
		'''
		edges = typedlist(dtype=uvec2)
		tracks = typedlist(dtype='I')
		for i in self.qualified_indices(quals):
			edges.append(self.edges[i])
			tracks.append(self.tracks[i])
		return Web(self.points, edges, tracks, self.groups)
	
	def replace(self, mesh, groups=None) -> 'self':
		''' replace the given groups by the given mesh.
			If groups is not specified, it will take the matching groups (with same index) in the current mesh
		'''
		if groups:
			groups = set(self.qualified_groups(groups))
		else:
			groups = set(mesh.tracks)
		j = 0
		for i,t in enumerate(self.tracks):
			if t not in groups:
				self.edges[j] = self.edges[i]
				self.tracks[j] = t
				j += 1
		del self.edges[j:]
		del self.tracks[j:]
		self += mesh
		return self
	
	# END BEGIN --- extraction methods ---
	
	def edgepoints(self, e) -> tuple:
		''' tuple of the points for edge `e`
		
			the edge can be given using its index in `self.edges` or using a tuple of point idinces
		'''
		if isinstance(e, Integral):	e = self.edges[e]
		return self.points[e[0]], self.points[e[1]]
	
	def edgedirection(self, e) -> vec3:
		''' direction of edge e 
			
			the edge can be given using its index in `self.edges` or using a tuple of point idinces
		'''
		if isinstance(e, Integral):	e = self.edges[e]
		return normalize(self.points[e[1]] - self.points[e[0]])
	
	def flip(self) -> 'Web':
		''' reverse direction of all edges '''
		return Web(	self.points, 
					typedlist((uvec2(b,a)  for a,b in self.edges), dtype=uvec2), 
					self.tracks, 
					self.groups)
		
	def segmented(self, group=None) -> 'Web':
		''' return a copy of the mesh with a group each edge 
		
			if group is specified, it will be the new definition put in each groups
		'''
		return Web(self.points, self.edges,
					typedlist(range(len(self.edges)), dtype='I'),
					[group]*len(self.edges),
					self.options,
					)
	
	def extremities(self) -> set:
		''' return the points that are used once only (so at wire terminations)
			1D equivalent of Mesh.outlines()
		'''
		extr = set()
		for l in self.edges:
			for p in l:
				if p in extr:	extr.remove(p)
				else:			extr.add(p)
		return extr
	
	def groupextremities(self) -> 'Wire':
		''' return the extremities of each group.
			1D equivalent of Mesh.groupoutlines()
			
			On a frontier between multiple groups, there is as many points as groups, each associated to a group.
		'''
		indices = typedlist(dtype='I')
		tracks = []
		tmp = {}
		# insert points belonging to different groups
		for i,edge in enumerate(self.edges):
			track = self.tracks[i]
			for p in edge:
				if p in tmp:
					if tmp[p] != track:
						indices.append(p)
						tracks.append(track)
					del tmp[p]
				else:
					tmp[p] = track
		indices.extend(tmp.keys())
		tracks.extend(tmp.values())
		return Wire(self.points, indices, tracks, self.groups)
		
	def frontiers(self, *args) -> 'Wire':
		''' return a Wire of points that split the given groups appart.
		
			The arguments are groups indices or lists of group qualifiers (as set in `qualify()`). If there is one only argument it is considered as as list of arguments.
		
			- if no argument is given, then return the frontiers between every groups
			- to include the groups extremities that are on the group border but not at the frontier with an other group, add `None` to the group set
			
			Example:
			
				>>> w = Web([...], [uvec2(0,1), uvec2(1,2)], [0, 1], [...])
				>>> w.frontiers(0,1).indices
				[1]
				
				>>> # equivalent to
				>>> w.frontiers({0,1}).indices
				[1]
				
				>>> w.frontiers(0,None).indices
				[0]
		'''
		if args:
			if len(args) == 1 and hasattr(args[0], '__iter__'):
				args = args[0]
			groups = set()
			for arg in args:
				if arg is None:		groups.add(None)
				else:				groups.update(self.qualified_groups(arg))
		else:
			groups = None
		
		indices = typedlist(dtype='I')
		tracks = typedlist(dtype='I')
		couples = OrderedDict()
		belong = {}
		for i,edge in enumerate(self.edges):
			track = self.tracks[i]
			if groups and track not in groups:	continue
			for p in edge:
				if p in belong:
					if belong[p] != track:
						g = edgekey(belong[p],track)
						indices.append(p)
						tracks.append(couples.setdefault(g, len(couples)))
					del belong[p]
				else:
					belong[p] = track
		if groups and None in groups:
			indices.extend(belong.keys())
		return Wire(self.points, indices, tracks, list(couples))
		
		
	
	def length(self) -> float:
		''' total length of edges '''
		s = 0
		for e in self.edges:
			s += distance(self.points[e[1]], self.points[e[0]])
		return s
	
	def surface(self) -> float:
		''' return the surface enclosed by the web if planar and is composed of loops (else it has no meaning) '''
		o = self.barycenter()
		s = vec3(0)
		for e in self.edges:
			s += cross(self.points[e[1]] - o, self.points[e[0]] - o)
		return length(s)
	
	def barycenter(self) -> vec3:
		''' curve barycenter of the mesh '''
		if not self.edges:	return vec3(0)
		acc = vec3(0)
		tot = 0
		for e in self.edges:
			a,b = self.edgepoints(e)
			weight = distance(a,b)
			tot += weight
			acc += weight*(a+b)
		return acc / (2*tot)
		
	def assignislands(self) -> 'Web':
		conn = connpe(self.edges)
		reached = [False] * len(self.edges)
		stack = []
		start = 0
		group = 0
		while True:
			# search start point
			for start in range(start,len(reached)):
				if not reached[start]:
					stack.append(start)
					break
			# end when everything reached
			if not stack:	break
			# propagate
			island = Web(points=self.points, groups=self.groups)
			while stack:
				i = stack.pop()
				if reached[i]:	continue	# make sure this face has not been stacked twice
				reached[i] = True
				yield i, group
				for p in self.edges[i]:
					stack.extend(n	for n in conn[p] if not reached[n])
			group += 1
	
	def groupislands(self) -> 'Web':
		''' return the same web but with a new group each island '''
		tracks = typedlist.full(0, len(self.edges), dtype='I')
		track = 0
		for edge, track in self.assignislands():
			tracks[edge] = track
		return Web(self.points, self.edges, tracks, [None]*(track+1))
		
	def islands(self) -> '[Web]':
		''' return the unconnected parts of the mesh as several meshes '''
		islands = []
		island = Web(points=self.points, groups=self.groups)
		for edge, group in self.assignislands():
			if group > len(islands):
				islands.append(island)
				island = Web(points=self.points, groups=self.groups)
			else:
				island.edges.append(self.edges[edge])
				island.tracks.append(self.tracks[edge])
		islands.append(island)
		return islands
	
	def arcs(self) -> '[Wire]':
		''' return the contiguous portions of this web '''
		return [	Wire(self.points, typedlist(loop, dtype=uvec2))		
					for loop in suites(self.edges, oriented=False)]
		
	# END BEGIN ----- output methods ------
	
	def __repr__(self):
		return '<Web with {} points at 0x{:x}, {} edges>'.format(len(self.points), id(self.points), len(self.edges))
	
	def __str__(self):
		return 'Web(\n  points={},\n  edges={},\n  tracks={},\n  groups={},\n  options={})'.format(
					reprarray(self.points, 'points'),
					reprarray(self.edges, 'edges'),
					reprarray(self.tracks, 'tracks'),
					reprarray(self.groups, 'groups'),
					repr(self.options))
					
	def display(self, scene):		
		from .. import displays
		
		points = typedlist(dtype=vec3)
		idents = typedlist(dtype='I')
		edges = typedlist(dtype=uvec2)
		frontiers = typedlist(dtype='I')
		def usept(pi, ident, used):
			if used[pi] >= 0:	
				return used[pi]
			else:
				used[pi] = i = len(points)
				points.append(self.points[pi])
				idents.append(ident)
				return i
		
		for group in range(len(self.groups)):
			used = [-1]*len(self.points)
			frontier = set()
			for edge,track in zip(self.edges, self.tracks):
				if track != group:	continue
				edges.append(uvec2(usept(edge[0], track, used), usept(edge[1], track, used)))
				for p in edge:
					if p in frontier:	frontier.remove(p)
					else:				frontier.add(p)
			for p in frontier:
				frontiers.append(used[p])
				
		if not points or not edges:
			return displays.Display()
		
		return displays.WebDisplay(scene,
				typedlist_to_numpy(points, 'f4'), 
				typedlist_to_numpy(edges, 'u4'),
				typedlist_to_numpy(frontiers, 'u4'),
				typedlist_to_numpy(idents, 'u4'),
				color=self.options.get('color'))
				
	# END


