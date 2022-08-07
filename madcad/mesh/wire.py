from .container import *


class Wire(NMesh):
	''' Line as continuous suite of points 
		Used to borrow reference of points from a mesh by keeping their original indices

		Attributes:
			points:	    points buffer
			indices:	indices of the line's points in the buffer
			tracks:	    group index for each point in indices
						it can be used to associate groups to points or to edges (if to edges, then take care to still have as many track as indices)
			groups:	    data associated to each point (or edge)
			options:	custom informations for the entire wire
	'''
	__slots__ = 'points', 'indices', 'tracks', 'groups', 'options'
	
	# BEGIN ----- special methods -----
	
	def __init__(self, points=None, indices=None, tracks=None, groups=None, options=None):
		self.points = ensure_typedlist(points, vec3)
		self.indices = ensure_typedlist(indices if indices is not None else range(len(self.points)), 'I')
		self.tracks = tracks if tracks is not None else None
		self.groups = groups if groups is not None else [None]
		self.options = options or {}
	
	def __len__(self):	return len(self.indices)
	def __iter__(self):	return (self.points[i] for i in self.indices)
	def __getitem__(self, i):
		''' return the ith point of the wire, useful to use the wire in a same way as list of points
		
			equivalent to `self.points[self.indices[i]]` 
		'''
		if isinstance(i, Integral):		return self.points[self.indices[i]]
		elif isinstance(i, slice):	return typedlist((self.points[j] for j in self.indices[i]), dtype=vec3)
		else:						raise TypeError('item index must be int or slice')
	
	def __add__(self, other):
		''' append the indices and points of the other wire '''
		if isinstance(other, Wire):
			r = Wire(
				self.points if self.points is other.points else self.points[:], 
				self.indices[:], 
				self.tracks[:] if self.tracks else None, 
				self.groups if self.groups is other.groups else self.groups[:],
				)
			r.__iadd__(other)
			return r
		else:
			return NotImplemented
			
	def __iadd__(self, other):
		''' append the indices and points of the other wire '''
		if isinstance(other, Wire):		
			li = len(self.indices)
			
			if self.points is other.points:
				self.indices.extend(other.indices)
			else:
				lp = len(self.points)
				self.points.extend(other.points)
				self.indices.extend(i+lp  for i in other.indices)
			
			if self.groups is other.groups:
				if self.tracks or other.tracks:
					if not self.tracks:
						self.tracks = typedlist.full(0, li, 'I')
					self.tracks.extend(other.tracks or typedlist.full(0, len(other.indices), 'I'))
			else:
				lg = len(self.groups)
				self.groups.extend(other.groups)
				if not self.tracks:	
					self.tracks = typedlist.full(0, li, 'I')
				if other.tracks:
					self.tracks.extend(track+lg	for track in other.tracks)
				else:
					self.tracks.extend(typedlist.full(lg, len(other.indices), 'I'))
			return self
		else:
			return NotImplemented
			
	# END BEGIN ----- data management -----
	
	def strippoints(self):
		''' remove points that are used by no edge
			if used is provided, these points will be removed without usage verification
			
			no reindex table is returned as its generation costs more than the stripping operation
		'''
		self.points = typedlist((self.points[i]	for i in self.indices), dtype=vec3)
		self.indices = typedlist(range(len(self.points)), dtype='I')
		if self.points[-1] == self.points[0]:	
			self.points.pop()
			self.indices[-1] = 0
	
	def mergepoints(self, merges) -> 'self':
		''' merge points with the merge dictionnary {src index: dst index}
			merged points are not removed from the buffer.
		'''
		j = 0
		for i,f in enumerate(self.edges):
			f = merges.get(f, f)
			if not f == self.edges[i-1]:
				self.indices[i] = f
				if self.tracks:
					self.tracks[j] = self.tracks[i]
				j += 1
		del self.indices[j:]
		return self
	
	def mergeclose(self, limit=None):
		''' merge close points ONLY WHEN they are already linked by an edge.
			the meaning of this method is different than `Web.mergeclose()`
		'''
		if limit is None:	limit = self.precision()
		limit *= limit
		merges = {}
		for i in reversed(range(1, len(self.indices))):
			if distance2(self[i-1], self[i]) <= limit:
				merges[self.indices[i]] = self.indices[i-1]
				self.indices.pop(i)
				if self.tracks:
					self.tracks.pop(i-1)
		if distance2(self[0], self[-1]) < limit:
			merges[self.indices[-1]] = self.indices[0]
			self.indices[-1] = self.indices[0]
			if self.tracks:
				self.tracks[-1] = self.tracks[0]
		return merges
		
	# END BEGIN ----- mesh checks -----
	
	def check(self):
		''' raise if the internal data are not consistent '''
		if not (isinstance(self.points, typedlist) and self.points.dtype == vec3):	raise MeshError("points must be a typedlist(dtype=vec3)")
		if not (isinstance(self.indices, typedlist) and self.indices.dtype == 'I'): 	raise MeshError("indices must be a typedlist(dtype='I')")
		if self.tracks and not (isinstance(self.tracks, typedlist) and self.tracks.dtype == 'I'): 	raise MeshError("tracks must be a typedlist(dtype='I')")
		l = len(self.points)
		for i in self.indices:
			if i >= l:	raise MeshError("some indices are greater than the number of points", i, l)
		if self.tracks:
			if len(self.indices) != len(self.tracks):	raise MeshError("tracks list doesn't match indices list length")
			if max(self.tracks) >= len(self.groups):	raise MeshError("some tracks are greater than the number of groups", max(self.tracks), len(self.groups))

	
	# END BEGIN ----- selection methods -----
	
	def groupnear(self, point: vec3) -> int:
		''' return group id if the edge the closest to the given point '''
		return self.tracks[self.edgenear(point)]
	
	def edgenear(self, point: vec3) -> int:
		''' return the index of the closest edge to the given point '''
		return min( range(len(self.indices)-1),
					lambda i: distance_pe(point, self.edgepoints(i)) )
	
	def group(self, groups):
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
		if not self.tracks:
			return self
		if isinstance(groups, set):			pass
		elif hasattr(groups, '__iter__'):	groups = set(groups)
		else:								groups = (groups,)
		indices = typedlist(dtype=self.indices.dtype)
		tracks = typedlist(dtype=self.tracks.dtype)
		for i,t in zip(self.indices, self.tracks):
			if t in groups:
				indices.append(i)
				tracks.append(t)
		return Wire(self.points, indices, tracks, self.groups, self.options)

	# END BEGIN ----- extraction methods -----
		
	def edgepoints(self, e) -> tuple:
		''' shorthand to the tuple of points forming edge e '''
		if isinstance(f, Integral):
			e = self.edge(e)
		return self.points[e[0]], self.points[e[1]]
		
	def edgedirection(self, e) -> vec3:
		''' direction of edge e '''
		if isinstance(e, Integral):	
			e = self.edge(e)
		return normalize(self.points[e[1]] - self.points[e[0]])
	
		
	def edge(self, i) -> uvec2:
		''' ith edge of the wire '''
		return uvec2(self.indices[i], self.indices[i+1])
	
	def edges(self) -> typedlist:
		''' list of successive edges of the wire '''
		return typedlist((self.edge(i)  for i in range(len(self.indices)-1)), dtype=uvec2)
	
	def length(self) -> float:
		''' curviform length of the wire (sum of all edges length) '''
		s = 0
		for i in range(1,len(self.indices)):
			s += distance(self[i-1], self[i])
		return s
		
	def surface(self) -> float:
		''' return the surface enclosed by the web if planar and is a loop (else it has no meaning) '''
		s = vec3(0)
		o = self.barycenter()
		for i in range(1, len(self)):
			area += cross(self[i-1]-o, self[i]-o)
		return length(area)
		
	def barycenter(self) -> vec3:
		''' curve barycenter '''
		if not self.indices:	return vec3(0)
		if len(self.indices) == 1:	return self.points[self.indices[0]]
		acc = vec3(0)
		tot = 0
		for i in range(1,len(self)):
			a,b = self[i-1], self[i]
			weight = distance(a,b)
			tot += weight
			acc += weight*(a+b)
		return acc / (2*tot)
			
	def normal(self) -> vec3:
		''' return an approximated normal to the curve as if it was the outline of a flat surface.
			if this is not a loop the result is undefined.
		'''
		area = vec3(0)
		c = self.barycenter()
		for i in range(1, len(self)):
			area += cross(self[i-1]-c, self[i]-c)
		return normalize(area)
	
	def vertexnormals(self, loop=False):
		''' return the opposed direction to the curvature in each point 
			this is called normal because it would be the normal to a surface whose section would be that wire
		'''
		normals = typedlist.full(vec3(0), len(self.indices))
		for i in range(len(self.indices)):
			a,b,c = self.indices[i-2], self.indices[i-1], self.indices[i]
			normals[i-1] = normalize(normalize(self.points[b]-self.points[a]) + normalize(self.points[b]-self.points[c]))
		self._make_loop_consistency(normals, loop)
		return normals
		
	def tangents(self, loop=False):
		''' return approximated tangents to the curve as if it was a surface section.
			if this is not a loop the result is undefined.
		'''
		tangents = typedlist.full(vec3(0), len(self.indices))
		for i in range(len(self.indices)):
			a,b,c = self.indices[i-2], self.indices[i-1], self.indices[i]
			tangents[i-1] = normalize(cross(self.points[b]-self.points[a], self.points[b]-self.points[c]))
		self._make_loop_consistency(tangents, loop)
		return tangents
	
	def _make_loop_consistency(self, normals, loop):
		l = len(self.indices)
		# make normals consistent if asked
		if loop:
			# get an outermost point as it is always well oriented
			dir = self[l//2] - self[0]	# WARNING: if those two points are too close the computation is misleaded
			i = max(self.indices, key=lambda i: dot(self.points[i], dir))
			# propagation reorient
			# WARNING: if there is a cusp in the curve (2 consecutive segments with opposite directions) the final result can be wrong
			for i in range(i+1, i+l):
				j = i%l
				e = normalize(self[j]-self[j-1])
				if dot(noproject(normals[j],e), noproject(normals[j-1],e)) < 0:
					normals[j] = -normals[j]
		# propagate to borders if not loop
		else:
			normals[0] = normals[1]
			normals[-1] = normals[-2]
		# propagate to erase undefined normals
		for _ in range(2):
			for i in range(l):
				if glm.any(isnan(normals[i])):	
					normals[i] = normals[i-1]
					
	
	def flip(self) -> 'Wire':
		''' reverse direction of all edges '''
		indices = deepcopy(self.indices)
		indices.reverse()
		if self.tracks:
			tracks = deepcopy(self.tracks[:-1])
			tracks.reverse()
			tracks.append(self.tracks[-1])
		else:
			tracks = None
		return Wire(self.points, indices, tracks, self.groups, self.options)
		
	def close(self) -> 'self':
		''' make a loop of the wire by appending its first point to its end '''
		if self.indices[-1] != self.indices[0]:
			self.indices.append(self.indices[0])
			if self.tracks:
				self.tracks.append(self.tracks[0])
		return self
		
	def segmented(self, group=None) -> 'Wire':
		''' return a copy of the mesh with a group each edge 
		
			if group is specified, it will be the new definition put in each groups
		'''
		return Wire(self.points, 
					self.indices,
					typedlist(range(len(self.indices)), dtype='I'),
					[group]*len(self.indices),
					self.options,
					)
	
	# END BEGIN ----- ouput methods -----
	
	def display(self, scene):
		from .. import displays
		from .conversions import web
		
		w = web(self)
		w.options = self.options
		return w.display(scene)
	
	def __repr__(self):
		return '<Wire with {} points at 0x{:x}, {} indices>'.format(len(self.points), id(self.points), len(self.indices))
	
	def __str__(self):
		return 'Wire(\n  points={},\n  indices={},\n  tracks={},\n  groups={})'.format(
					reprarray(self.points, 'points'),
					reprarray(self.indices, 'indices'),
					reprarray(self.tracks, 'tracks') if self.tracks else None,
					repr(self.groups))
					
	# END




