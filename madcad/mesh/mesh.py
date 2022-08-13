from .container import *
from .web import Web
from .wire import Wire


class Mesh(NMesh):
	''' set of triangles, used to represent volumes or surfaces.
		As volumes are represented by their exterior surface, there is no difference between representation of volumes and faces, juste the way we interpret it.
		
		Attributes:
			points:     typedlist of vec3 for points
			faces:		typedlist of uvec3 for faces, the triplet is (a,b,c) such that  cross(b-a, c-a) is the normal oriented to the exterior.
			tracks:	    typedlist of integers giving the group each face belong to
			groups:     custom information for each group
			options:	custom informations for the entire mesh
	'''
	__slots__ = 'points', 'faces', 'tracks', 'groups', 'options'
	
	# BEGIN --- special methods ---
	
	def __init__(self, points=None, faces=None, tracks=None, groups=None, options=None):
		self.points = ensure_typedlist(points, vec3)
		self.faces = ensure_typedlist(faces, uvec3)
		self.tracks = ensure_typedlist(tracks if tracks is not None else typedlist.full(0, len(self.faces), 'I'), 'I')
		self.groups = groups if groups is not None else [None] * (max(self.tracks, default=-1)+1)
		self.options = options or {}
	
	def __add__(self, other):
		''' return a new mesh concatenating the faces and points of both meshes '''
		if isinstance(other, Mesh):
			r = Mesh(
				self.points if self.points is other.points else self.points[:], 
				self.faces[:], 
				self.tracks[:], 
				self.groups if self.groups is other.groups else self.groups[:],
				)
			r.__iadd__(other)
			return r
		else:
			return NotImplemented
			
	def __iadd__(self, other):
		''' append the faces and points of the other mesh '''
		if isinstance(other, Mesh):		
			if self.points is other.points:
				self.faces.extend(other.faces)
			else:
				lp = len(self.points)
				self.points.extend(other.points)
				self.faces.extend(f+lp  for f in other.faces)
			if self.groups is other.groups:
				self.tracks.extend(other.tracks)
			else:
				lt = len(self.groups)
				self.groups.extend(other.groups)
				self.tracks.extend(track+lt  for track in other.tracks)
			return self
		else:
			return NotImplemented
		
	# END BEGIN --- data management ---
	
	def strippoints(self) -> list:
		''' remove points that are used by no faces, return the reindex list.
			if used is provided, these points will be removed without usage verification
			
			return a table of the reindex made
		'''
		self.points, self.faces, reindex = striplist(self.points, self.faces)
		return reindex
	
	def mergepoints(self, merges) -> 'self':
		''' merge points with the merge dictionnary {src index: dst index}
			merged points are not removed from the buffer.
		'''
		j = 0
		for i,f in enumerate(self.faces):
			f = uvec3(
				merges.get(f[0], f[0]),
				merges.get(f[1], f[1]),
				merges.get(f[2], f[2]),
				)
			if not (f[0] == f[1] or f[1] == f[2] or f[2] == f[0]):
				self.faces[j] = f
				self.tracks[j] = self.tracks[i]
				j += 1
		del self.faces[j:]
		del self.tracks[j:]
		return self
					
					
	# END BEGIN ---- mesh checks -----
		
	def issurface(self):
		''' return True if the mesh is a well defined surface (an edge has 2 connected triangles at maximum, with consistent normals)
			such meshes are usually called 'manifold'
		''' 
		reached = set()
		for face in self.faces:
			for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
				if e in reached:	return False
				else:				reached.add(e)
		return True
	
	def isenvelope(self):
		''' return True if the surfaces are a closed envelope (the outline is empty)
		'''
		return len(self.outlines_oriented()) == 0
	
	def check(self):
		''' raise if the internal data is inconsistent '''
		if not (isinstance(self.points, typedlist) and self.points.dtype == vec3):	raise MeshError("points must be a typedlist(dtype=vec3)")
		if not (isinstance(self.faces, typedlist) and self.faces.dtype == uvec3): 	raise MeshError("faces must be a typedlist(dtype=uvec3)")
		if not (isinstance(self.tracks, typedlist) and self.tracks.dtype == 'I'): 	raise MeshError("tracks must be a typedlist(dtype='I')")
		l = len(self.points)
		for face in self.faces:
			for p in face:
				if p >= l:	raise MeshError("some point indices are greater than the number of points", face, l)
			if face[0] == face[1] or face[1] == face[2] or face[2] == face[0]:	raise MeshError("some faces use the same point multiple times", face)
		if len(self.faces) != len(self.tracks):	raise MeshError("tracks list doesn't match faces list length")
		if max(self.tracks, default=-1) >= len(self.groups): raise MeshError("some face group indices are greater than the number of groups", max(self.tracks, default=-1), len(self.groups))
	

	# END BEGIN --- selection methods ---
	
	def groupnear(self, point) -> int:
		''' return group id if the face the closest to the given point '''
		return self.tracks[self.face_near(point)]
		
	def facenear(self, point) -> int:
		''' return the index of the closest triangle to the given point '''
		return min(	range(len(self.faces)), 
					lambda i: distance_pt(point, self.facepoints(i)) )
	
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
		faces = typedlist(dtype=uvec3)
		tracks = typedlist(dtype='I')
		for i in self.qualified_indices(quals):
			faces.append(self.faces[i])
			tracks.append(self.tracks[i])
		return Mesh(self.points, faces, tracks, self.groups)
		
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
				self.faces[j] = self.faces[i]
				self.tracks[j] = t
				j += 1
		del self.faces[j:]
		del self.tracks[j:]
		self += mesh
		return self
	
	
	# END BEGIN --- extraction methods ---
		
	def facepoints(self, f) -> tuple:
		''' shorthand to get the points of a face (index is an int or a triplet) '''
		if isinstance(f, Integral):
			f = self.faces[f]
		return self.points[f[0]], self.points[f[1]], self.points[f[2]]
	
	def facenormal(self, f) -> vec3:
		''' normal for a face '''
		if isinstance(f, Integral):	
			f = self.faces[f]
		p0 = self.points[f[0]]
		e1 = self.points[f[1]] - p0
		e2 = self.points[f[2]] - p0
		return normalize(cross(e1, e2))
	
	def facenormals(self) -> '[vec3]':
		''' list normals for each face '''
		return typedlist(map(self.facenormal, self.faces), vec3)
	
	def edgenormals(self) -> '{uvec2: vec3}':
		''' dict of normals for each UNORIENTED edge '''
		normals = {}
		for face in self.faces:
			normal = self.facenormal(face)
			for edge in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
				e = edgekey(*edge)
				normals[e] = normals.get(e,0) + normal
		for e,normal in normals.items():
			normals[e] = normalize(normal)
		return normals
	
		
	def vertexnormals(self) -> '[vec3]':
		''' list of normals for each point '''
		
		# collect the mesh border as edges and as points
		outline = self.outlines_oriented()
		border = set()
		for a,b in outline:
			border.add(a)
			border.add(b)
		
		# sum contributions to normals
		l = len(self.points)
		normals = typedlist.full(vec3(0), l)
		for face in self.faces:
			normal = self.facenormal(face)
			if not isfinite(normal):	continue
			for i in range(3):
				o = self.points[face[i]]
				# point on the surface
				if face[i] not in border:
					# triangle normals are weighted by their angle at the point
					contrib = anglebt(self.points[face[i-2]]-o, self.points[face[i-1]]-o)
					normals[face[i]] += contrib * normal
				# point on the outline
				elif (face[i], face[i-1]) in outline:
					# only the triangle creating the edge does determine its normal
					normals[face[i]] += normal
					normals[face[i-1]] += normal
		
		for i in range(l):
			normals[i] = normalize(normals[i])
		assert len(normals) == len(self.points)
		return normals
		
	def tangents(self) -> '{int: vec3}':
		''' tangents to outline points '''
		# outline with associated face normals
		edges = {}
		for face in self.faces:
			for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
				if e in edges:	del edges[e]
				else:			edges[(e[1], e[0])] = self.facenormal(face)
		
		# cross neighbooring normals
		tangents = {}
		for loop in suites(edges, cut=False):
			assert loop[-1] == loop[0],  "non-manifold mesh"
			loop.pop()
			for i in range(len(loop)):
				c = cross(	edges[(loop[i-2],loop[i-1])], 
							edges[(loop[i-1],loop[i])] )
				o = cross(  self.points[loop[i-2]] - self.points[loop[i]],
				            edges[(loop[i-2],loop[i-1])] + edges[(loop[i-1],loop[i])] )
				tangents[loop[i-1]] = normalize(mix(o, c, clamp(length2(c)/length2(o)/NUMPREC, 0, 1) ))
		return tangents
	
	def edges(self) -> set:
		''' set of UNORIENTED edges present in the mesh '''
		edges = set()
		for face in self.faces:
			edges.add(edgekey(face[0], face[1]))
			edges.add(edgekey(face[1], face[2]))
			edges.add(edgekey(face[2], face[0]))
		return edges
	
	def edges_oriented(self) -> set:
		''' iterator of ORIENTED edges, directly retreived of each face '''
		for face in self.faces:
			yield face[0], face[1]
			yield face[1], face[2]
			yield face[2], face[0]
	
	def outlines_oriented(self) -> set:
		''' return a set of the ORIENTED edges delimiting the surfaces of the mesh '''
		edges = set()
		for face in self.faces:
			for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
				if e in edges:	edges.remove(e)
				else:			edges.add((e[1], e[0]))
		return edges
	
	def outlines_unoriented(self) -> set:
		''' return a set of the UNORIENTED edges delimiting the surfaces of the mesh 
			this method is robust to face orientation aberations
		'''
		edges = set()
		for face in self.faces:
			for edge in ((face[0],face[1]),(face[1],face[2]),(face[2],face[0])):
				e = edgekey(*edge)
				if e in edges:	edges.remove(e)
				else:			edges.add(e)
		return edges
	
	def outlines(self) -> 'Web':
		''' return a Web of ORIENTED edges '''
		return Web(self.points, self.outlines_oriented())
		
	def groupoutlines(self) -> 'Web':
		''' return a Web of ORIENTED edges indexing groups.
			
			On a frontier between multiple groups, there is as many edges as groups, each associated to a group.
		'''
		edges = typedlist(dtype=uvec2)		# outline
		tracks = typedlist(dtype='I')		# groups for edges

		tmp = {}	# faces adjacent to edges
		for i,face in enumerate(self.faces):
			for e in ((face[1],face[0]),(face[2],face[1]),(face[0],face[2])):
				track = self.tracks[i]
				if e in tmp:
					if tmp[e] != track:
						edges.append(e)
						tracks.append(track)
					del tmp[e]
				else:
					tmp[(e[1],e[0])] = track
		edges.extend(tmp.keys())
		tracks.extend(tmp.values())
		return Web(self.points, edges, tracks, self.groups)
		
	def frontiers(self, *args) -> 'Web':
		''' return a Web of UNORIENTED edges that split the given groups appart.
		
			The arguments are groups indices or lists of group qualifiers (as set in `qualify()`). If there is one only argument it is considered as as list of arguments.
		
			- if no argument is given, then return the frontiers between every groups
			- to include the groups edges that are on the group border but not at the frontier with an other group, add `None` to the group set
			
			Example:
			
				>>> m = Mesh([...], [uvec3(0,1,2), uvec3(2,1,3)], [0, 1], [...])
				>>> m.frontiers(0,1).edges
				[uvec2(1,2)]
				
				>>> # equivalent to
				>>> m.frontiers({0,1}).edges
				[uvec2(1,2)]
				
				>>> m.frontiers(0,None)
				[uvec2(0,1), uvec2(0,2)]
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
		
		edges = typedlist(dtype=uvec2)
		tracks = typedlist(dtype='I')
		couples = OrderedDict()
		belong = {}
		for i,face in enumerate(self.faces):
			if groups is None or self.tracks[i] in groups:	
				for edge in ((face[0],face[1]),(face[1],face[2]),(face[2],face[0])):
					e = edgekey(*edge)
					if e in belong:
						if belong[e] != self.tracks[i]:
							g = edgekey(belong[e],self.tracks[i])
							edges.append(e)
							tracks.append(couples.setdefault(g, len(couples)))
						del belong[e]
					else:
						belong[e] = self.tracks[i]
		if groups and None in groups:
			edges.extend(belong.keys())
		return Web(self.points, edges, tracks, list(couples))
	
	def surface(self) -> float:
		''' total surface of triangles '''
		s = 0
		for f in self.faces:
			a,b,c = self.facepoints(f)
			s += length(cross(a-b, a,c))/2
		return s
		
	def volume(self) -> float:
		''' return the volume enclosed by the mesh if composed of envelopes (else it has no meaning) '''
		o = self.barycenter()
		s = vec3(0)
		for f in self.faces:
			s += glm.determinant(mat3(self.points[f[0]] - o, self.points[f[1]] - o, self.points[f[2]] - o))
		return s
	
	def barycenter(self) -> vec3:
		''' surface barycenter of the mesh '''
		if not self.faces:	return vec3(0)
		acc = vec3(0)
		tot = 0
		for f in self.faces:
			a,b,c = self.facepoints(f)
			weight = length(cross(b-a, c-a))
			tot += weight
			acc += weight*(a+b+c)
		return acc / (3*tot)
			
	def propagate(self, atface, atisland=None, find=None, conn=None):
		''' return the unconnected parts of the mesh as several meshes '''
		if not conn:	
			conn = connef(self.faces)
		
		reached = [False] * len(self.faces)	# faces reached
		stack = []
		# procedure for finding the new islands to propagate on
		if not find:
			start = [0]
			def find(stack, reached):
				for i in range(start[0],len(reached)):
					if not reached[i]:
						stack.append(i)
						break
				start[0] = i
		# propagation
		while True:
			# search start point
			find(stack, reached)
			# end when everything reached
			if not stack:	break
			# propagate
			while stack:
				i = stack.pop()
				if reached[i]:	continue	# make sure this face has not been stacked twice
				reached[i] = True
				atface(i, reached)
				f = self.faces[i]
				for i in range(3):
					e = f[i],f[i-1]
					if e in conn and not reached[conn[e]]:
						stack.append(conn[e])
			if atisland:
				atisland(reached)
	
	
	def splitgroups(self, edges=None):
		''' split the mesh groups into connectivity separated groups.
			the points shared by multiple groups will be duplicated
			if edges is provided, only the given edges at group frontier will be splitted
			
			return a list of tracks for points
		'''
		if edges is None:	edges = self.frontiers().edges
		# mark points on the frontier
		frontier = [False]*len(self.points)
		for a,b in edges:
			frontier[a] = True
			frontier[b] = True
		# duplicate points and reindex faces
		points = copy(self.points)
		idents = typedlist.full(0, len(self.points), 'I')		# track id corresponding to each point
		duplicated = {}		# new point index for couples (frontierpoint, group)
		def repl(pt, track):
			if frontier[pt]:
				key = (pt,track)
				if key in duplicated:	i = duplicated[key]
				else:
					i = duplicated[key] = len(points)
					points.append(points[pt])
					idents.append(track)
				return i
			else:
				idents[pt] = track
				return pt
		faces = typedlist((uvec3(repl(a,t), repl(b,t), repl(c,t))  
						for (a,b,c),t in zip(self.faces, self.tracks)), dtype=uvec3)
		
		self.points = points
		self.faces = faces
		return idents
		
	def split(self, edges) -> 'Self':
		''' split the mesh around the given edges. 
			The points in common with two or more designated edges will be dupliated once or more, and the face indices will be reassigned so that faces each side of the given edges will own a duplicate of that point each.
		'''
		# get connectivity and set of edges to manage
		conn = connef(self.faces)
		edges = set(edgekey(*edge)   for edge in edges)
		# collect the points to multiply
		ranks = Counter(p  for e in edges for p in e)
		separations = set(p  for p, count in ranks.items() if count > 1)
		newfaces = deepcopy(self.faces)
		# for each edge, reassign neighboring faces to the proper points
		for edge in edges:
			for pivot in edge:
				if edge in conn and pivot in newfaces[conn[edge]]:
					dupli = len(self.points)
					self.points.append(self.points[pivot])
					
					# change the point index in every neighbooring face
					front = edge
					while front in conn:
						fi = conn[front]
						f = arrangeface(self.faces[fi], pivot)
						fm = arrangeface(newfaces[fi], pivot)
						
						assert f[0] == pivot
						if fm[0] != pivot: 
							break
						
						newfaces[fi] = uvec3(dupli, fm[1], fm[2])
						if pivot == front[0]:	front = (pivot, f[2])
						elif pivot == front[1]:	front = (f[1], pivot)
						else:
							raise AssertionError('error in connectivity')
						if edgekey(*front) in edges:
							break
		
		self.faces = newfaces
		return self
	
	def islands(self, conn=None) -> '[Mesh]':
		''' return the unconnected parts of the mesh as several meshes '''
		islands = []
		faces = typedlist(dtype=uvec3)
		tracks = typedlist(dtype='I')
		def atface(i, reached):
			faces.append(self.faces[i])
			tracks.append(self.tracks[i])
		def atisland(reached):
			islands.append(Mesh(self.points, deepcopy(faces), deepcopy(tracks), self.groups))
			faces.clear()
			tracks.clear()
		self.propagate(atface, atisland, conn=conn)
		return islands
	
	def flip(self) -> 'Self':
		''' flip all faces, getting the normals opposite '''
		return Mesh(self.points, 
					typedlist((uvec3(f[0],f[2],f[1]) for f in self.faces), dtype=uvec3), 
					self.tracks, 
					self.groups)
	
	def orient(self, dir=None, conn=None) -> 'Self':
		''' flip the necessary faces to make the normals consistent, ensuring the continuity of the out side.
			
			Argument `dir` tries to make the result deterministic:
			
				* if given, the outermost point in this direction will be considered pointing outside
				* if not given, the farthest point to the barycenter will be considered pointing outside
				
				note that if the mesh contains multiple islands, that direction must make sense for each single island
		'''
		if dir:	
			metric = lambda p, n: (dot(p, dir), abs(dot(n, dir)))
			orient = lambda p, n: dot(n, dir)
		else:	
			center = self.barycenter()
			metric = lambda p, n: (length2(p-center), abs(dot(n, p-center)))
			orient = lambda p, n: dot(n, p-center)
		if not conn:	
			conn = Asso(  (edgekey(*e),i)
							for i,f in enumerate(self.faces)
							for e in ((f[0],f[1]), (f[1],f[2]), (f[2],f[0]))
							)
		
		faces = self.faces
		normals = self.facenormals()
		
		reached = [False] * len(faces)	# faces reached
		stack = []
		
		# propagation
		while True:
			# search start point
			best = (-inf,0)
			candidate = None
			for i,f in enumerate(faces):
				if not reached[i]:
					for p in f:
						score = metric(self.points[p], normals[i])
						if score > best:
							best, candidate = score, i
							if orient(self.points[p], normals[i]) < 0:
								faces[i] = (f[2],f[1],f[0])
			# end when everything reached
			if candidate is None:
				break
			else:
				stack.append(candidate)
			# process neighbooring
			while stack:
				i = stack.pop()
				if reached[i]:	continue	# make sure this face has not been stacked twice
				reached[i] = True
				
				f = faces[i]
				for i in range(3):
					e = f[i], f[i-1]
					for n in conn[edgekey(*e)]:
						if reached[n]:	continue
						nf = faces[n]
						# check for orientation continuity
						if arrangeface(nf,f[i-1])[1] == f[i]:
							faces[n] = (nf[2],nf[1],nf[0])
						# propagate
						stack.append(n)
		
		return self
	
	# END BEGIN ----- output methods ------
	
	def display(self, scene):
		from .. import displays
		
		m = copy(self)
		
		m.split(m.frontiers().edges)
		edges = m.outlines().edges
		
		# select edges above a threshold
		tosplit = []
		thresh = cos(settings.display['sharp_angle'])
		conn = connef(m.faces)
		for edge, f1 in conn.items():
			if edge[1] > edge[0]:	continue
			f2 = conn.get((edge[1],edge[0]))
			if f2 is None:	continue
			if m.tracks[f1] != m.tracks[f2] or dot(m.facenormal(f1), m.facenormal(f2)) <= thresh:
				tosplit.append(edge)
		
		m.split(tosplit)
		
		# get the group each point belong to
		idents = [0] * len(m.points)
		for face, track in zip(m.faces, m.tracks):
			for p in face:
				idents[p] = track
		
		normals = m.vertexnormals()
		
		if not m.points or not m.faces:	
			return displays.Display()
		
		return displays.SolidDisplay(scene, 
				typedlist_to_numpy(m.points, 'f4'), 
				typedlist_to_numpy(normals, 'f4'), 
				typedlist_to_numpy(m.faces, 'u4'),
				typedlist_to_numpy(edges, 'u4'),
				typedlist_to_numpy(idents, 'u4'),
				color = self.options.get('color'),
				)
	
	def __repr__(self):
		return '<Mesh with {} points at 0x{:x}, {} faces>'.format(len(self.points), id(self.points), len(self.faces))
	
	def __str__(self):
		return 'Mesh(\n  points={},\n  faces={},\n  tracks={},\n  groups={},\n  options={})'.format(
					reprarray(self.points, 'points'),
					reprarray(self.faces, 'faces'),
					reprarray(self.tracks, 'tracks'),
					reprarray(self.groups, 'groups'),
					repr(self.options))
		
	# END






def mktri(mesh, pts, track=0):
	''' append a triangle '''
	mesh.faces.append(pts)
	mesh.tracks.append(track)

def mkquad(mesh, pts, track=0):
	''' append a quad, choosing the best diagonal '''
	if (	distance2(mesh.points[pts[0]], mesh.points[pts[2]]) 
		<=	distance2(mesh.points[pts[1]], mesh.points[pts[3]]) ):
		mesh.faces.append((pts[:-1]))
		mesh.faces.append((pts[3], pts[0], pts[2]))
	else:
		mesh.faces.append((pts[0], pts[1], pts[3]))
		mesh.faces.append((pts[2], pts[3], pts[1]))
	mesh.tracks.append(track)
	mesh.tracks.append(track)
