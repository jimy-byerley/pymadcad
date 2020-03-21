'''	
	Defines triangular meshes for pymadcad
'''

from copy import deepcopy
from random import random
import numpy as np
from array import array
from mathutils import Box, vec3, vec4, mat3, mat4, quat, mat3_cast, cross, dot, normalize, length, distance, anglebt, NUMPREC
import math
import view
import text

__all__ = ['Mesh', 'Web', 'Wire', 'MeshError', 'web', 'edgekey', 'lineedges', 'striplist']

class MeshError(Exception):	pass

class Container:
	''' common methods for points container (typically Mesh or Wire) '''
	
	# --- basic transformations of points ---
	
	def transform(self, trans):
		''' apply the transform to the points of the mesh'''
		if isinstance(trans, quat):		trans = mat3_cast(trans)
		if isinstance(trans, vec3):		transformer = lambda v: v + trans
		elif isinstance(trans, mat3):	transformer = lambda v: trans * v
		elif isinstance(trans, mat4):	transformer = lambda v: vec3(trans * vec4(v,1))
		elif callable(trans):	pass
		for i in range(len(self.points)):
			self.points[i] = transformer(self.points[i])
			
	def mergeclose(self, limit=NUMPREC):
		''' merge points below the specified distance, or below the precision '''
		merges = {}
		for j in reversed(range(len(self.points))):
			for i in range(j):
				if distance(self.points[i], self.points[j]) < limit and i not in merges:
					merges[j] = i
					break
		self.mergepoints(merges)
		return merges
		
	def stripgroups(self):
		''' remove groups that are used by no faces, return the reindex list '''
		used = [False] * len(self.groups)
		for track in self.tracks:
			used[track] = True
		reindex = striplist(self.groups, used)
		for i,track in enumerate(self.tracks):
			self.tracks[i] = reindex[track]
		return reindex
	
	def finish(self):
		''' finish and clean the mesh 
			note that this operation can cost as much as other transformation operation
			job done
				- mergeclose
				- strippoints
				- stripgroups
		'''
		self.mergeclose()
		self.strippoints()
		self.stripgroups()
		assert self.isvalid()
	
	# --- verification methods ---
		
	def isvalid(self):
		try:				self.check()
		except MeshError:	return False
		else:				return True
	
	# --- selection methods ---
		
	def usepointat(self, point, neigh=NUMPREC):
		''' return the index of the first point in the mesh at the location, if none is found, insert it and return the index '''
		i = self.pointat(point, neigh=neigh)
		if i is None:
			i = len(self.points)
			self.points.append(point)
		return i
	
	def pointat(self, point, neigh=NUMPREC):
		''' return the index of the first point at the given location, or None '''
		for i,p in enumerate(self.points):
			if distance(p,point) <= neigh:	return i
	
	def pointnear(self, point):
		''' return the nearest point the the given location '''
		return min(	range(len(self.points)), 
					lambda i: distance(self.points[i], point))
					
	def box(self):
		''' return the extreme coordinates of the mesh (vec3, vec3) '''
		if not self.points:		return None
		max = deepcopy(self.points[0])
		min = deepcopy(self.points[0])
		for pt in self.points:
			for i in range(3):
				if   pt[i] < min[i]:	min[i] = pt[i]
				elif pt[i] > max[i]:	max[i] = pt[i]
		return Box(min, max)
	


class Mesh(Container):
	''' set of triangles, used to represent volumes or surfaces.
		As volumes are represented by their exterior surface, there is no difference between representation of volumes and faces, juste the way we interpret it.
		
		Attributes:
			points		list of vec3 for points
			faces		list of triplets for faces, the triplet is (a,b,c) such that  cross(b-a, c-a) is the normal oriented to the exterior.
			tracks		integer giving the group each face belong to
			groups		custom information for each group
			options		custom informations for the entire mesh
	'''
	
	# --- standard point container methods ---
	
	def __init__(self, points=None, faces=None, tracks=None, groups=None):
		self.points = points or []
		self.faces = faces or []
		self.tracks = tracks or [0] * len(self.faces)
		self.groups = groups or [None] * (max(self.tracks, default=0)+1)
		self.options = {}
	
	
	def __add__(self, other):
		''' append the faces and points of the other mesh '''
		if isinstance(other, Mesh):
			r = deepcopy(self)
			r.__iadd__(other)
			return r
		else:
			return NotImplemented
			
	def __iadd__(self, other):
		''' append the faces and points of the other mesh '''
		if isinstance(other, Mesh):		
			lp = len(self.points)
			lt = len(self.groups)
			self.points.extend(other.points)
			self.groups.extend(other.groups)
			for face,track in zip(other.faces, other.tracks):
				self.faces.append((face[0]+lp, face[1]+lp, face[2]+lp))
				self.tracks.append(track+lt)
			return self
		else:
			return NotImplemented
	
	def reverse(self):
		''' reverse direction of all faces '''
		self.faces = [(a,c,b)   for a,b,c in self.faces]
		
	# --- mesh optimization ---
		
	def mergepoints(self, merges):
		''' merge points with the merge dictionnary {src index: dst index}
			remaining points are not removed
		'''
		i = 0
		while i < len(self.faces):
			f = self.faces[i]
			self.faces[i] = f = (
				merges.get(f[0], f[0]),
				merges.get(f[1], f[1]),
				merges.get(f[2], f[2]),
				)
			if f[0] == f[1] or f[1] == f[2] or f[2] == f[0]:
				self.faces.pop(i)
				self.tracks.pop(i)
			else:
				i += 1
	
	def strippoints(self):
		''' remove points that are used by no faces, return the reindex list '''
		used = [False] * len(self.points)
		for face in self.faces:
			for p in face:
				used[p] = True
		reindex = striplist(self.points, used)
		for i,f in enumerate(self.faces):
			self.faces[i] = (reindex[f[0]], reindex[f[1]], reindex[f[2]])
		return reindex
		
	def issurface(self):
		reached = set()
		for face in self.faces:
			for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
				if e in reached:	return False
				else:				reached.add(e)
		return True
	def isenvelope(self):
		''' return true if the surfaces are a closed envelope '''
		return len(self.outlines_oriented()) == 0
	
	def check(self):
		''' check that the internal data references are good (indices and list lengths) '''
		l = len(self.points)
		for face in self.faces:
			for p in face:
				if p >= l:	raise MeshError("some point indices are greater than the number of points", face, l)
			if face[0] == face[1] or face[1] == face[2] or face[2] == face[0]:	raise MeshError("some faces use the same point multiple times", face)
		if len(self.faces) != len(self.tracks):	raise MeshError("tracks list doesn't match faces list length")
		if max(self.tracks, default=-1) >= len(self.groups): raise MeshError("some face group indices are greater than the number of groups", max(self.tracks), len(self.groups))
	
	def finish(self):
		''' finish and clean the mesh 
			note that this operation can cost as much as other transformation operation
			job done
				- mergeclose
				- strippoints
				- stripgroups
		'''
		self.mergeclose()
		self.strippoints()
		self.stripgroups()
		assert self.isvalid()
	
	
	# --- selection methods ---
		
	def outptnear(self, point):
		''' return the closest point to the given point, that belongs to a group outline '''
		outpts = set()
		for edge in self.outlines_oriented():	# TODO changer ou supprimer: outlines_oriented ne donne que les contours independament des groups
			outpts += edge
		return min(	range(len(self.points)), 
					lambda i: distance(self.points[i], point) if i in outpts else math.inf)
	
	def groupnear(self, point):
		''' return the id of the group for the nearest surface to the given point '''
		track = None
		best = math.inf
		for i,face in enumerate(self.faces):
			n = self.facenormal(i)
			dist = abs(dot(point - self.points[face[0]], n))		# TODO intergrer les limites du triangle
			if dist < best:
				track = self.tracks[i]
		return track
	
	
	# --- extraction methods ---
		
	def facenormal(self, face):
		''' normal for each face '''
		if isinstance(face, int):	
			face = self.faces[face]
		p0 = self.points[face[0]]
		e1 = self.points[face[1]] - p0
		e2 = self.points[face[2]] - p0
		return normalize(cross(e1, e2))
	
	def vertexnormals(self):
		''' normal for each point '''
		l = len(self.points)
		normals = [vec3(0) for _ in range(l)]
		for face in self.faces:
			normal = self.facenormal(face)
			for i in range(3):
				o = self.points[face[i]]
				contrib = anglebt(self.points[face[i-2]]-o, self.points[face[i-1]]-o)
				normals[face[i]] += contrib * normal
		for i in range(l):
			normals[i] = normalize(normals[i])
		return normals
	
	def facepoints(self, index):
		''' shorthand to get the points of a face (index is an int or a triplet) '''
		f = self.faces[index]
		return self.points[f[0]], self.points[f[1]], self.points[f[2]]
	
	def edges(self, oriented=True):
		''' set of unoriented edges present in the mesh '''
		edges = set()
		for face in self.faces:
			edges.add(edgekey(face[0], face[1]))
			edges.add(edgekey(face[1], face[2]))
			edges.add(edgekey(face[2], face[0]))
		return edges
	
	def group(self, groups):
		''' return a new mesh linked with this one, containing only the faces belonging to the given groups '''
		if isinstance(groups, set):			pass
		elif hasattr(groups, '__iter__'):	groups = set(groups)
		else:								groups = (groups,)
		faces = []
		tracks = []
		for f,t in zip(self.faces, self.tracks):
			if t in groups:
				faces.append(f)
				tracks.append(t)
		return Mesh(self.points, faces, tracks, self.groups)
	
	def outlines_oriented(self):
		''' return a set of the ORIENTED edges delimiting the surfaces of the mesh '''
		edges = set()
		for face in self.faces:
			for e in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
				if e in edges:	edges.remove(e)
				else:			edges.add((e[1], e[0]))
		return edges
	
	def outlines_unoriented(self):
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
	
	def outlines(self):
		''' return a Web of UNORIENTED edges '''
		return Web(self.points, self.outlines_unoriented())
		
	def groupoutlines(self):
		''' return a Web of UNORIENTED edges delimiting all the mesh groups '''
		lines = []	# outline
		tmp = {}	# faces adjacent to edges
		# insert edges adjacent to two different groups
		for i,face in enumerate(self.faces):
			for edge in ((face[0],face[1]),(face[1],face[2]),(face[2],face[0])):
				e = edgekey(*edge)
				if e in tmp:
					if self.tracks[tmp[e]] != self.tracks[i]:
						lines.append(e)
					del tmp[e]
				else:
					tmp[e] = i
		# insert edges that border only one face
		lines.extend(tmp.keys())
		return Web(self.points, lines)
	
	def surface(self):
		''' total surface of triangles '''
		s = 0
		for f in self.faces:
			a,b,c = self.facepoints(f)
			s += length(cross(a-b, a,c))
		return s
	
	
	# --- renderable interfaces ---
		
	def display_triangles(self, scene):		
		if self.options.get('debug_points', False):
			for i,p in enumerate(self.points):
				yield from text.Text(
					p, 
					' '+str(i), 
					size=8, 
					color=(0.2, 0.8, 1),
					align=('left', 'center'),
					).display(scene)
		
		if self.options.get('debug_faces', None) == 'indices':
			for i,f in enumerate(self.faces):
				p = (self.points[f[0]] + self.points[f[1]] + self.points[f[2]]) /3
				yield from text.Text(p, '  '+str(i), 9, (1, 0.2, 0), align=('left', 'center')).display(scene)
		if self.options.get('debug_faces', None) == 'tracks':
			for i,f in enumerate(self.faces):
				p = (self.points[f[0]] + self.points[f[1]] + self.points[f[2]]) /3
				yield from text.Text(p, '  '+str(self.tracks[i]), 9, (1, 0.2, 0), align=('left', 'center')).display(scene)
		
		fn = np.array([tuple(self.facenormal(f)) for f in self.faces])
		points = np.array([tuple(p) for p in self.points], dtype=np.float32)		
		lines = []
		for i in range(0, 3*len(self.faces), 3):
			lines.append((i, i+1))
			lines.append((i+1, i+2))
			lines.append((i, i+2))
		
		idents = []
		for i in self.tracks:
			idents.append(i)
			idents.append(i)
			idents.append(i)
		
		yield view.SolidDisplay(scene,
			points[np.array(self.faces, dtype='u4')].reshape((len(self.faces)*3,3)),
			np.hstack((fn, fn, fn)).reshape((len(self.faces)*3,3)),
			faces = np.array(range(3*len(self.faces)), dtype='u4').reshape(len(self.faces),3),
			idents = np.array(idents, dtype='u2'),
			lines = np.array(lines, dtype='u4'),
			color = self.options.get('color', None),
			)
	
	def display_groups(self, scene):
		facenormals = [self.facenormal(f)  for f in self.faces]
		# buffers for display
		points = array('f')
		normals = array('f')
		faces = array('L')
		edges = array('L')
		tracks = array('H')
		# function to register new points
		def usept(pi,xi,yi, fi, used):
			o = self.points[pi]
			x = self.points[xi] - o
			y = self.points[yi] - o
			contrib = anglebt(x,y)
			if pi in used:
				i = used[pi]
				# contribute to the points normals
				normals[3*i+0] += facenormals[fi][0] * contrib
				normals[3*i+1] += facenormals[fi][1] * contrib
				normals[3*i+2] += facenormals[fi][2] * contrib
				return i
			else:
				points.append(self.points[pi][0])
				points.append(self.points[pi][1])
				points.append(self.points[pi][2])
				normals.append(facenormals[fi][0] * contrib)
				normals.append(facenormals[fi][1] * contrib)
				normals.append(facenormals[fi][2] * contrib)
				tracks.append(self.tracks[fi])
				j = used[pi] = len(points) // 3 -1
				return j
		
		# get the faces for each group
		for group in range(len(self.groups)):
			# reset the points extraction for each group
			indices = {}
			frontier = set()
			# get faces and exterior edges
			for i,face in enumerate(self.faces):
				if self.tracks[i] == group and face[0] != face[1] and face[1] != face[2] and face[2] != face[0]:
					faces.append(usept(face[0],face[1],face[2], i, indices))
					faces.append(usept(face[1],face[2],face[0], i, indices))
					faces.append(usept(face[2],face[0],face[1], i, indices))
					for edge in ((face[0], face[1]), (face[1], face[2]), (face[2],face[0])):
						e = edgekey(*edge)
						if e in frontier:	frontier.remove(e)
						else:				frontier.add(e)
			# render exterior edges
			for edge in frontier:
				edges.append(indices[edge[0]])
				edges.append(indices[edge[1]])
		
		for i in range(0,len(normals),3):
			n = normalize(vec3(normals[i:i+3]))
			normals[i+0] = n[0]
			normals[i+1] = n[1]
			normals[i+2] = n[2]
		
		return view.SolidDisplay(scene,
			np.array(points).reshape((len(points)//3,3)),
			np.array(normals).reshape((len(normals)//3,3)),
			faces = np.array(faces).reshape((len(faces)//3,3)),
			lines = np.array(edges).reshape((len(edges)//2,2)),
			idents = np.array(tracks, dtype=view.IDENT_TYPE),
			color = self.options.get('color', None),
			),
	
	def display(self, scene):
		if self.options.get('debug_display', False):
			yield from self.display_triangles(scene)
		else:
			yield from self.display_groups(scene)
	
	def __repr__(self):
		return 'Mesh(\n  points= {},\n  faces=  {},\n  tracks= {},\n  groups= {},\n  options= {})'.format(
					reprarray(self.points, 'points'),
					reprarray(self.faces, 'faces'),
					reprarray(self.tracks, 'tracks'),
					reprarray(self.groups, 'groups'),
					repr(self.options))
		

def reprarray(array, name):
	if len(array) <= 5:		content = ', '.join((str(e) for e in array))
	elif len(array) <= 20:	content = ',\n           '.join((str(e) for e in array))
	else:					content = '{} {}'.format(len(array), name)
	return '['+content+']'

def striplist(list, used):
	''' remove all elements of list that match a False in used, return a reindexation list '''
	reindex = [0] * len(list)
	j = 0
	for i,u in enumerate(used):
		if u:
			list[j] = list[i]
			reindex[i] = j
			j += 1
	list[j:] = []
	return reindex


def edgekey(a,b):
	''' return a key for a non-directional edge '''
	if a < b:	return (a,b)
	else:		return (b,a)
	

def lineedges(line):
	''' yield the successive couples in line '''
	if isinstance(line, Wire):	
		line = line.indices
	line = iter(line)
	j = next(line)
	for i in line:
		yield (j,i)
		j = i


class Web(Container):
	''' set of bipoint lines, used to represent wires
		this definition is very close to the definition of Mesh, but with edges instead of triangles
		
		Attributes:
			points		list of vec3 for points
			faces		list of couples for lines, the couple is oriented (meanings of this depends on the usage)
			tracks		integer giving the group each line belong to
			groups		custom information for each group
			options		custom informations for the entire web
	'''

	# --- standard point container methods ---
	
	def __init__(self, points=None, lines=None, tracks = None, groups=None):
		self.points = points or []
		self.lines = lines or []
		self.tracks = tracks or [0] * len(self.lines)
		self.groups = groups or [None] * (max(self.tracks, default=0)+1)
		self.options = {}
	
	def transform(self, trans):
		''' apply the transform to the points of the mesh'''
		if isinstance(trans, quat):		trans = mat3_cast(trans)
		if isinstance(trans, vec3):		transformer = lambda v: v + trans
		elif isinstance(trans, mat3):	transformer = lambda v: trans * v
		elif isinstance(trans, mat4):	transformer = lambda v: vec3(trans * vec4(v,1))
		elif callable(trans):	pass
		for i in range(len(self.points)):
			self.points[i] = transformer(self.points[i])
			
	def __add__(self, other):
		''' append the faces and points of the other mesh '''
		if isinstance(other, Web):
			r = deepcopy(self)
			r.__iadd__(other)
			return r
		else:
			return NotImplemented
			
	def __iadd__(self, other):
		''' append the faces and points of the other mesh '''
		if isinstance(other, Web):		
			lp = len(self.points)
			lt = len(self.groups)
			self.points.extend(other.points)
			self.groups.extend(other.groups)
			for line,track in zip(other.lines, other.tracks):
				self.lines.append((line[0]+lp, line[1]+lp))
				self.tracks.append(track+lt)
			return self
		else:
			return NotImplemented
	
	def reverse(self):
		''' reverse direction of all lines '''
		self.faces = [(b,a)  for a,b in self.lines]
		
	# --- mesh optimization ---
	
	def mergepoints(self, merges):
		''' merge points with the merge dictionnary {src index: dst index}
			remaining points are not removed
		'''
		i = 0
		while i < len(self.lines):
			e = self.lines[i]
			self.lines[i] = l = (
				merges.get(e[0], e[0]),
				merges.get(e[1], e[1]),
				)
			if e[0] == e[1]:
				self.faces.pop(i)
				self.tracks.pop(i)
			else:
				i += 1
	
	def strippoints(self):
		''' remove points that are used by no faces, return the reindex list '''
		used = [False] * len(self.points)
		for edge in self.lines:
			for p in edge:
				used[p] = True
		reindex = striplist(self.points, used)
		for i,e in enumerate(self.lines):
			self.lines[i] = (reindex[e[0]], reindex[e[1]])
		return reindex
		
	# --- verification methods ---
			
	def isline(self):
		''' true if each point is used at most 2 times by lines '''
		reached = [0] * len(self.points)
		for line in self.lines:
			for p in line:	reached[p] += 1
		for r in reached:
			if r > 2:	return False
		return True
	
	def isloop(self):
		''' true if the wire form a loop '''
		return len(self.extremities) == 0
	
	def check(self):
		''' check that the internal data references are good (indices and list lengths) '''
		l = len(self.points)
		for line in self.faces:
			for p in line:
				if p >= l:	raise MeshError("some indices are greater than the number of points", line, l)
			if line[0] == line[1]:	raise MeshError("some lines use the same point multiple times", line)
		if len(self.faces) != len(self.tracks):	raise MeshError("tracks list doesn't match faces list length")
		if max(self.tracks) >= len(self.groups): raise MeshError("some line group indices are greater than the number of groups", max(self.tracks), len(self.groups))
		
	# --- extraction methods ---
		
	def extremities(self):
		''' return the points that are used once only '''
		extr = set()
		for l in self.lines:
			for p in l:
				if p in extr:	extr.remove(p)
				else:			extr.add(p)
		return extr
	
	def length(self):
		''' total length of edges '''
		s = 0
		for a,b in lineedges(self):
			s += distance(self.points[a], self.points[b])
		return s
		
	def __repr__(self):
		return 'Web(\n  points= {},\n  lines=  {},\n  tracks= {},\n  groups= {},\n  options= {})'.format(
					reprarray(self.points, 'points'),
					reprarray(self.lines, 'lines'),
					reprarray(self.tracks, 'tracks'),
					reprarray(self.groups, 'groups'),
					repr(self.options))


def web(*args):
	''' build a web object from parts 
		parts can be any of:	Web, Wire, Primitive, [vec3]
		arguments can be of thos types, or list of those types
	'''
	if not args:	raise TypeError('web take at least one argument')
	if len(args) == 1:	args = args[0]
	if isinstance(args, Web):		return args
	elif isinstance(args, Wire):	return Web(args.points, lineedges(args))
	elif hasattr(args, 'mesh'):
		res = args.mesh()
		if isinstance(res, tuple):
			pts,grp = res
			return Web(pts, [(i,i+1) for i in range(len(pts)-1)], groups=[grp])
		else:
			return web(res)
	elif isinstance(args, list) and isinstance(args[0], vec3):
		return Web(args, [(i,i+1) for i in range(len(args)-1)])
	elif isinstance(args, tuple) and isinstance(args[0], vec3):
		return Web(list(args), [(i,i+1) for i in range(len(args)-1)])
	elif hasattr(args, '__iter__'):
		pool = Web()
		for primitive in args:
			pool += web(primitive)
		pool.mergeclose()
		return pool
	else:
		raise TypeError('incompatible data type for Web creation')


class Wire:
	''' line as continuous suite of points 
		used to borrow reference of points from a mesh by keeping their original indices
		
		Attributes:
			points		points buffer
			indices		indices of the line's points in the buffer
	'''
	def __init__(self, points, indices=None):
		self.points = points
		if indices is None:	self.indices = list(range(len(points))) #arraylike(lambda i: i, lambda: len(points))
		else:				self.indices = indices
	
	def __len__(self):	return len(self.indices)
	def __iter__(self):	return (self.points[i] for i in self.indices)
	def __getitem__(self, i):
		if isinstance(i, int):		return self.points[self.indices[i]]
		elif isinstance(i, slice):	return [self.points[j] for j in self.indices[i]]
		else:						raise TypeError('item index must be int or slice')
	
	def length(self):
		s = 0
		for i in range(1,len(self.indices)):
			s += distance(self.points[self.indices[i-1]], self.points[self.indices[i]])
		return s
		
	def isvalid(self):
		try:				self.check()
		except MeshError:	return False
		else:				return True
	
	def check(self):
		l = len(self.points)
		for i in self.indices:
			if i >= l:	raise MeshError("some indices are greater than the number of points", i, l)

	def edge(self, i):
		return (self.indices[i+1], self.indices[i-1])
	
	def __iadd__(self, other):
		if not isinstance(other, Wire):		return NotImplemented
		if self.points is not other.points:	raise ValueError("lines doesn't refer to the same points buffer")
		self.indices.extend(other.indices)
	
	def __add__(self, other):
		if not isinstance(other, Wire):		return NotImplemented
		if self.points is not other.points:	raise ValueError("lines doesn't refer to the same points buffer")
		return Wire(self.points, self.indices+other.indices)
		
