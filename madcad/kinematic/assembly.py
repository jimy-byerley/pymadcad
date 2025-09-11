# This file is part of pymadcad,  distributed under license LGPL v3
__all__ = ['Solid', 'placement', 'explode', 'explode_offsets']

from ..mathutils import *


class Solid(dict):
	''' Movable group of objects
	
		A Solid is just like a dictionary with a pose.
			
		Example:
			
			>>> mypart = icosphere(vec3(0), 1)
			>>> s = Solid(part=mypart, anything=vec3(0))   # create a solid with whatever inside
			
			>>> st = s.transform(vec3(1,2,3))   # make a new translated solid, keeping the same content without copy
			
			>>> # use content as attributes
			>>> s.part
			<Mesh ...>
			
			>>> # put any content in as a dict
			>>> s['part']
			<Mesh ...>
			>>> s['whatever'] = vec3(5,2,1)
	'''
	
	pose: mat4
	''' placement matrix, it defines a base in which other attributes are displayed '''
	
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.__dict__ = self
		if 'pose' not in self:
			self.pose = mat4()
	
	def transform(self, value) -> 'Solid':
		''' Displace the solid by the transformation '''
		return Solid(self).update(pose = transform(value) * self.pose)
	
	def place(self, *args, **kwargs) -> 'Solid': 
		''' Strictly equivalent to `.transform(placement(...))`, see `placement` for parameters specifications. '''
		return self.transform(placement(*args, **kwargs))
		
	def loc(self, *args) -> object:
		''' chain transforms in the given key path '''
		obj, transform = self._unroll(args)
		return transfrom
	
	def deloc(self, *args) -> object:
		''' return the object at the given key path with the chain of transforms applied to it '''
		obj, transform = self._unroll(args)
		return obj.transform(transform)
	
	def _unroll(self, args):
		''' seek on object in nested solids and return the object and the chain of transformations to it '''
		obj = self
		transform = mat4()
		for key in args:
			transform = transform @ obj.pose
			obj = obj[key]
		return obj, transform
	
	def __reduce__(self):		return type(self), (dict(self),)
	
	def update(self, *args, **kwargs) -> 'Solid':
		super().update(*args, **kwargs)
		return self
	
	def append(self, value) -> int:
		''' Add an item in self.content, a key is automatically created for it and is returned '''
		key = next(i 	for i in range(len(self.content)+1)
						if i not in self.content	)
		self[key] = value
		return key
		
	def __repr__(self):
		return '{}({})'.format(
			self.__class__.__name__,
			','.join('{}={}'.format(key, repr(value))  for key,value in self.items()),
			)
	
	def display(self, scene):
		from .displays import SolidDisplay
		return SolidDisplay(scene, self)
		
	def __eq__(self, other):
		''' equaly when all members including pose are equal '''
		return self is other or isinstance(other, Solid) and (
			len(self.__dict__) == len(other.__dict__)
			and all(self.__dict__.get(k) == other.__dict__.get(k)  for k in self.__dict__ if not k.startswith('_'))
			)


def placement(*pairs, precision=1e-3):
	''' Return a transformation matrix that solved the placement constraints given by the surface pairs
	
		Parameters:
		
			pairs:	a list of pairs to convert to kinematic joints
					
					- items can be couples of surfaces to convert to joints using `guessjoint`
					- tuples (joint_type, a, b)  to build joints `joint_type(solida, solidb, a, b)`
			
			precision: surface guessing and kinematic solving precision (distance)
		
		Each pair define a joint between the two assumed solids (a solid for the left members of the pairs, and a solid for the right members of the pairs). Placement will return the pose of the first relatively to the second, satisfying the constraints.
		
		Example:
		
			>>> # get the transformation for the pose
			>>> pose = placement(
			...		(screw['part'].group(0), other['part'].group(44)),  # two cylinder surfaces: Cylindrical joint
			...		(screw['part'].group(4), other['part'].group(25)),    # two planar surfaces: Planar joint
			...		)  # solve everything to get solid's pose
			>>> # apply the transformation to the solid
			>>> screw.pose = pose
			
			>>> # or
			>>> screw.place(
			...		(screw['part'].group(0), other['part'].group(44)),
			...		(screw['part'].group(4), other['part'].group(25)),
			...		)
			
			>>> screw.place(
			...		(Revolute, screw['axis'], other['screw_place']),
			...		)
	'''
	from ..reverse import guessjoint
	
	joints = []
	for pair in pairs:
		if len(pair) == 2:		joints.append(guessjoint((0, 1), *pair, precision*0.25))
		elif len(pair) == 3:	joints.append(pair[0]((0, 1), *pair[1:]))
		else:
			raise TypeError('incorrect pair definition', pair)
	
	if len(joints) > 1:
		kin = Kinematic(joints)
		parts = kin.parts(kin.solve())
		return affineInverse(parts[1]) * parts[0]
	else:
		return affineInverse(joints[0].direct(joints[0].default))
	
def convexhull(pts):
	import scipy.spatial
	if len(pts) == 3:
		return Mesh(pts, [(0,1,2),(0,2,1)])
	elif len(pts) > 3:
		hull = scipy.spatial.ConvexHull(typedlist_to_numpy(pts, 'f8'))
		m = Mesh(pts, hull.simplices.tolist())
		return m
	else:
		return Mesh(pts)

			
def extract_used(obj):
	if isinstance(obj, Mesh):	links = obj.faces
	elif isinstance(obj, Web):	links = obj.edges
	elif isinstance(obj, Wire):	links = [obj.indices]
	else:
		raise TypeError('obj must be a mesh of any kind')
	
	return striplist(obj.points[:], links)[0]

	
def explode_offsets(solids) -> '[(solid_index, parent_index, offset, barycenter)]':
	''' Build a graph of connected objects, ready to create an exploded view or any assembly animation.
		See `explode()` for an example. The exploded view is computed using the meshes contained in the given solids, so make sure there everything you want in their content.
	
		Complexity is `O(m * n)` where m = total number of points in all meshes, n = number of solids
		
		NOTE:
			
			Despite the hope that this function will be helpful, it's (for computational cost reasons) not a perfect algorithm for complex assemblies (the example above is at the limit of a simple one). The current algorithm will work fine for any simple enough assembly but may return unexpected results for more complex ones.
		
	'''
	import scipy.spatial.qhull
	# build convex hulls
	points = [[] for s in solids]
	# recursively search for meshes in solids
	def process(i, solid):
		if hasattr(solid.content, 'values'):	it = solid.content.values()
		else:									it = solid.content
		for obj in solid.content.values():
			if isinstance(obj, Solid):
				process(i, obj)
			elif isinstance(obj, (Mesh,Web,Wire)):
				try:
					points[i].extend(extract_used(convexhull(extract_used(obj)).transform(solid.pose)))
				except scipy.spatial.qhull.QhullError:
					continue
			
	for i,solid in enumerate(solids):
		process(i,solid)
	
	# create convex hulls and prepare for parenting
	hulls = [convexhull(pts).orient()  for pts in points]
	boxes = [hull.box()  for hull in hulls]
	normals = [hull.vertexnormals()  for hull in hulls]
	barycenters = [hull.barycenter()  for hull in hulls]
	
	scores = [inf] * len(solids)
	parents = [None] * len(solids)
	offsets = [vec3(0)] * len(solids)
	
	# build a graph of connected things (distance from center to convex hulls)
	for i in range(len(solids)):
		center = barycenters[i]	
		for j in range(len(solids)):
			if i == j:
				continue
			
			# case of non-connection, the link won't appear in the graph
			if boxes[i].intersection(boxes[j]).isempty():
				continue
			
			# the parent is always the biggest of the two, this also breaks any possible parenting cycle
			if length2(boxes[i].size) > length2(boxes[j].size):
				continue
			
			# select the shortest link
			d, prim = distance2_pm(center, hulls[j])
			
			if d < scores[i]:
				# take j as new parent for i
				scores[i] = d
				parents[i] = j
				
				# get the associated displacement vector
				pts = hulls[j].points
				if isinstance(prim, int):
					normal = normals[j][prim]
					offsets[i] = center - pts[prim]
				elif len(prim) == 2:
					normal = normals[j][prim[0]] + normals[j][prim[1]]
					offsets[i] = noproject(center - pts[prim[0]],  
										pts[prim[0]]-pts[prim[1]])
				elif len(prim) == 3:
					normal = cross(pts[prim[1]]-pts[prim[0]], pts[prim[2]]-pts[prim[0]])
					offsets[i] = project(center - pts[prim[0]],  normal)
				else:
					raise AssertionError('prim should be an index for  point, face, triangle')
				if dot(offsets[i], normal) < 0:
					offsets[i] = -offsets[i]
	
	# resolve dependencies to output the offsets in the resolution order
	order = []
	reached = [False] * len(solids)
	i = 0
	while i < len(solids):
		if not reached[i]:
			j = i
			chain = []
			while not (j is None or reached[j]):
				reached[j] = True
				chain.append(j)
				j = parents[j]
			order.extend(reversed(chain))
		i += 1
	
	# move more parents that have children on their way out				
	blob = [deepcopy(box) 	for box in boxes]
	for i in reversed(range(len(solids))):
		j = parents[i]
		if j and length2(offsets[i]):
			offsets[i] *= (1 
							+ 0.5* length(blob[i].size) / length(offsets[i]) 
							- dot(blob[i].center - barycenters[i], offsets[i]) / length2(offsets[i])
							)
			blob[j].merge_update(blob[i].transform(offsets[i]))
								
	return [(i, parents[i], offsets[i], barycenters[i])  for i in order]
			
	
def explode(solids, factor=1, offsets=None) -> '(solids:list, graph:Mesh)':
	''' Move the given solids away from each other in the way of an exploded view.
		It makes easier to seen the details of an assembly . See `explode_offsets` for the algorithm.
		
		Parameters:
			
			solids:		a list of solids (copies of each will be made before displacing)
			factor:		displacement factor, 0 for no displacement, 1 for normal displacement
			offsets:	if given, must be the result of `explode_offsets(solids)`
		
		Example:
		
			>>> # pick some raw model and separate parts
			>>> imported = read(folder+'/some_assembly.stl')
			>>> imported.mergeclose()
			>>> parts = []
			>>> for part in imported.islands():
			...     part.strippoints()
			...     parts.append(Solid(part=segmentation(part)))
			... 
			>>> # explode the assembly to look into it
			>>> exploded = explode(parts)
		
	'''
	solids = [copy(solid)  for solid in solids]
	if not offsets:
		offsets = explode_offsets(solids)
	
	graph = Web(groups=[None])
	shifts = [	(solids[solid].position - solids[parent].position)
				if parent else vec3(0)
				for solid, parent, offset, center in offsets]
	for solid, parent, offset, center in offsets:
		if parent:
			solids[solid].position = solids[parent].position + shifts[solid] + offset * factor
			
			graph.edges.append((len(graph.points), len(graph.points)+1))
			graph.tracks.append(0)
			graph.points.append(solids[parent].position + shifts[solid] + center)
			graph.points.append(solids[solid].position + center)
			
	return [solids, graph]

from itertools import chain
from pnprint import nprint
def explode_offsets(boxes:list[Box], places:list[mat4], spacing=0.) -> list[vec3]:
	# create a tree of almost enclosing boxes
	def ios(a, b):
		return a.intersection(b).volume() / (a.volume() + b.volume())
	def nest(root, new, threshold):
		if root.children:
			parent = max(root.children, key=lambda parent: ios(new.world, parent.world))
			score = ios(new.world, parent.world)
			if score > threshold:
				nest(parent, new, score)
				return
		root.children.append(new)
	
	root = _Node(None, 
		world=Box(width=-inf), 
		local=Box(width=-inf), 
		place=mat4(), 
		children=[],
		offset=vec3(),
		)
	for index, box in sorted(enumerate(boxes), key=lambda item: item[1].volume(), reverse=True):
		nest(root, _Node(index, 
			world=box.transform(places[index]), 
			local=box, 
			place=places[index], 
			children=[],
			offset=None,
			), 0)
	
	# explode boxes recursively
	def inner_distance(parent, child):
		return min(glm.min(child.min - parent.min, parent.max - child.max) / parent.size)
	def place(node):
		for child in node.children:
			place(child)
		placed = [node.place * p   for p in node.local.corners() if isfinite(p)]
		ordered = sorted(node.children, key=lambda child: inner_distance(node.world, child.world), reverse=True)
		for child in ordered:
			# choose offset direction the closest to exterior of enclosing box
			bounds = _box_planes(node.local, node.place)
			shape = _box_planes(child.local, child.place)
			direction = max(shape, 
				key=lambda axis: max(dot(axis.origin - bound.origin, bound.direction)  for bound in bounds) * (
						max(dot(axis.direction, bound.origin)  for bound in bounds)
						- min(dot(axis.direction, bound.origin)  for bound in bounds)
						)
				).direction
			shape_bot = min(dot(axis.origin, direction)  for axis in _box_planes(child.world, 1))
			shape_top = max(dot(axis.origin, direction)  for axis in _box_planes(child.world, 1))
			explode_top = max((dot(p, direction)  for p in placed), default=shape_bot)
			
			offset = explode_top - shape_bot + spacing*(shape_top - shape_bot)
			child.offset = direction * offset
			child.world = child.world.transform(translate(child.offset))
			placed.extend(offset + child.place * p   for p in child.local.corners())
			
		node.world |= boundingbox(child.world  for child in node.children)
	place(root)
	
	offsets = [vec3()  for box in boxes]
	def get(node, offset):
		offsets[node.index] = node.offset + offset
		for child in node.children:
			get(child, node.offset + offset)
	for node in root.children:
		get(node, vec3(0))
	return offsets

def _box_planes(box, place):
	m = box.to_matrix(centered=True)
	# return [Axis(m*d, d/dot(box.size, d)).transform(place)  for d in (-X,-Y,-Z, +X,+Y,+Z)]
	return [Axis(m*d, d).transform(place)  for d in (-X,-Y,-Z, +X,+Y,+Z)]
	
from dataclasses import dataclass
@dataclass
class _Node:
	index: int
	world: Box
	local: Box
	place: mat4()
	children: list
	offset: vec3
	
	
	
# def explode_offsets(boxes:list[Box]) -> list[vec3]:
# 	tree = {}
# 	for box in sorted(boxes, key=Box.volume):
# 		parent, iou = min((
# 			(boxes, intersection(box, boxes[parent]).volume() / union(box, boxes[parent]).volume())
# 			for parent in tree
# 			), key=itemgetter(1))
# 		if iou > 

import numpy as np
def explode_offsets_(boxes:list[Box], places:list[mat4], spacing=0.) -> list[vec3]:
	offsets = [vec3(0)  for box in boxes]
	gboxes = [box.transform(place)  for box,place in zip(boxes, places)]
	gbounds = boundingbox(gboxes)
	# sort by distance to bounding box
	ordered = sorted(range(len(boxes)), 
			key=lambda i:  min(glm.min((gbounds.min - gboxes[i].min), (gbounds.max - gboxes[i].max)) / gbounds.size), 
			reverse=True)
	print('  order', ordered)
	for k,i in enumerate(ordered):
		if not k:
			continue
		# choose local offset direction the closest to side
		iplace = affineInverse(places[i])
		lbounds = gbounds.transform(iplace)
		# closest side
		# idir = np.argmin(np.concatenate([boxes[i].min - lbounds.min, lbounds.max - boxes[i].max]))
		# opposite to farest side
		idir = np.argmax(np.concatenate([(lbounds.max - boxes[i].max)/lbounds.size, (boxes[i].min - lbounds.min)/lbounds.size]))
		bots = np.concatenate([-boxes[i].max, boxes[i].min])
		dir = vec3()
		dir[idir%3] = -1 if idir<3 else +1
		
		# dir = vec3(0,0,1)
		# move on the top of already moved parts
		top = max( dot(iplace * (p + offsets[j]), dir)  for j in ordered[:k] for p in gboxes[j].corners() )
		print('    top', dir, top)
		offset = top - bots[idir]
		offsets[i] = mat3(places[i]) * dir * (offset + dot(boxes[i].size, dir)*spacing)
	return offsets
