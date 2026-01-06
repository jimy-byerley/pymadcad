'''
	convenient data structures and functions for putting parts together and explode them to other functions
'''

from __future__ import annotations

from itertools import chain
from pnprint import nprint
from dataclasses import dataclass

from ..mathutils import *

# This file is part of pymadcad,  distributed under license LGPL v3
__all__ = ['Solid', 'placement', 'explode', 'explode_offsets', 'SolidBox']


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


def explode(solids, spacing=1, offsets=None) -> list[Solid]:
	''' Move the given solids away from each other in the way of an exploded view.
		It makes easier to seen the details of an assembly . See `explode_offsets` for the algorithm.
		
		Parameters:
			
			solids:		a list of solids (copies of each will be made before displacing)
			spacing:	spacing factor, 0 for no displacement, 1 for normal displacement
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
	if not offsets:
		offsets = explode_offsets([
			SolidBox(
				box = boundingbox(list(solid.values()), ignore=True),
				place = solid.pose,
				)
			for solid in solids], spacing)
	return [solid.transform(offset)  for solid, offset in zip(solids, offsets)]


	
@dataclass(slots=True)
class SolidBox:
	''' structure just for `explode_offsets` '''
	box: Box
	''' object's bounding boxes in local space '''
	place: mat4
	''' transform from local to common space '''
	exploded: Box = None
	''' if given, it must be a list of object's bounding boxes in their local space representing the size of the object when internally exploded '''

def explode_offsets(solids:list[SolidBox], spacing=0.) -> list[vec3]:
	''' return offsets for an exploded view
		
		Args:
			solids: objects to explode, described as boundingboxes in their own space
			spacing:  spacing ratio relative to objects sizes (0 means no spacing, 0.5 is a good value)
	'''
	# create a tree of almost enclosing boxes
	# criterion is intersection over sum
	def ios(a, b):
		return a.intersection(b).volume() / (a.volume() + b.volume())
	def nest(root, new, threshold):
		# try to insert in one of the children
		if root.children:
			parent = max(root.children, key=lambda parent: ios(new.world, parent.world))
			score = ios(new.world, parent.world)
			if score > threshold:
				nest(parent, new, score)
				return
		# otherwise insert here
		root.children.append(new)
	
	root = _Node(None, 
		world = Box(size=-inf), 
		local = Box(size=-inf), 
		exploded = Box(size=-inf), 
		place = mat4(), 
		children = [],
		offset = vec3(),
		)
	for index, solid in sorted(enumerate(solids), key=lambda item: item[1].box.volume(), reverse=True):
		nest(root, _Node(index, 
			world = solid.box.transform(solid.place), 
			local = solid.box, 
			exploded = solid.exploded or solid.box,
			place = solid.place, 
			children = [],
			offset = None,
			), 0)
	
	# explode boxes recursively
	def inner_distance(parent, child):
		return min(glm.min(child.min - parent.min, parent.max - child.max) / parent.size)
	def place(node):
		# process lower level first
		for child in node.children:
			place(child)
		# bounding points of already exploded boxes at this level
		exploded = [node.place * p   for p in node.exploded.corners() if isfinite(p)]
		# process box in this level from most inner to most outer
		ordered = sorted(node.children, key=lambda child: inner_distance(node.world, child.world), reverse=True)
		for child in ordered:
			# choose offset direction the closest to exterior of enclosing box
			bounds = _box_planes(node.local, node.place)
			shape = _box_planes(child.local, child.place)
			# move in the direction where the needed offset is the smallest
			direction = - max(shape, 
				key=lambda axis: max(
					dot(axis.origin - bound.origin, -axis.direction)
					for bound in bounds
					if dot(axis.direction, bound.direction) < 0)
				).direction
			
			shape_length = max(dot(axis.origin, direction)  for axis in shape) - min(dot(axis.origin, direction)  for axis in shape)
			shape_bot = min(dot(axis.origin, direction)  for axis in _box_planes(child.exploded, child.place))
			explode_top = max((dot(p, direction)  for p in exploded), default=shape_bot)
			
			# compute offset along direction to get the box out of the already exploded area
			offset = max(0, explode_top - shape_bot) + spacing*mix(shape_length, max(child.world.size), 0.1)
			# update tree
			child.offset = direction * offset
			exploded.extend(offset + child.place * p   for p in child.exploded.corners())
		# publish new exploded bounds for upper level
		node.exploded |= boundingbox(affineInverse(node.place) * p  for p in exploded)
	place(root)
	
	# collect offsets from the tree
	offsets = [vec3()  for box in solids]
	def get(node, offset):
		offsets[node.index] = node.offset + offset
		for child in node.children:
			get(child, node.offset + offset)
	for node in root.children:
		get(node, vec3(0))
	return offsets

def _box_planes(box, transform):
	''' return a list of one axis per box face, transformed '''
	m = box.to_matrix(centered=True)
	return [Axis(m*d, d).transform(transform)  for d in (-X,-Y,-Z, +X,+Y,+Z)]
	
@dataclass
class _Node:
	''' node representation in the box nesting algorithm of explode_offsets '''
	index: int
	world: Box
	local: Box
	exploded: Box
	place: mat4()
	children: list
	offset: vec3
