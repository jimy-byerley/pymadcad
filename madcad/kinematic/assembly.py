# This file is part of pymadcad,  distributed under license LGPL v3
__all__ = ['Solid', 'placement', 'explode', 'explode_offsets']

from ..mathutils import *


class Solid:
	''' Solid for objects display
	
		A Solid is also a way to group objects and move it anywhere without modifying them, as the objects contained in a solid are considered to be in solid local coordinates.
		A Solid is just like a dictionary with a pose.
	
		Attributes:
			pose (mat4):   placement matrix, it defines a base in which the solid's content is displayed
			content (dict/list):      objects to display using the solid's pose
			
		Example:
			
			>>> mypart = icosphere(vec3(0), 1)
			>>> s = Solid(part=mypart, anything=vec3(0))   # create a solid with whatever inside
			
			>>> s.transform(vec3(1,2,3))   # make a new translated solid, keeping the same content without copy
			
			>>> # put any content in as a dict
			>>> s['part']
			<Mesh ...>
			>>> s['whatever'] = vec3(5,2,1)
	'''
	
	def __init__(self, pose=1, **content):
		self.pose = mat4(pose)
		self.content = content
		
	def __copy__(self):
		return Solid(self.pose, **self.content)
	
	def transform(self, value) -> 'Solid':
		''' Displace the solid by the transformation '''
		return Solid(transform(value) * self.pose, **self.content)
	
	def place(self, *args, **kwargs) -> 'Solid': 
		''' Strictly equivalent to `.transform(placement(...))`, see `placement` for parameters specifications. '''
		return Solid(placement(*args, **kwargs), **self.content)
		
	def loc(self, *args):
		transform = mat4()
		for key in args:
			transform = transform @ obj.pose
		return transform
	
	def deloc(self, *args):
		obj = self
		transform = mat4()
		for key in args:
			transform = transform @ obj.pose
			obj = obj.content[key]
		return obj.transform(transform)
	
	def reloc(self, *args):
		indev
	
	# convenient content access
	def __getitem__(self, key):
		''' Shorthand to `self.content` '''
		return self.content[key]
		
	def __setitem__(self, key, value):
		''' Shorthand to `self.content` '''
		self.content[key] = value
	
	def append(self, value):
		''' Add an item in self.content, a key is automatically created for it and is returned '''
		key = next(i 	for i in range(len(self.content)+1)
						if i not in self.content	)
		self.content[key] = value
		return key
	
	def set(self, **objs):
		''' Convenient method to set many elements in one call.
			Equivalent to `self.content.update(objs)`
		'''
		self.content.update(objs)
		return self
	
	def display(self, scene):
		from .displays import SolidDisplay
		return SolidDisplay(scene, self)


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
			if length2(boxes[i].width) > length2(boxes[j].width):
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
							+ 0.5* length(blob[i].width) / length(offsets[i]) 
							- dot(blob[i].center - barycenters[i], offsets[i]) / length2(offsets[i])
							)
			blob[j].union_update(blob[i].transform(offsets[i]))
								
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
