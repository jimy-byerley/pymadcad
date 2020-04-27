from mathutils import vec3, ivec3, noproject, norminf, normalize, dot, glm, cross, length, NUMPREC
from mesh import Web
from functools import reduce
from math import floor, ceil, sqrt, inf


class PositionMap:
	''' Holds objects assoiated with their location
		every object can be bound to multiple locations, and each location can hold multiple objects
		cellsize defines the box size for location hashing (the smaller it is, the bigger the memory footprint will be for non-point primitives)
		
		Attributes defined here:
			cellsize   - the boxing parameter (DON'T CHANGE IT IF NON-EMPTY)
			dict       - the hashmap from box to objects lists
	'''
	__slots__ = 'cellsize', 'dict', 'options'
	def __init__(self, cellsize, iterable=None):
		self.options = {}
		self.cellsize = cellsize
		self.dict = {}
		if iterable:	self.update(iterable)

	def keysfor(self, space):
		''' rasterize the primitive, yielding the successive position keys '''
		cell = self.cellsize
		# point
		if isinstance(space, vec3):
			yield tuple(ivec3(glm.floor(space/cell)))
		
		# segment
		elif isinstance(space, tuple) and len(space) == 2:
			# permutation of coordinates to get the direction the closer to Z
			n = glm.abs(space[1]-space[0])
			if max(n) < NUMPREC*max((max(p) for p in space)):	return
			if   n[1] >= n[0] and n[1] >= n[2]:	order,reorder = (2,0,1),(1,2,0)
			elif n[0] >= n[1] and n[0] >= n[2]:	order,reorder = (1,2,0),(2,0,1)
			else:								order,reorder = (0,1,2),(0,1,2)
			space = [ vec3(p[order[0]], p[order[1]], p[order[2]])	for p in space]
			
			# prepare variables
			cell2 = cell/2
			v = space[1]-space[0]
			dy = v[1]/v[2]
			dx = v[0]/v[2]
			o = space[0]
			fy = lambda z:	o[1] + dy*(z-o[2])
			fx = lambda z:	o[0] + dx*(z-o[2])
			
			# z selection
			if v[2] > 0:	zmin,zmax = space[0][2], space[1][2]
			else:			zmin,zmax = space[1][2], space[0][2]
			zmin -= zmin%cell
			for i in range(max(1,ceil((zmax-zmin)/cell))):
				z = zmin + cell*i + cell2
				
				# y selection
				if dy > 0:	ymin,ymax = fy(z-cell2), fy(z+cell2)
				else:		ymin,ymax = fy(z+cell2), fy(z-cell2)
				ymin -= ymin%cell
				for j in range(max(1,ceil((ymax-ymin)/cell))):
					y = ymin + j*cell + cell2
					
					# x selection
					if dx > 0:	xmin,xmax = fx(z-cell2), fx(z+cell2)
					else:		xmin,xmax = fx(z+cell2), fx(z-cell2)
					xmin -= xmin%cell
					for k in range(max(1,ceil((xmax-xmin)/cell))):
						x = xmin + k*cell + cell2
						
						p = (x,y,z)
						yield tuple([floor(p[i]/cell)		for i in reorder])
		
		# triangle
		elif isinstance(space, tuple) and len(space) == 3:
			# permutation of coordinates to get the normal the closer to Z
			n = glm.abs(cross(space[1]-space[0], space[2]-space[0]))
			if max(n) < NUMPREC*max((max(p) for p in space)):	return
			if   n[1] >= n[0] and n[1] >= n[2]:	order,reorder = (2,0,1),(1,2,0)
			elif n[0] >= n[1] and n[0] >= n[2]:	order,reorder = (1,2,0),(2,0,1)
			else:								order,reorder = (0,1,2),(0,1,2)
			space = [ vec3(p[order[0]], p[order[1]], p[order[2]])	for p in space]
			
			# prepare variables
			v = [space[i-1]-space[i]	for i in range(3)]
			n = cross(v[0],v[1])
			dx = -n[0]/n[2]
			dy = -n[1]/n[2]
			o = space[0]
			fz = lambda x,y:	o[2] + dx*(x-o[0]) + dy*(y-o[1])
			cell2 = cell/2
			pmin = reduce(glm.min, space)
			pmax = reduce(glm.max, space)
			xmin,xmax = pmin[0],pmax[0]
			pmin -= pmin%cell
			pmax += cell - pmax%cell
			
			# x selection
			xmin -= xmin%cell
			for i in range(max(1,ceil((xmax-xmin)/cell))):
				x = xmin + cell*i + cell2
			
				# y selection
				cand = []
				for i in range(3):
					# NOTE: cet interval ajoute parfois des cases inutiles apres les sommets
					if (space[i-1][0]-x+cell2)*(space[i][0]-x-cell2) <= 0 or (space[i-1][0]-x-cell2)*(space[i][0]-x+cell2) <= 0:
						d = v[i][1]/(v[i][0] if v[i][0] else inf)
						cand.append( space[i][1] + d * (x-cell2-space[i][0]) )
						cand.append( space[i][1] + d * (x+cell2-space[i][0]) )
				ymin,ymax = max(pmin[1],min(cand)), min(pmax[1],max(cand))
				ymin -= ymin%cell
				#print(cand)
				for j in range(max(1,ceil((ymax-ymin)/cell))):
					y = ymin + cell*j + cell2
				
					# z selection
					cand = []
					cand.append( fz(x-cell2, y-cell2) )
					cand.append( fz(x+cell2, y-cell2) )
					cand.append( fz(x-cell2, y+cell2) )
					cand.append( fz(x+cell2, y+cell2) )
					zmin,zmax = max(pmin[2],min(cand)), min(pmax[2],max(cand))
					zmin -= zmin%cell
					for k in range(max(1,ceil((zmax-zmin)/cell))):
						z = zmin + cell*k + cell2
						
						# remove box from corners that goes out of the area
						p = (x,y,z)
						if pmin[0]<p[0] and pmin[1]<p[1] and pmin[2]<p[2] and p[0]<pmax[0] and p[1]<pmax[1] and p[2]<pmax[2]:
							yield tuple([floor(p[i]/cell)	for i in reorder])
		
		else:
			raise TypeError("PositionMap only supports keys of type:  points, segments, triangles")
	
	def update(self, other):
		if isinstance(other, PositionMap):
			assert self.cellsize == other.cellsize
			for k,v in other.dict.items():
				if k in self.dict:	self.dict[k].extend(v)
				else:				self.dict[k] = v
		elif hasattr(other, '__iter__'):
			for space,obj in other:
				self.add(space,obj)
		else:
			raise TypeError("update requires a PositionMap or an iterable of couples (space, obj)")
	
	def add(self, space, obj):
		for k in self.keysfor(space):
			if k not in self.dict:	self.dict[k] = [obj]
			else:					self.dict[k].append(obj)
	
	def get(self, space):
		for k in self.keysfor(space):
			if k in self.dict:
				yield from self.dict[k]
	
	_display = Web(
		[	
			vec3(0,0,0),
			vec3(1,0,0),vec3(0,1,0),vec3(0,0,1),
			vec3(0,1,1),vec3(1,0,1),vec3(1,1,0),
			vec3(1,1,1)],
		[	
			(0,1),(0,2),(0,3),
			(1,6),(2,6),
			(1,5),(3,5),
			(2,4),(3,4),
			(4,7),(5,7),(6,7),
			],
		)
	def display(self, scene):
		web = Web()
		if 'color' in self.options:		web.options['color'] = self.options['color']
		base = vec3(self.cellsize)
		for k in self.dict:
			l = len(web.points)
			web += Web([base*(p+k)  for p in self._display.points], self._display.edges, groups=[k])
		return web.display(scene)

def meshcellsize(mesh):
	''' returns a good cell size to index primitives of a mesh with a PositionMap '''
	# the number of key for triangles is propotionate to the surface
	# points are placed on this surface, so sqrt(len(pts)) is a good hint for the point density on the surface
	return length(mesh.box().width) / sqrt(len(mesh.points))


class PointSet:
	''' Holds a list of points and hash them.
		the points are holds using indices, that allows to get the point buffer at any time, or to retreive only a point index
		cellsize defines the box size in which two points are considered to be the same
		
		methods are inspired from the builtin type set
		
		Attributes defined here:
			points     - the point buffer (READ-ONLY PURPOSE)
			cellsize   - the boxing parameter (DON'T CHANGE IT IF NON-EMPTY)
			dict       - the hashmap from box to point indices
	'''
	__slots__ = 'points', 'cellsize', 'dict'
	def __init__(self, cellsize, iterable=None):
		self.points = []
		self.dict = {}
		if iterable:	self.update(iterable)
	
	def keyfor(self, pt):
		return tuple(ivec3(pt/self.cellsize))
	
	def update(self, iterable):
		for pt in iterable:	self.add(pt)
	def difference_update(self, iterable):
		for pt in iterable:	self.discard(pt)
		
	def add(self, pt):
		key = self.keyfor(pt)
		if key not in self.dict:
			self.dict[key] = len(self.points)
			self.points.append(pt)
	def remove(self, pt):
		key = self.keyfor(p)
		if key in self.dict:	del self.dict[k]
		else:					raise IndexError("position doesn't exist in set")
	def discard(self, pt):
		key = self.keyfor(p)
		if key in self.dict:	del self.dict[k]
	
	def __contains__(self, pt):
		return self.keyfor(p) in self.dict
	def __getitem__(self, pt):
		key = self.keyfor(pt)
		if key in self.dict:	return self.dict[key]
		else:					raise IndexError("position doesn't exist in set")
		
	__iadd__ = update
	__isub__ = difference_update
	def __add__(self, iterable):
		s = PositionSet()
		s.union(self.points)
		s.union(iterable)
		return s
	def __sub__(self, iterable):
		s = PositionSet()
		s.union(self.points)
		s.difference(iterable)
		return s


if __name__ == '__main__':
	from mesh import Mesh
	from nprint import nprint
	
	triangles = Mesh([
						vec3(1,1,0), vec3(3,1,0), vec3(1,3,0),
						vec3(0,1,1), vec3(0,3,1), vec3(0,1,4),
						vec3(1,0,1), vec3(3,0,1), vec3(1,0,4),
						vec3(1,2,5), vec3(-3,0,4), vec3(4,-2,7),
						],
					[(0,1,2), (3,4,5), (6,7,8), (9,10,11)],
					)
	lines = Web([
					vec3(5,0,0), vec3(5,0,4),
					vec3(0,5,0), vec3(0,6,4),
					#vec3(-6,-8,9), vec3(-2,-5,2), 
					vec3(-6,-7,2), vec3(-4,-5,9),
					],
				[(0,1),(2,3),(4,5)],
				)
	m = PositionMap(3.7, [
		(triangles.facepoints(0), 'x'),
		(triangles.facepoints(1), 'y'),
		(triangles.facepoints(2), 'z'),
		(triangles.facepoints(3), 'truc'),
		((vec3(5,0,0), vec3(5,0,4)), 'a'),
		((vec3(0,5,0), vec3(0,6,4)), 'b'),
		((vec3(-6,-7,2), vec3(-4,-5,9)), 'c'),
		])
	
	nprint(m.dict)
	print(set( m.get((vec3(8,0,0),vec3(0,8,0),vec3(-2,-2,8))) ))
	
	from mathutils import Box,fvec3
	import view
	import sys
	from PyQt5.QtWidgets import QApplication
	app = QApplication(sys.argv)
	scn = view.Scene()
	m.options['color'] = (0.7, 0.9, 1)
	scn.add(m)
	scn.add(triangles)
	scn.add(lines)
	scn.look(Box(center=fvec3(0,0,3), width=fvec3(4)))
	scn.show()
	sys.exit(app.exec())