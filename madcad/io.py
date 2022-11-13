# This file is part of pymadcad,  distributed under license LGPL v3

import numpy as np
import numpy.lib.recfunctions as rfn
import os, tempfile
from functools import wraps
from hashlib import md5

from .mathutils import vec3, glm, inf, typedlist
from .mesh import Mesh, Wire

class FileFormatError(Exception):	pass


def filetype(name, type=None):
	''' get the name for the file format, using the given forced type or the name extension '''
	if not type:
		type = name[name.rfind('.')+1:]
	if not type:
		raise FileFormatError('unable to guess the file type')
	return type
	
def read(name: str, type=None, **opts) -> Mesh:
	''' load a mesh from a file, guessing its file type '''
	type = filetype(name, type)
	reader = globals().get(type+'_read')
	if reader:
		return reader(name, **opts)
	else:
		raise FileFormatError('no read function available for format '+type)

def write(mesh: Mesh, name: str, type=None, **opts):
	''' write a mesh to a file, guessing its file type '''
	type = filetype(name, type)
	writer = globals().get(type+'_write')
	if writer:
		return writer(mesh, name, **opts)
	else:
		raise FileFormatError('no write function available for format '+type)

cachedir = tempfile.gettempdir() + '/madcad-cache'
caches = {}
def cache(filename: str, create: callable=None, name=None, storage=None, **opts) -> Mesh:
	''' Small cachefile system, it allows to dump objects to files and to get them when needed.
		
		It's particularly usefull when working with other processes. The cached files are reloaded only when the cache files are newer than the memory cache data
		
		If specified, create() is called to provide the data, in case it doesn't exist in memory neighter as file
		if specified, name is the cache name used to index the file it defaults to the filename
		if specified, storage is the dictionnary used to storage cache data, defaults to io.caches
	'''
	if not storage:	storage = caches
	if not name:	name = filename
	
	# create the cache file if it doesn't exist
	if os.path.exists(filename):
		cachedate = storage[name][0] if name in storage else -inf
		filedate = os.path.getmtime(filename)
		if cachedate < filedate:
			storage[name] = (filedate, read(filename, **opts))
	# load reload the file content if it's newer that the data in memory
	else:
		if name in storage:	obj = storage[name][1]
		elif create:		obj = create()
		else:
			raise IOError("the cache file doesn't exist")
		write(obj, filename, **opts)
		storage[name] = (os.path.getmtime(filename), obj)
	return storage[name][1]
	
	

def cachefunc(f):
	''' Decorator to cache a function results.
		
		Use it if you want to cache their result associated with the argument set used 
	'''
	@wraps(f)
	def repl(*args, **kwargs):
		if not os.path.exists(cachedir):
			os.makedirs(cachedir)
		key = '{}/{}{}-{}.pickle'.format(
			cachedir,
			f.__module__ + '.' if f.__module__ else '',
			f.__name__,
			hex(int.from_bytes(
				md5(repr((
					*args, 
					sorted(list(kwargs.items()))
					)).encode())
					.digest(),
				'little')),
			)
		return cache(key, lambda: f(*args, **kwargs))
	return repl

	
'''
	pickle files are the standard python serialized files, they are absolutely not secure ! so do not use it for something else than your own caching.
'''
import pickle

def pickle_read(file, **opts):
	return pickle.load(open(file, 'rb'))
	
def pickle_write(obj, file, **opts):
	return pickle.dump(obj, open(file, 'wb'))

'''
	PLY is loaded using plyfile module 	https://github.com/dranjan/python-plyfile
	using the specifications from 	https://web.archive.org/web/20161221115231/http://www.cs.virginia.edu/~gfx/Courses/2001/Advanced.spring.01/plylib/Ply.txt
		(also locally available in ply-description.txt)
'''
try:
	from plyfile import PlyData, PlyElement
except ImportError:	pass
else:
	from . import triangulation

	def ply_read(file, **opts):
		mesh = Mesh()
		
		data = PlyData.read(file)
		if 'vertex' not in data:	raise FileFormatError('file must have a vertex buffer')
		if 'face' not in data:		raise FileFormatError('file must have a face buffer')
		
		# collect points
		mesh.points = typedlist(data['vertex'].data.astype('f8, f8, f8'), dtype=vec3)
		
		# collect faces
		faces = data['face'].data
		if faces.dtype.names[0] == 'vertex_indices':
			for face in faces['vertex_indices']:
				#print('  ', type(face), face, face.dtype, face.strides)
				if len(face) == 3:	# triangle
					mesh.faces.append(face)
				elif len(face) > 3:	# quad or other extended face
					mesh += triangulation.triangulation_outline(Wire(mesh.points, face))
		else:
			mesh.faces = numpy_to_typedlist(faces.astype('u4'), dtype=uvec3)

		# collect tracks
		if 'group' in faces.dtype.names:
			mesh.tracks = typedlist(faces['group'].astype('u4'), dtype='I')
		else:
			mesh.tracks = typedlist.full(0, len(mesh.faces), 'I')
		
		# create groups  (TODO find a way to get it from the file, PLY doesn't support non-scalar types)
		mesh.groups = [None] * (max(mesh.tracks, default=-1)+1)
		
		return mesh

	def ply_write(mesh, file, **opts):
		vertices = np.array(mesh.points, copy=False).astype(np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')]))
		faces = np.empty(len(mesh.faces), dtype=[('vertex_indices', 'u4', (3,)), ('group', 'u2')])
		faces['vertex_indices'] = typedlist_to_numpy(mesh.faces, 'u4')
		faces['group'] = typedlist_to_numpy(mesh.tracks, 'u4')
		ev = PlyElement.describe(vertices, 'vertex')
		ef = PlyElement.describe(faces, 'face')
		PlyData([ev,ef], opts.get('text', False)).write(file)


'''
	STL is loaded using numpy-stl module 	https://github.com/WoLpH/numpy-stl
'''
try:	
	import stl
except ImportError:	pass
else:

	from .mathutils import *
	from .mesh import numpy_to_typedlist, typedlist_to_numpy
	def stl_read(file, **opts):
		stlmesh = stl.mesh.Mesh.from_file(file, calculate_normals=False)
		trinum = stlmesh.points.shape[0]
		mesh = Mesh(
			numpy_to_typedlist(stlmesh.points.reshape(trinum*3, 3), vec3), 
			typedlist(uvec3(i, i+1, i+2)  for i in range(0, 3*trinum, 3)),
			)
		mesh.options['name'] = stlmesh.name
		return mesh

	def stl_write(mesh, file, **opts):
		stlmesh = stl.mesh.Mesh(np.zeros(len(mesh.faces), dtype=stl.mesh.Mesh.dtype), name=mesh.options.get('name'))
		stlmesh.vectors[:] = typedlist_to_numpy(mesh.points, 'f4')[typedlist_to_numpy(mesh.faces, 'i4')]
		stlmesh.save(file)

'''
	OBJ is loaded using the pywavefront module	https://github.com/pywavefront/PyWavefront
	using the specifications from 	https://en.wikipedia.org/wiki/Wavefront_.obj_file
'''
try:
	import pywavefront
except ImportError:	pass
else:
	
	def obj_read(file, **opts):
		scene = pywavefront.Wavefront(file, parse=True, collect_faces=True)
		points = [vec3(v[:3]) for v in scene.vertices]
		faces = []
		for sub in scene.meshes.values():
			faces.extend(( tuple(f[:3]) for f in sub.faces ))
		mesh = Mesh(points, faces)
		if len(scene.meshes) == 1:
			mesh.options['name'] = next(iter(scene.meshes))
		return mesh
	
	# no write function available at this time
	#def obj_write(mesh, file, **opts):

'''
	JSON is loaded using the builtin json module
	always using the official json specifications
	it can store many object types, not only shapes
'''
import json

class JSONEncoder(json.JSONEncoder):
	def default(self, obj):
		if isinstance(obj, (vec2,vec3,vec4,mat2,mat3,mat4,quat)):
			return {'type':type(obj).__name__, 'content':list(obj)}
		elif isinstance(obj, np.ndarray):
			return {'type':'ndarray', 'dtype':obj.dtype, 'content':list(obj)}
		elif isinstance(obj, Mesh):
			return {'type':'Mesh', 'points': [tuple(p) for p in obj.points], 'faces':obj.faces, 'tracks':obj.tracks, 'groups':obj.groups}
		elif isinstance(obj, Web):
			return {'type':'Web', 'points': [tuple(p) for p in obj.points], 'edges':obj.edges, 'tracks':obj.tracks, 'groups':obj.groups}
		else:
			return json.JSONEncoder.default(self, obj)

def jsondecode(obj):
	if 'type' in obj:
		t = obj['type']
		if t in {'vec2','vec3','vec4','mat2','mat3','mat4','quat'}:		
			return vec3(obj['content'])
		elif t == 'ndarray':
			return np.array(obj['content'], dtype=obj['dtype'])
		elif t == 'Mesh':
			return Mesh([vec3(p) for p in obj['points']], [tuple(f) for f in obj['faces']], obj['tracks'], obj['groups'])
		elif t == 'Web':
			return Mesh([vec3(p) for p in obj['points']], [tuple(f) for f in obj['edges']], obj['tracks'], obj['groups'])
		else:
			raise FileFormatError('unable to load json for dumped type {}', t)
	return obj
	
def json_read(file, **opts):
	return json.load(open(file, 'r'), cls=JSONDecoder, **opts)

def json_write(objs, file, **opts):
	return json.dump(open(file, 'w'), object_hook=jsondecode, **opts)
