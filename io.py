import numpy as np
from .mathutils import vec3
from .mesh import Mesh, Wire
from . import generation

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
		raise FileFormatError('no function available for format '+type)

def write(mesh: Mesh, name: str, type=None, **opts):
	''' write a mesh to a file, guessing its file type '''
	type = filetype(name, type)
	writer = globals().get(type+'_write')
	if writer:
		return writer(mesh, name, **opts)
	else:
		return FileFormatError('no function available for format '+type)

'''
	PLY is loaded using plyfile module 	https://github.com/dranjan/python-plyfile
	using the specifications from 	https://web.archive.org/web/20161221115231/http://www.cs.virginia.edu/~gfx/Courses/2001/Advanced.spring.01/plylib/Ply.txt
		(also locally available in ply-description.txt)
'''
try:
	from plyfile import PlyData, PlyElement
except ImportError:	pass
else:

	def ply_read(file, **opts):
		mesh = Mesh()
		
		data = PlyData.read(file)
		index = {}
		for i,e in enumerate(data.elements):
			index[e.name] = i
		if 'vertex' not in index:	raise FileFormatError('file must have a vertex buffer')
		if 'face' not in index:		raise FileFormatError('file must have a face buffer')
		
		# collect points
		for vertex in data.elements[index['vertex']].data.astype(tuple):
			mesh.points.append(vec3(vertex))
		
		# collect faces
		faces = data.elements[index['face']].data
		if faces.dtype.names[0] == 'vertex_indices':
			for face in faces['vertex_indices']:
				if len(face) == 3:	# triangle
					mesh.faces.append(tuple(face))
				elif len(face) > 3:	# quad or other extended face
					generation.triangulate(mesh, list(face))
		else:
			for face in faces.data:
				mesh.faces.append(tuple(*face[:2]))

		# collect tracks
		if 'group' in faces.dtype.names:
			mesh.tracks = list(faces['group'])
		else:
			mesh.tracks = [0] * len(mesh.faces)
		
		# create groups  (TODO find a way to get it from the file, PLY doesn't support non-scalar types)
		mesh.groups = [None] * (max(mesh.tracks, default=-1)+1)
		
		return mesh

	def ply_write(mesh, file, **opts):
		vertices = np.array(
						[ tuple(p) for p in mesh.points], 
						dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
		faces = np.array(
					[ (f,t)  for f,t in zip(mesh.faces, mesh.tracks)],
					dtype=[('vertex_indices', 'u4', (3,)), ('group', 'u2')])
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

	def stl_read(file, **opts):
		stlmesh = stl.mesh.Mesh.from_file(file, calculate_normals=False)
		mesh = Mesh()
		trinum = stlmesh.points.shape[0]
		for p in stlmesh.points.reshape(trinum*3, 3):
			mesh.points.append(vec3(p))
		mesh.faces = [(i, i+1, i+2)  for i in range(3*trinum, 3)]
		mesh.options['name'] = stlmesh.name
		mesh.mergeclose()
		return mesh

	def stl_write(mesh, file, **opts):
		stlmesh = stl.mesh.Mesh(np.zeros(len(mesh.faces), dtype=stl.mesh.Mesh.dtype), name=mesh.options.get('name'))
		for i, f in enumerate(mesh.faces):
			for j in range(3):
				stlmesh.vectors[i][j] = mesh.points[f[j]]
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
		scene = pywavefront.Wavefront('something.obj', parse=True, cache=False)
		mesh = Mesh()
		mesh.points = [tuple(v[:2]) for v in scene.vertices]
		for sub in scene.meshes.values():
			mesh.faces.extend(( tuple(f[:2]) for f in sub.faces ))
		if len(scene.meshes) == 1:
			mesh.options['name'] = next(iter(scene.meshes))
		return mesh
	
	# no write function available at this time
	#def obj_write(mesh, file, **opts):
