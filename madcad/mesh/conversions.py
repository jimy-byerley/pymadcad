from ..mathutils import typedlist, vec3
from .container import NMesh, suites
from .mesh import Mesh
from .web import Web
from .wire import Wire


def mesh(*arg) -> Mesh:
	''' Build a Mesh object from supported objects:
	
		:mesh:              return it with no copy
		:primitive:         call its ``.mesh`` method and convert the result to web
		:iterable:          convert each element to web and join them
	'''
	if not arg:	
		raise TypeError('mesh takes at least one argument')
	if len(arg) == 1:	
		arg = arg[0]
	if isinstance(arg, Mesh):		
		return arg
	elif hasattr(arg, 'mesh'):
		return mesh(arg.mesh())
	elif hasattr(arg, '__iter__'):
		pool = Mesh()
		for primitive in arg:
			pool += mesh(primitive)
		pool.mergeclose()
		return pool
	else:
		raise TypeError('incompatible data type for Web creation')

def web(*arg) -> Web:
	''' Build a Web object from supported objects:
	
		:web:               return it with no copy
		:wire:              reference points and generate edge couples
		:primitive:         call its ``.mesh`` method and convert the result to web
		:iterable:          convert each element to web and join them
		:list of vec3:      reference it and generate trivial indices
		:iterable of vec3:  get points and generate trivial indices
	'''
	if not arg:	
		raise TypeError('web takes at least one argument')
	if len(arg) == 1:	
		arg = arg[0]
	if isinstance(arg, Web):		
		return arg
	elif isinstance(arg, Wire):	
		return Web(
				arg.points, 
				arg.edges(), 
				arg.tracks[:-1] if arg.tracks else None, 
				groups=arg.groups,
				)
	elif hasattr(arg, 'mesh'):
		return web(arg.mesh())
	elif isinstance(arg, (typedlist,list,tuple)) and isinstance(arg[0], vec3):
		return Web(arg, [(i,i+1) for i in range(len(arg)-1)])
	elif hasattr(arg, '__iter__'):
		pool = Web()
		for primitive in arg:
			pool += web(primitive)
		pool.mergeclose()
		return pool
	else:
		raise TypeError('incompatible data type for Web creation')

def wire(*arg) -> Wire:
	''' Build a Wire object from the other compatible types.
		Supported types are:
		
		:wire:              return it with no copy
		:web:               find the edges to joint, keep the same point buffer
		:primitive:         call its ``.mesh`` method and convert the result to wire
		:iterable:          convert each element to Wire and joint them
		:list of vec3:      reference it and put trivial indices
		:iterable of vec3:  create internal point list from it, and put trivial indices
	'''
	if not arg:	
		raise TypeError('web take at least one argument')
	if len(arg) == 1:	
		arg = arg[0]
	if isinstance(arg, Wire):		
		return arg
	elif isinstance(arg, Web):
		indices = suites(arg.edges)
		if len(indices) > 1:	raise ValueError('the given web has junctions or is discontinuous')
		return Wire(arg.points, indices[0], groups=[None])	# TODO: find a way to get the groups from the Web edges through suites or not
	elif hasattr(arg, 'mesh'):
		return wire(arg.mesh())
	elif isinstance(arg, (typedlist,list,tuple)) and isinstance(arg[0], vec3):
		return Wire(arg)
	elif hasattr(arg, '__iter__'):
		pool = Wire()
		for primitive in arg:
			pool += wire(primitive)
		pool.mergeclose()
		return pool
	else:
		raise TypeError('incompatible data type for Wire creation')

