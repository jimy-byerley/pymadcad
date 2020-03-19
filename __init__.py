'''     madcad    - it's time to throw parametric softwares out

	main concepts
	-------------
		TODO
	
	data types
	----------
	
'''
version = '0'

from . import mathutils, mesh, generation, cut

from .mathutils import *
from .mesh import Mesh, Wire, Line, MeshError, web
from .cut import bevel, beveltgt, planeoffsets
from .generation import flatsurface, junction, extrans, extrusion, revolution, saddle, tube

