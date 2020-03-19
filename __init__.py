'''     pymadcad    - it's time to throw parametric softwares out

	main concepts
	-------------
		TODO
	
	data types
	----------
	
'''


from . import mathutils, mesh, generation, cut

from .mathutils import *
from .mesh import Mesh, Wire, Line, MeshError, web
from .cut import bevel, beveltgt
from .generation import flatsurface, junction, extrans, extrusion, revolution, saddle, tube
