'''     madcad    - it's time to throw parametric softwares out

	main concepts
	-------------
		TODO
	
	data types
	----------
		TODO
'''
version = '0'

# computation
from . import (
		# base tools (defines types for the whole library)
		mathutils, mesh, 
		# interdependent functionnalities
		generation, cut, primitives, constraints, kinematic, 
		# near-independant modules
		io, hashing,
	)
# gui
from . import view, text

# the most common tools, imported to access it directly from madcad
from .mathutils import *
from .mesh import Mesh, Web, Wire, MeshError, web
from .cut import chamfer, bevel, beveltgt, planeoffsets
from .generation import flatsurface, junction, extrans, extrusion, revolution, saddle, tube
from .primitives import Axis, Segment, ArcThrough, ArcCentered, Circle
from .constraints import SolveError, Tangent, Distance, Angle, Parallel, Radius, Projected, PointOn, solve
from .kinematic import Solid, InSolid, Pivot, Plane
from .selection import select
from .io import read, write

from .text import Text
from .view import Scene

