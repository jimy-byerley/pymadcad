'''     madcad    - it's time to throw parametric softwares out

	main concepts
	-------------
		TODO
	
	data types
	----------
		TODO
'''
version = '0'

from . import mathutils, mesh, generation, cut, primitives, constraints, kinematic, io
from . import view, text

from .mathutils import *
from .mesh import Mesh, Wire, Line, MeshError, web
from .cut import chamfer, bevel, beveltgt, planeoffsets
from .generation import flatsurface, junction, extrans, extrusion, revolution, saddle, tube
from .primitives import Axis, Segment, Arc, Circle, TangentEllipsis
from .constraints import SolveError, Tangent, Distance, Angle, Parallel, Radius, Projected, PointOn, solve
from .kinematic import Solid, InSolid, Pivot, Plane
from .selection import select
from .io import read, write

from .text import Text
from .view import Scene

