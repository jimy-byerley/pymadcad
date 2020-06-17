'''     pymadcad, core library of the madcad software    it's time to throw parametric softwares out

	main concepts
	-------------
		- built on the idea that current parametric softwares, such as Catia or OpenCascade, are leading to very messy and tricky softwares (just look at their GUI!)
		- procedural engineering design library
		- full python code for great clearity of the algorithms, polyvalence, extensibility, and intuitive use
	
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
		generation, boolean, cut, primitives, constraints, kinematic, 
		# near-independant modules
		io, hashing, triangulation,
	)
# gui
from . import view, text, displays

# the most common tools, imported to access it directly from madcad
from .mathutils import *
from .mesh import Mesh, Web, Wire, MeshError, web, wire, suites
from .boolean import difference, union, intersection
from .cut import chamfer, bevel, beveltgt, planeoffsets
from .generation import flatsurface, junction, extrans, extrusion, revolution, saddle, tube, icosurface
from .primitives import isprimitive, Point, Axis, Segment, ArcThrough, ArcCentered, Circle
from .constraints import isconstraint, SolveError, Tangent, Distance, Angle, Parallel, Radius, Projected, PointOn, OnPlane, solve
from .kinematic import Solid, InSolid, Pivot, Plane, Kinemanip
from .selection import select
from .io import read, write, cache
from .triangulation import TriangulationError

from .text import Text
from .view import Scene, isdisplay, displayable

