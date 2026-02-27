# This file is part of pymadcad,  distributed under license LGPL v3

'''     pymadcad
		it's time to throw parametric softwares out !
		
Simple yet powerful CAD (Computer Aided Design) library, written with Python.

main concepts
-------------

Current CAD softwares, are based on a full GUI oriented approach, with parametric features bases on dynamic relationship between the scene objects. This can leads to many problems:
	
	- Objects relationships are never visible, thus tricky to understand and to manage. On most of these softwares it's in fact impossible to manage it.
	
	- The software has to be very complex to handle all possible behaviors in parameters change in order to update the model.
	
	- The GUI has to be very complex to allow the user to deal with every aspect of the objects (thus many tools, many object menus, sub-menus, context-dependent behaviors). The softwares are very hard to maintain, and often buggy
	
	- When a CAD file grow in complexity, we must avoid to modify it deeply or the entire build-tree fails and it's no more possible to repair.
	
	- The build tree is rarely human-readable.
	
The current library differs in its approach by the following points:

	- MADCAD is a script based CAD
		all the functions of madcad are available in the current API. Nothing is hidden.
		The python script is the build tree, it is as readable as the user makes it.
		We strive to keep all the functions simple and intuitive to allow a good readability and a fast scripting.

	- MADCAD is not a parametric software
		a madcad script is just a regular Python script. What makes the madcad files parametric is the ability to change variables and reexecute the script.
		
		So madcad is generative, for implicit geometry definitions, the user has to use a solver before constructing the objects.
	
	- MADCAD is extensible
		As it's a regular script, it's possible to use from simple to complex programming concepts (if/else, loops, tables, functions, classes, module imports, generators, ...)


Features
--------

- surface generation (3D sketch primitives, extrusion, revolution, ...)
- fast boolean operations
- common mesh file format import/export
- kinematic manipulation
- implicit geometry definition through the constraint/solver system
- objects display with high-quality graphics

data types
----------

The most common object types of MADCAD are the following:

math types: 
* vec3    a 3D vector (with fast operations)
* mat3    linear transformation, used for rotations and scaling
* mat4    affine transformation, used for poses, rotations, translations, scaling
* quat    a quaternion, used for rotation (faster and more convenient than matrices for non-repeated operations)

mesh data:
* Mesh		used to represent 3D surfaces
* Web		used to represent 3D lines
* Wire		used to represent only contiguous 3D lines

* kinematic data:
* Solid		each instance constitutes a kinematic undeformable solid
* Kinematic	holds joints and solids for a kinematic structure, with some common mathematical operations

most of the remaining classes are definition elements for kinematics or meshes, see `primitives`, `constraints` and `joints` modules.


Examples
--------

Math operations
***************

	>>> a = vec3(1,2,3)
	>>> O = vec3(0)
	>>> X = vec3(1,0,0)
	>>> normalize(a)
	vec3( 0.267261. 0.534522. 0.801784 )
	>>> 5*X
	vec3(5,0,0)
	>>> cross(a, X)		# cross product  a ^ X
	vec3(0,3,2)
	>>> dot(cross(a, X), X)		# X is orthogonal to its cross product with an other vector
	0
	>>> quat(vec3(0,0,2)) * X		# rotation of the X vector by 2 rad around the Z axis
	vec3( -0.416147. 0.909297. 0 )
	
Primitives
**********

	# define points
	O = vec3(0)
	A = vec3(2,0,0)
	B = vec3(1,2,0)
	C = vec3(0,2,0)
	# create a list of primitives
	line = [
		Line(O, A),          # segment from 0 to A (the direction is important for the surface generation)
		ArcThrough(A, B, C), # arc from A to C, with waypoint B
		Line(C,O),           # segment from C to O
		]

	>>> web(line)	# convert the list of primitives into a Web object, ready for extrusion and so on
	Web( ... )

Solver
******

Suppose that you want to set the Arc tangent to the A and B segments, and fix its radius. It is not easy to guess the precise coordinates for A, B and C for this. You can then specify the constraints to the solver. He will fix that for you.

	csts = [
		Tangent(line[0], line[1], A),   # segment and arc are tangent in A
		Tangent(line[1], line[2], C),   # arc and segment are tangent in C
		Radius(line[1], 1.5),           # radius of arc must be equal to 1.5
		]
	solve(csts, fixed=[0])		# solve the constraints, O is fixed and therefore will not move during the process
	
That's it ! The primitive list can now be converted to Wire or Web with the good shape.	

	>>> A, B, C    # points have been modified inplace
	(vec3(...), vec3(...), vec3(...))

'''

import importlib.metadata

# computation
from . import (
		settings,
		# base tools (defines types for the whole library)
		mathutils, mesh, 
		# interdependent functionnalities
		generation, text, boolean, bevel, primitives, constraints, kinematic, joints,
		# near-independant modules
		io, hashing, triangulation,
		# parts
		standard,
	)

# the most common tools, imported to access it directly from madcad
from .mathutils import (
		Axis, COMPREC, NUMPREC, O, Point, Screw, Vector, X, Y, Z, acos,
		affineInverse, angleAxis, anglebt, arclength, asin, atan, atan2,
		bisect, ceil, clamp, comoment, cos, cross, degrees, dirbase, dist,
		distance, distance2, distance_aa, distance_ae, distance_pa,
		distance_pe, distance_pt, dmat2, dmat2x3, dmat3, dmat3x2, dmat4,
		dmat4x3, dot, dquat, dvec2, dvec3, dvec4, e, exp, fbisect, find, floor,
		fmat2, fmat3, fmat4, fquat, fvec1, fvec2, fvec3, fvec4, gcd, glm,
		hypot, i64vec3, imax, inf, interpol1, interpol2, intri_flat,
		intri_parabolic, intri_smooth, intri_sphere, inverse, isclose,
		isfinite, isinf, isnan, ivec2, l1Norm, length, length2, linrange,
		linstep, log, mat2, mat3, mat3_cast, mat4, mat4x4, mix, nan, noproject,
		norm1, norm2, normalize, norminf, perp, perpdot, perspective, pi, pow,
		project, quat, radians, reflect, rotate, rotatearound, scale, scaledir,
		sign, sin, skew, slerp, smoothstep, sqrt, tan, transform, transformer,
		translate, transpose, typedlist, u8vec4, unproject, unskew, uvec2,
		uvec3, vec1, vec2, vec3, vec4,
	)
from .mesh import Mesh, Web, Wire, MeshError, web, wire
from .boolean import pierce, difference, union, intersection
from .bevel import chamfer, filet, edgecut, planeoffsets
from .generation import (
		extrans, extrusion, revolution, helix, screw, saddle, tube, repeat,
		repeataround, flatsurface, icosurface, square, brick, parallelogram,
		cylinder, cone, pyramid, icosahedron, icosphere, uvsphere, regon, 
	)
from .offseting import thicken, inflate, inflate_offsets, expand
from .blending import junction, multijunction, blend, blendloop, blendpair, blenditer
from .primitives import (
		isprimitive, Segment, ArcThrough, ArcCentered, ArcTangent,
		TangentEllipsis, Ellipsis, Circle, Interpolated, Softened
	)
from .constraints import (
	isconstraint, SolveError, Tangent, Distance, Angle, Parallel, Radius,
	PointOn, OnPlane, solve
)
from .kinematic import KinematicError, Joint, Chain, Kinematic
from .assembly import Solid
from .reverse import segmentation
from .selection import select
from .io import read, write, cache, cachefunc
from .triangulation import TriangulationError
from .hull import convexhull, convexoutline, horizon
from .hashing import suites
from .standard import (
		nut, washer, bolt, coilspring_compression, coilspring_tension,
		coilspring_torsion, bearing, slidebearing, section_s, section_w,
		section_c, section_l, section_tslot, screw_slot, bolt_slot,
		bearing_slot_exterior, bearing_slot_interior, circular_screwing,
		grooves_profile, grooves, stfloor, stceil
	)

__all__ = [
		"Angle", "ArcCentered", "ArcTangent", "ArcThrough", "Axis", "COMPREC",
		"Chain", "Circle", "Distance", "Ellipsis", "Interpolated", "Joint",
		"Kinematic", "KinematicError", "Mesh", "MeshError", "NUMPREC", "O",
		"OnPlane", "Parallel", "Point", "PointOn", "Radius", "Screw",
		"Segment", "Softened", "Solid", "SolveError", "Tangent",
		"TangentEllipsis", "TriangulationError", "Vector", "Web", "Wire", "X",
		"Y", "Z", "acos", "affineInverse", "angleAxis", "anglebt", "arclength",
		"asin", "atan", "atan2", "bearing", "bearing_slot_exterior",
		"bearing_slot_interior", "bevel", "bisect", "blend", "blenditer",
		"blendloop", "blendpair", "bolt", "bolt_slot", "boolean", "brick",
		"cache", "cachefunc", "ceil", "chamfer", "circular_screwing", "clamp",
		"coilspring_compression", "coilspring_tension", "coilspring_torsion",
		"comoment", "cone", "constraints", "convexhull", "convexoutline",
		"cos", "cross", "cylinder", "degrees", "difference", "dirbase", "dist",
		"distance", "distance2", "distance_aa", "distance_ae", "distance_pa",
		"distance_pe", "distance_pt", "dmat2", "dmat2x3", "dmat3", "dmat3x2",
		"dmat4", "dmat4x3", "dot", "dquat", "dvec2", "dvec3", "dvec4", "e",
		"edgecut", "exp", "expand", "extrans", "extrusion", "fbisect", "filet",
		"find", "flatsurface", "floor", "fmat2", "fmat3", "fmat4", "fquat",
		"fvec1", "fvec2", "fvec3", "fvec4", "gcd", "generation", "glm",
		"grooves", "grooves_profile", "hashing", "helix", "horizon", "hypot",
		"i64vec3", "icosahedron", "icosphere", "icosurface", "imax", "inf",
		"inflate", "inflate_offsets", "interpol1", "interpol2", "intersection",
		"intri_flat", "intri_parabolic", "intri_smooth", "intri_sphere",
		"inverse", "io", "isclose", "isconstraint", "isfinite", "isinf",
		"isnan", "isprimitive", "ivec2", "joints", "junction", "kinematic",
		"l1Norm", "length", "length2", "linrange", "linstep", "log", "mat2",
		"mat3", "mat3_cast", "mat4", "mat4x4", "mathutils", "mesh", "mix",
		"multijunction", "nan", "noproject", "norm1", "norm2", "normalize",
		"norminf", "nut", "parallelogram", "perp", "perpdot", "perspective",
		"pi", "pierce", "planeoffsets", "pow", "primitives", "project",
		"pyramid", "quat", "radians", "read", "reflect", "regon", "render",
		"repeat", "repeataround", "revolution", "rotate", "rotatearound",
		"saddle", "scale", "scaledir", "screw", "screw_slot", "section_c",
		"section_l", "section_s", "section_tslot", "section_w", "segmentation",
		"select", "settings", "show", "sign", "sin", "skew", "slerp",
		"slidebearing", "smoothstep", "solve", "sqrt", "square", "standard",
		"stceil", "stfloor", "suites", "tan", "text", "thicken", "transform",
		"transformer", "translate", "transpose", "triangulation", "tube",
		"typedlist", "u8vec4", "union", "unproject", "unskew", "uvec2",
		"uvec3", "uvsphere", "vec1", "vec2", "vec3", "vec4", "washer", "web",
		"wire", "write",
	]

# Get version from pyproject.toml
# __version__ = importlib.metadata.version(__package__)

def show(*args, **kwargs):
	''' shorthand to `rendering.show`,
		but this function can be exposed without loading Qt and openGL
	'''
	from .rendering import show
	return show(*args, **kwargs)

def render(*args, **kwargs):
	''' shorthand to `rendering.render`,
		but this function can be exposed without loading Qt and openGL
	'''
	from .rendering import render
	return render(*args, **kwargs)
