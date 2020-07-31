API summary
============

Here is the most used functions and classes of madcad.


Features
--------

- surface generation (3D sketch primitives, extrusion, revolution, ...)
- fast boolean operations
- common mesh file format import/export
- kinematic manipulation
- indirect geometry definition through the constraint/solver system
- objects display with high-quality graphics

data types
----------

The most common object types of MADCAD are the following:

math types: 

	(:ref:`mathutils<mathutils>`)

	:vec3:    a 3D vector (with fast operations)
	:mat3:    linear transformation, used for rotations and scaling
	:mat4:    affine transformation, used for poses, rotations, translations, scaling
	:quat:    a quaternion, used for rotation (faster and more convenient than matrices for non-repeated operations)


mesh data: 

	(:ref:`mesh module<mesh>`)

	:Mesh:		used to represent 3D surfaces.
	:Web:		used to represent 3D lines.
	:Wire:		used to represent only contiguous 3D lines.


kinematic data: 

	(:ref:`kinematic module<kinematic>`)

	:Solid:		each instance constitutes a kinematic undeformable solid
	:Kinematic:	holds joints and solids for a kinematic structure, with some common mathematical operations

most of the remaining classes are definition elements for kinematics or meshes, see :ref:`primitives<primitives>` , :ref:`constraints<constraints>` , and :ref:`joints<joints>` modules.

Examples
--------

Math operations
***************

.. code-block:: python
	
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

.. code-block:: python
	
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

.. code-block:: python

	>>> web(line)	# convert the list of primitives into a Web object, ready for extrusion and so on
	Web( ... )

Solver
******

Suppose that you want to set the Arc tangent to the A and B segments, and fix its radius. It is not easy to guess the precise coordinates for A, B and C for this. You can then specify the constraints to the solver. He will fix that for you.

.. code-block:: python

	csts = [
		Tangent(line[0], line[1], A),   # segment and arc are tangent in A
		Tangent(line[1], line[2], C),   # arc and segment are tangent in C
		Radius(line[1], 1.5),           # radius of arc must be equal to 1.5
		]
	solve(csts, fixed=[0])		# solve the constraints, O is fixed and therefore will not move during the process
	
That's it ! The primitive list can now be converted to Wire or Web with the good shape.	

.. code-block:: python

	>>> A, B, C    # points have been modified inplace
	(vec3(...), vec3(...), vec3(...))

Kinematic
*********

TODO

Generation
**********

TODO
