API summary
============

Here are the most used functions and classes of madcad.


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
		Segment(O, A),          # segment from 0 to A (the direction is important for the surface generation)
		ArcThrough(A, B, C), # arc from A to C, with waypoint B
		Segment(C,O),           # segment from C to O
		]

.. code-block:: python

	>>> web(line)	# convert the list of primitives into a Web object, ready for extrusion and so on
	Web( ... )
	
.. image:: /screenshots/primitives-unsolved.png

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
	
.. image:: /screenshots/primitives-solved.png

Kinematic
*********

Prior part design (or after for assembly), we may want to see how what we are making should behave. We use then a `Kinematic`, using the current engineering conventions. In the same spirit as for the primitives, the `solvekin` function solves the *joints* constraints.

.. code-block:: python

	# we define the solids, they intrinsically have nothing particular
	base = Solid()
	s1 = Solid()
	s2 = Solid()
	s3 = Solid()
	s4 = Solid()
	s5 = Solid()
	wrist = Solid(name='wrist')	# give it a fancy name
	
	# the joints defines the kinematic.
	# this is a 6 DoF (degrees of freedom) robot arm
	csts = [
		Pivot(base,s1, (O,Z)),                   # pivot using axis (O,Z) both in solid base and solid 1
		Pivot(s1,s2, (vec3(0,0,1), X), (O,X)),   # pivot using different axis coordinates in each solid
		Pivot(s2,s3, (vec3(0,0,2), X), (O,X)),
		Pivot(s3,s4, (vec3(0,0,1), Z), (vec3(0,0,-1), Z)),
		Pivot(s4,s5, (O,X)),
		Pivot(s5,wrist, (vec3(0,0,0.5), Z), (O,Z)),
		]
	
	# the kinematic is created with some fixed solids (they interact but they don't move)
	kin = Kinematic(csts, fixed=[base])
	
Kinematics are displayable as interactive objects the user can move. Thay also are usefull to compute force repartitions during the movmeents or movement trajectories or kinematic cycles ...
	
.. image:: /screenshots/kinematic-robot-arm.png

Generation
**********

Most of the common surfaces are generated from an outline (closed is often not mendatory). An outline can be a `Web` or a `Wire`, depending on the algorithm behind. Those can be created by hand or obtained from primitives (see above).

Generaly speaking, generation functions are all functions that can produce a mesh from simple parameters by knowing by advance where each point will be.

.. note::
	Most generation functions produce a surface. To represent a volume we use a closed surface so you have to pay attention to if your input ooutline is well closed too.


Suppose we want a torus, let's make a simple revolution around an axis, the extruded outline have not even to be in a plane:

.. code-block:: python

	revolution(
	    radians(180),       # 180 degrees converted into radiaus 
	    (O,Z),              # revolution axis
	    web(Circle((A,Y), 0.5)),
	    )

.. image:: /screenshots/revolution-circle.png

Joint two arbitrary lines by a smooth surface, to close a complex surface for instance:
	
.. code-block:: python

	junction(
		# find the matching points in the two lines.
		# you can stack matching function to achieve exactly what you want.
		matchclosest(
			Wire([vec3(-2,0,0), vec3(-1,0,0), vec3(0,0,0), vec3(1,0,0), vec3(2,0,0)]),
			Wire([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)]),
		))

.. image:: /screenshots/junction-closest.png


Reworking
*********

For some geometries it is much faster to rework the already generated mesh to add complex geometries. Putting a hole in a surface for instance. Thus you won't need to generate all the intersection surfaces by hand.

.. code-block:: python

	# obtain two different shapes that has noting to to with each other
	m1 = brick(Box(width=vec3(2)))
	m2 = m1.transform(vec3(0.5, 0.3, 0.4)) .transform(quat(0.7*vec3(1,1,0)))
	
	# remove the volue of the second to the first
	difference(m1, m2)
	
.. image:: /screenshots/boolean-cube.png

