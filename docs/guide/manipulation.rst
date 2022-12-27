Manipulation
============

Constraints solver
------------------

Suppose that you want to set the Arc tangent to the A and B segments, and fix its radius. It is not easy to guess the precise coordinates for A, B and C for this. You can then specify the constraints to the solver. He will fix that for you.

.. code-block:: python
   
    from madcad import *
    # Define points
    O = vec3(0)
    A = vec3(2, 0, 0)
    B = vec3(1, 2, 0)
    C = vec3(0, 2, 0)
    # Create a list of primitives
    # /!\ The direction is important for the surface generation
    lines = [
        Segment(O, A), # Segment from O to A 
        ArcThrough(A, B, C),  # Arc from A to C, with waypoint B 
        Segment(C, O),  # Segment from C to O
    ]
    show([lines])

.. image:: /screenshots/manipulation/constraints-before.png

That’s it ! The primitive list can now be converted to Wire or Web with the good shape.

.. code-block:: python

    csts = [
        Tangent(lines[0], lines[1], A),  # Segment and arc are tangent in A
        Tangent(lines[1], lines[2], C),  # Arc and segment are tangent in C
        Radius(lines[1], 1.5),  # Radius of arc must be equal to 1.5
    ]

    # Solve the constraints, 
    # O is fixed and therefore will not move during the process
    solve(csts, fixed=[0])
    print("\n".join("{} = {}".format(name, v) for name, v in zip("ABC", (A, B, C))))
    # A = dvec3(      1.83758,    -0.092837,  -5.8732e-09 )
    # B = dvec3(      1.21717,      2.83567,  3.43597e-10 )
    # C = dvec3(     0.145109,      1.64325, -2.07437e-09 )
    show([lines])

.. image:: /screenshots/manipulation/constraints-after.png

Boolean operations - Intersection
---------------------------------

For some geometries it is much faster to rework the already generated mesh to add complex geometries. Putting a hole in a surface for instance. Thus you won’t need to generate all the intersection surfaces by hand.

.. code-block:: python

    from madcad import *
    # Obtain two different shapes that has noting to to with each other
    m1 = brick(width=vec3(2))
    m2 = m1.transform(vec3(0.5, 0.3, 0.4)).transform(quat(0.7 * vec3(1, 1, 0)))

    # Remove the volume of the second to the first
    diff = difference(m1, m2)
    show([diff])

.. image:: /screenshots/manipulation/boolean-op.png

All boolean operation are documented :doc:`here</reference/boolean>`.

Junction
--------

Join arbitrary outlines in nicely blended surfaces.

.. code-block:: python

    from madcad import *
    interfaces = [
        Circle((vec3(0, 0, 3), vec3(0, 0, 1)), 1),
        Circle((vec3(-1, -1, -1), normalize(vec3(-1, -1, -1))), 1),
        Circle((vec3(1, -1, -1), normalize(vec3(1, -1, -1))), 1),
    ]

    m = junction(
        interfaces[0],
        interfaces[1],
        interfaces[2],
        tangents="tangent",
    )
    for c in interfaces:
        m += extrusion(c.axis[1] * 3, web(c))

    show([m])

.. image:: /screenshots/manipulation/junction.png

More information are available :doc:`here</reference/blending>`.

Bevel
-----

An other usual rework operation is cut edges with chamfers or roundings. Because `round` is already a math function, we use the term bevel

.. code-block:: python

    from madcad import *
    # Obtain a mesh
    cube = brick(width=vec3(2))

    # Cut some edges
    # No need to do cube = bevel(...)
    bevel(
       cube,
       [(0, 1), (1, 2), (2, 3), (0, 3), (1, 5), (0, 4)],  # Edges to smooth
       ("width", 0.3),  # Cutting description, known as 'cutter'
    )
    show([cube])

.. image:: /screenshots/manipulation/bevel-cube.png

.. tip::

   `chamfer` and `bevel` work in the same way.
