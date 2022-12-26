Overview
========

Features
--------

- Surface generation (3D sketch primitives, extrusion, revolution, ...)
- Fast boolean operations
- Common mesh file format import/export
- Kinematic manipulation
- Indirect geometry definition through the constraint/solver system
- Blending and envelope completion functions
- Objects display with high-quality graphics

Data types
----------

The most common object types of `madcad` are the following:

Math types: 

	:vec3:    a 3D vector (with fast operations)
	:mat3:    linear transformation, used for rotations and scaling
	:mat4:    affine transformation, used for poses, rotations, translations, scaling
	:quat:    a quaternion, used for rotation (faster and more convenient than matrices for non-repeated operations)
	
Examples: 

.. code-block:: python

    >>> from madcad import *
    >>> print("\n".join(map(str, (O, X, Y, Z)))) # already created
    dvec3(            0,            0,            0 )
    dvec3(            1,            0,            0 )
    dvec3(            0,            1,            0 )
    dvec3(            0,            0,            1 )
    >>> A = vec3(3.54, -2.22, 82.17)
    >>> B = 3.54 * X - 2.22 * Y + 82.17 * Z
    >>> print(A == B)
    True
    >>> normalized_A = normalize(A)
    >>> print(length(normalized_A)) # also see length2
    0.9999999999999999
    >>> rot90aroundA = angleAxis(pi / 2, A)
    >>> print(type(rot90aroundA))
    <class 'glm.dquat'>
    >>> print(type(rotate(pi / 2, A)))
    <class 'glm.dmat4x4'>
    >>> print(type(translate(2 * X))) 
    <class 'glm.dmat4x4'>
    >>> print(rotate(pi / 2, Z) * X) # = Y
    dvec3(  6.12323e-17,            1,            0 )
    >>> print(angleAxis(pi / 2, Z) * X) # = Y
    dvec3(  2.22045e-16,            1,            0 )
    >>> print(translate(Z) * X) # = X + Z
    dvec3(            1,            0,            1 )

More functions and details are in :ref:`module mathutils<mathutils>`.

.. note::
   `translate(vector)` can be replaced by `vector` (i.e. `X + Z == vec3(1., 0., 1.)`). Nevertheless, some functions or methods in `madcad` require a matrix of transformation such as `translate(v)` or `rotate(v)`. See examples in Mesh data section.

Mesh data: 

	:Wire:		used to represent only contiguous 3D lines.
	:Web:		  used to represent 3D lines.
	:Mesh:		used to represent 3D surfaces.


Kinematic data: 

	:Solid:		each instance constitutes a kinematic rigid solid
	:Kinematic:	holds joints and solids for a kinematic structure, with some common mathematical operations
	
Details in :ref:`module kinematic<kinematic>`
