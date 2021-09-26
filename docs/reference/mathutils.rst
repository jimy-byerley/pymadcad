.. _mathutils:
.. py:module:: madcad.mathutils

mathutils    - all the basic math types and functions
=====================================================

Most of the names here are coming from the `glm <http://github.com/Zuzu-Typ/PyGLM>`_ module.
But the goal is to harvest at one point all the basic math functions and objects that are used all around madcad.

.. tip::
	All the present functions and types are present in the root madcad module.

Most common **glm** types
~~~~~~~~~~~~~~~~~~~~~~~~~


all implements the common operators :code:`+ - * / <> ==`

.. autoclass:: vec3

.. autoclass:: mat3

.. autoclass:: mat4

.. autoclass:: quat


all glm types exists with several element types and in several precision: 

.. list-table:: 
	:header-rows: 1
	
	* - prefix
	  - precision
	
	* - **d**
	  - f64 aka double precisin floating point
	* - **f**
	  - f32 aka for simple precision floating point
	* - **i**
	  - i32 aka integer
	* - **i16** 
	  - 16 bits integer
	* - **i8**
	  - 8 bits integer aka byte
	* - **u**
	  - unsigned integer (also declines in u16 and u32)
	* - **b**
	  - bit aka boolean

- precision specification is put as prefix: :code:`dvec3, fvec3, imat4`. Notation without prefix refers to the madcad implementation precision: **float64** (prefix 'd').
- object dimension is put as suffix.

In this documentation, when we refer to a 'vector' without explicit type, we obviously mean a :code:`vec3` aka. :code:`dvec3`.

.. note::
	The default glm_ precision type is **float32** (prefix 'f'). For convenience, these are overriden in madcad to use **float64** for a better precision.

Common vector operations
~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: dot

	.. image:: /schemes/mathutils-dot.svg

.. autofunction:: cross

	.. image:: /schemes/mathutils-cross.svg

.. autofunction:: length
	
	.. image:: /schemes/mathutils-length.svg
	
.. autofunction:: distance

	.. image:: /schemes/mathutils-distance.svg
	
.. function:: normalize(x) -> vecN

	Returns `x` normalized. ie. `x / length(x)`
	
	The new vector has the same direction than `x` but with length `1`. 

.. autofunction:: anglebt

	.. image:: /schemes/mathutils-anglebt.svg
	
.. autofunction:: arclength


.. autofunction:: project

	.. image:: /schemes/mathutils-project.svg

.. autofunction:: noproject

	.. image:: /schemes/mathutils-noproject.svg

.. autofunction:: unproject

	.. image:: /schemes/mathutils-unproject.svg


.. autofunction:: perp

	.. image:: /schemes/mathutils-perp.svg

.. function:: norm1(x) -> float

	norm L1  ie.  `abs(x) + abs(y) + abs(z)`
	
	Alias to `glm.l1Norm`

.. function:: norm2(x) -> float
	
	norm L2  ie.  `sqrt(x**2 + y**2 + z**2)`   the usual distance also known as manhattan distance
	
	Alias to `glm.l2Norm` alias `glm.length`

.. function:: norminf(x) -> float

	norm L infinite  ie.  `max(abs(x), abs(y), abs(z))`
	
	Alias to `glm.lxNorm`

See `the glm complete reference <https://github.com/Zuzu-Typ/PyGLM/blob/master/wiki/function-reference/README.md>`_


Transformations
~~~~~~~~~~~~~~~

.. autofunction:: transform

.. autofunction:: dirbase

.. autofunction:: scaledir

.. autofunction:: rotatearound

.. autofunction:: transpose

.. autofunction:: inverse

.. autofunction:: affineInverse

.. autofunction:: angle

.. autofunction:: axis

.. autofunction:: angleAxis

.. autofunction:: lerp

.. autofunction:: slerp


Scalar functions
~~~~~~~~~~~~~~~~

.. autofunction:: mix

	.. image:: /schemes/mathutils-mix.svg

.. autofunction:: hermite

	.. image:: /schemes/mathutils-hermite.svg

.. autofunction:: step

	.. image:: /schemes/mathutils-step.svg
	
.. autofunction:: smoothstep

	.. image:: /schemes/mathutils-smoothstep.svg
	
.. autofunction:: clamp

	.. image:: /schemes/mathutils-clamp.svg

.. autofunction:: intri_smooth
.. autofunction:: intri_sphere
.. autofunction:: intri_parabolic

Distances
~~~~~~~~~

.. autofunction:: distance_pa
.. autofunction:: distance_pe
.. autofunction:: distance_aa
.. autofunction:: distance_ae

Constants
~~~~~~~~~

.. py:data:: NUMPREC

	Numeric precision of a unit float (using the default precision)
	
.. py:data:: COMPREC

	unit complement of NUMPREC for convenience: :code:`1 - NUMPREC`
	
	
Localy defined data types
~~~~~~~~~~~~~~~~~~~~~~~~~

.. autoclass:: Box

	.. autoproperty:: center
	.. autoproperty:: width
	.. automethod:: corners
	.. automethod:: volume
	
	.. automethod:: isvalid
	.. automethod:: isempty
	
	.. automethod:: contain
	.. automethod:: inside
	
	.. automethod:: intersection
	.. automethod:: union
	
	.. automethod:: intersection_update
	.. automethod:: union_update
	
	.. automethod:: transform
	
