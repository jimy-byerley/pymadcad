.. _mathutils:
.. py:module:: madcad.mathutils

mathutils    - All the basic math types and functions
=====================================================

Most of the names here are coming from the `glm <http://github.com/Zuzu-Typ/PyGLM>`_ module.
But the goal is to harvest at one point all the basic math functions and objects that are used all around madcad.

.. tip::
	All the present functions and types are present in the root madcad module.

Most common **glm** types
~~~~~~~~~~~~~~~~~~~~~~~~~


All following objects implement the common operators :code:`+ - * / <> ==`

.. autoclass:: vec3

.. autoclass:: mat3

.. autoclass:: mat4

.. autoclass:: quat


All `glm` types exists with several element types and in several precision: 

.. list-table:: 
	:header-rows: 1
	
	* - Prefix
	  - Precision
	
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

- Precision specification is put as prefix: :code:`dvec3, fvec3, imat4`. Notation without prefix refers to the madcad implementation precision: **float64** (prefix 'd').
- Object dimension is put as suffix.

In this documentation, when we refer to a 'vector' without explicit type, we obviously mean a :code:`vec3` aka. :code:`dvec3`.

.. note::
	The default glm_ precision type is **float32** (prefix 'f'). For convenience, these are overriden in madcad to use **float64** for a better precision.

Common vector operations
~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: dot

	.. image:: /schemes/mathutils-dot.svg
           :width: 480

.. autofunction:: cross

	.. image:: /schemes/mathutils-cross.svg
           :width: 480

.. autofunction:: length
	
	.. image:: /schemes/mathutils-length.svg
           :width: 480
	
.. autofunction:: distance

	.. image:: /schemes/mathutils-distance.svg
           :width: 480
	
.. function:: normalize(x) -> vecN

	Returns `x` normalized. ie. `x / length(x)`
	
	The new vector has the same direction than `x` but with length `1`. 

.. autofunction:: anglebt

	.. image:: /schemes/mathutils-anglebt.svg
           :width: 480
	
.. autofunction:: arclength


.. autofunction:: project

	.. image:: /schemes/mathutils-project.svg
           :width: 480

.. autofunction:: noproject

	.. image:: /schemes/mathutils-noproject.svg
           :width: 480

.. autofunction:: unproject

	.. image:: /schemes/mathutils-unproject.svg
           :width: 480

.. autofunction:: reflect

	.. image:: /schemes/mathutils-reflect.svg

.. autofunction:: perp

	.. image:: /schemes/mathutils-perp.svg
           :width: 480

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

.. autofunction:: translate

	.. image:: /schemes/mathutils-translate.svg

.. autofunction:: rotate

	.. image:: /schemes/mathutils-rotate.svg

.. autofunction:: scaledir

	.. image:: /schemes/mathutils-scaledir.svg

.. autofunction:: scale

	.. image:: /schemes/mathutils-scale.svg


.. autofunction:: rotatearound

.. autofunction:: dirbase

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
           :width: 960

.. autofunction:: hermite

	.. image:: /schemes/mathutils-hermite.svg
           :width: 960

.. autofunction:: step

	.. image:: /schemes/mathutils-step.svg
           :width: 480
	
.. autofunction:: smoothstep

	.. image:: /schemes/mathutils-smoothstep.svg
           :width: 480
	
.. autofunction:: clamp

	.. image:: /schemes/mathutils-clamp.svg
           :width: 480

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

.. autofunction:: linrange

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
