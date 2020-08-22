.. _mathutils:
.. py:module:: madcad.mathutils

mathutils    - all the basic math types and functions
=====================================================

Most of the names here are coming from the `glm <http://github.com/Zuzu-Typ/PyGLM>`_ module.
But the goal is to harvest at one point, all the basic math functions and objects that are used all around madcad.

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

- **'d'**  for double precisin floating point (float64)
- **'f'** for simple precision floating point (float32)
- **'i'** for integer (int32)
- **'i16'** for int16
- **'i8'** for byte (int8)
- **'u'** for unsigned integer (uint32), 'u16' and 'u8' also exists
- **'b'** for bit

precision specification is put as prefix: :code:`dvec3, fvec3, imat4`. Notation without prefix refers to the madcad implementation precision: **float64** (prefix 'd').

In this documentation, when we refer to a 'vector' without explicit type, we obviously mean a :code:`vec3` aka. :code:`dvec3`.

.. note::
	The default glm_ precision type is **float32** (prefix 'f'). For convenience, these are overriden in madcad to use a better precision.

Common vector operations
~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: dot

.. autofunction:: cross

.. autofunction:: mix

.. autofunction:: length
.. autofunction:: normalize

.. autofunction:: anglebt

.. autofunction:: project
.. autofunction:: noproject
.. autofunction:: unproject

.. autofunction:: perp

.. autofunction:: norm1

.. function:: norm2(x)
	
	norm L2 (euclidian)  ie.  strictly the same as length(x)

.. autofunction:: norminf

Transformations
~~~~~~~~~~~~~~~

.. autofunction:: transform

.. autofunction:: dirbase

.. autofunction:: scaledir

.. autofunction:: inverse

.. autofunction:: affineInverse

.. autofunction:: angle

.. autofunction:: axis

.. autofunction:: angleAxis

.. autofunction:: lerp


Scalar functions
~~~~~~~~~~~~~~~~

.. autofunction:: interpol1

.. autofunction:: interpol2

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
