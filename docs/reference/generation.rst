.. _generation:

generation    - functions to generate mesh surfaces from lines or meshes
========================================================================

.. automodule:: madcad.generation

based on extrusion/transformation of a Web
------------------------------------------

.. autofunction:: extrans
.. autofunction:: extrusion
.. autofunction:: revolution
.. autofunction:: saddle
.. autofunction:: tube

junction functions
------------------

The very simple direct junction of two lines

.. autofunction:: join

Soft junction surface between 2 lines

.. autofunction:: junction

.. autofunction:: junctioniter

	.. tip:: this is a more flexible but more complex version of junction

The two previous requires a matching between the lines, made with one of the following function

.. autofunction:: curvematch
.. autofunction:: matchexisting
.. autofunction:: matchclosest
.. autofunction:: dividematch

generation of common meshes
---------------------------

.. autofunction:: brick
.. autofunction:: icosahedron
.. autofunction:: icosphere
.. autofunction:: uvsphere

Unsorted
--------

.. autofunction:: thicken
.. autofunction:: flatsurface
.. autofunction:: icosurface
.. autofunction:: subdivide
.. autofunction:: interpol2tri
