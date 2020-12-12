Reference
=========

.. warning::
	The module is still in alpha version and the API may change a lot before release.

submodules
~~~~~~~~~~

.. tip::
	Most of the submodule functions and classes are present in the madcad module, so unless you write a library you won't need to import it explicitly.

.. toctree::
	:maxdepth: 1

	mesh.rst
	kinematic.rst
	joints.rst
	primitives.rst
	constraints.rst
	io.rst
	mathutils.rst
	
	generation.rst
	blending.rst
	cut.rst
	hashing.rst
	settings.rst
	rendering.rst

.. py:module:: madcad


solvers
~~~~~~~

	.. autofunction:: solve
	.. autofunction:: solvekin

volume boolean operators
~~~~~~~~~~~~~~~~~~~~~~~~

	.. autofunction:: difference
	.. autofunction:: union
	.. autofunction:: intersection

generation functions
~~~~~~~~~~~~~~~~~~~~

	.. autofunction:: extrusion
	.. autofunction:: revolution
	.. autofunction:: extrans
	.. autofunction:: junction

cut functions
~~~~~~~~~~~~~

	.. autofunction:: chamfer
	.. autofunction:: bevel
