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

	mathutils.rst
	mesh.rst
	
	kinematic.rst
	joints.rst
	
	primitives.rst
	constraints.rst
	
	generation.rst
	blending.rst
	cut.rst
	boolean.rst
	
	io.rst
	settings.rst
	hashing.rst
	rendering.rst
	scheme.rst

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
	.. autofunction:: tube
	
blending functions
~~~~~~~~~~~~~~~~~~
	.. autofunction:: junction
	.. autofunction:: blendpair
	.. autofunction:: blendloop

cut functions
~~~~~~~~~~~~~

	.. autofunction:: chamfer
	.. autofunction:: bevel
