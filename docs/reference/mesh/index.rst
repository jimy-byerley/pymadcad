mesh    - meshes and discretised objects
========================================

.. automodule:: madcad.mesh

.. toctree::
	:maxdepth: 0

	mesh.rst
	web.rst
	wire.rst

.. autoclass:: madcad.MeshError

Conversions
-----------

.. autofunction:: madcad.mesh.conversions.mesh
.. autofunction:: madcad.mesh.conversions.web
.. autofunction:: madcad.mesh.conversions.wire
.. autofunction:: madcad.mesh.typedlist_to_numpy
.. autofunction:: madcad.mesh.numpy_to_typedlist
.. autofunction:: madcad.mesh.ensure_typedlist

Connectivity
------------

.. autofunction:: madcad.mesh.edgekey
.. autofunction:: madcad.mesh.facekeyo
.. autofunction:: madcad.mesh.arrangeface
.. autofunction:: madcad.mesh.arrangeedge
.. autofunction:: madcad.mesh.connpp
.. autofunction:: madcad.mesh.connef
.. autofunction:: madcad.mesh.connpe
.. autofunction:: madcad.mesh.connexity

.. autofunction:: madcad.mesh.suites

Misc
----

.. autofunction:: madcad.mesh.line_simplification
.. autofunction:: madcad.mesh.mesh_distance
.. autofunction:: madcad.mesh.distance2_pm
