.. _mesh:

mesh    - meshes and discretised objects
========================================

.. automodule:: madcad.mesh


classes
-------


.. autoclass:: madcad.mesh.Mesh
		
	**Point container methods**
	
		Support for ``+, += Mesh``

		.. automethod:: transform
		.. automethod:: flip
		.. automethod:: splitgroups
		.. automethod:: mergepoints
		.. automethod:: mergeclose
		.. automethod:: finish
		.. automethod:: strippoints
		.. automethod:: stripgroups
		
		
	**check methods**
	
		.. automethod:: check
		.. automethod:: isvalid
		.. automethod:: issurface
		.. automethod:: isenvelope
		
	**scan methods**
	
		.. automethod:: pointat
		.. automethod:: usepointat
		.. automethod:: pointnear
		.. automethod:: groupnear
	
	**extraction methods**
	
		.. automethod:: facenormal
		.. automethod:: facenormals
		.. automethod:: edgenormals
		.. automethod:: vertexnormals
		
		.. automethod:: edges
		.. automethod:: edges_oriented
		.. automethod:: group
		.. automethod:: outlines_oriented
		.. automethod:: outlines_unoriented
		.. automethod:: outlines
		.. automethod:: groupoutlines
		.. automethod:: frontiers
		.. automethod:: tangents
		.. automethod:: islands
		.. automethod:: precision
		.. automethod:: box
		.. automethod:: surface
		.. automethod:: barycenter
	


		
.. autoclass:: madcad.mesh.Web
		
	
	**Point container methods**
	
		Support for ``+, += Web``

		.. automethod:: transform
		.. automethod:: flip
		.. automethod:: segmented
		.. automethod:: mergepoints
		.. automethod:: mergeclose
		.. automethod:: finish
		.. automethod:: strippoints
		.. automethod:: stripgroups
		
		
	**check methods**
	
		.. automethod:: check
		.. automethod:: isvalid
		.. automethod:: isline
		.. automethod:: isloop
		
	**scan methods**
	
		.. automethod:: pointat
		.. automethod:: usepointat
		.. automethod:: pointnear
	
	**extraction methods**
	
		.. automethod:: extremities
		.. automethod:: groupextremities
		.. automethod:: frontiers
		.. automethod:: group
		.. automethod:: arcs
		.. automethod:: box
		.. automethod:: precision
		.. automethod:: length
		.. automethod:: barycenter
	



.. autoclass:: madcad.mesh.Wire

	Support for item access to underlying points: 
	
	.. automethod:: __getitem__
	
	**Point container methods**
	
		Support for ``+, += Wire``
		
		.. automethod:: transform
		.. automethod:: flip
		.. automethod:: segmented
		.. automethod:: close
		.. automethod:: mergeclose
		.. automethod:: strippoints
		
	**check methods**
	
		.. automethod:: check
		.. automethod:: isvalid
	
	**extraction methods**
	
		.. automethod:: edges
		.. automethod:: edge
		.. automethod:: vertexnormals
		.. automethod:: tangents
		.. automethod:: normal
		.. automethod:: length
		.. automethod:: barycenter


conversion functions
--------------------

.. autofunction:: madcad.mesh.web
.. autofunction:: madcad.mesh.wire

mesh internals manipulation
------------------------------

.. autofunction:: edgekey
.. autofunction:: facekeyo
.. autofunction:: connpp
.. autofunction:: connef
.. autofunction:: lineedges
.. autofunction:: striplist
.. autofunction:: line_simplification
.. autofunction:: suites
.. autofunction:: mesh_distance


Exceptions defined here
-----------------------

.. autoexception:: MeshError
