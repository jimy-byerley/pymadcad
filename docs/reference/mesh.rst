.. _mesh:

mesh    - meshes and discretised objects
========================================

.. automodule:: madcad.mesh


classes
-------


.. autoclass:: madcad.mesh.Mesh
		
	**Point container methods**
	
		Support for ``+, += Mesh``

		.. automethod:: box
		.. automethod:: transform
		.. automethod:: mergepoints
		.. automethod:: mergeclose
		.. automethod:: finish
		.. automethod:: precision
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
		.. automethod:: group
		.. automethod:: outlines_oriented
		.. automethod:: outlines_unoriented
		.. automethod:: outlines
		.. automethod:: groupoutlines
		.. automethod:: groupoutlines_oriented
		.. automethod:: surface
	
	**other methods**
	
		.. automethod:: splitgroups
		.. automethod:: flip


		
.. autoclass:: madcad.mesh.Web
		
	
	**Point container methods**
	
		Support for ``+, += Mesh``

		.. automethod:: box
		.. automethod:: transform
		.. automethod:: mergepoints
		.. automethod:: mergeclose
		.. automethod:: finish
		.. automethod:: precision
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
		.. automethod:: length
	
	**other methods**
	
		.. automethod:: flip



.. autoclass:: madcad.mesh.Wire
		
	.. automethod:: length
	.. automethod:: edges
	.. automethod:: edge
	
	.. automethod:: isvalid
	.. automethod:: check


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


Exceptions defined here
-----------------------

.. autoexception:: MeshError
