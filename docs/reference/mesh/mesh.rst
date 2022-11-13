Mesh
====

.. image:: /screenshots/mesh-mesh.png

.. autoclass:: madcad.Mesh

	.. collapse:: special methods
	
		.. automethod:: __add__
		.. automethod:: __iadd__

	.. collapse:: data management
	
		.. automethod:: own
		.. automethod:: option
		.. automethod:: transform
		.. automethod:: mergeclose
		.. automethod:: mergepoints
		.. automethod:: mergegroups
		.. automethod:: strippoints
		.. automethod:: stripgroups
		.. automethod:: finish
	
	.. collapse:: verification methods
	
	
		.. automethod:: check
		.. automethod:: isvalid
		.. automethod:: issurface
		.. automethod:: isenvelope
		
	.. collapse:: selection methods
	
		.. automethod:: pointnear
		.. automethod:: pointat
		.. automethod:: groupnear
		.. automethod:: facenear
		.. automethod:: group
		.. automethod:: replace
		.. automethod:: qualify
		.. automethod:: qualified_indices
		.. automethod:: qualified_groups

	.. collapse:: extraction methods
	
		.. automethod:: maxnum
		.. automethod:: precision
		.. automethod:: surface
		.. automethod:: volume
		.. automethod:: barycenter
		.. automethod:: barycenter_points
		.. automethod:: box
		
		.. automethod:: usepointat
		.. automethod:: facepoints
		.. automethod:: facenormal
		.. automethod:: facenormals
		.. automethod:: edgenormals
		.. automethod:: vertexnormals
		.. automethod:: tangents
		
		.. automethod:: edges
		.. automethod:: edges_oriented
		.. automethod:: outlines
		.. automethod:: outlines_oriented
		.. automethod:: outlines_unoriented
		.. automethod:: groupoutlines
		.. automethod:: frontiers
		
		.. automethod:: splitgroups
		.. automethod:: split
		.. automethod:: islands
		
		.. automethod:: flip
		.. automethod:: orient
