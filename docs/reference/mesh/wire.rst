Wire
====

.. autoclass:: madcad.Wire

	.. collapse:: special methods
	
		.. automethod:: __add__
		.. automethod:: __iadd__
		
		.. automethod:: __len__
		.. automethod:: __iter__
		.. automethod:: __getitem__
	
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
		
	.. collapse:: mesh checks
	
		.. automethod:: check
		.. automethod:: isvalid
		
	.. collapse:: selection methods
	
		.. automethod:: pointnear
		.. automethod:: pointat
		.. automethod:: groupnear
		.. automethod:: edgenear
		
		.. automethod:: group
		
	.. collapse:: extraction methods
	
		.. automethod:: length
		.. automethod:: surface
		.. automethod:: barycenter
		.. automethod:: barycenter_points
		.. automethod:: box
		.. automethod:: normal
	
		.. automethod:: edgepoints
		.. automethod:: edgedirection
		.. automethod:: edge
		.. automethod:: edges
		.. automethod:: vertexnormals
		.. automethod:: tangents
		
		.. automethod:: flip
		.. automethod:: close
		.. automethod:: segmented
