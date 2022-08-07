Web
===

.. autoclass:: madcad.Web

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
		
	.. collapse:: mesh checks
	
		.. automethod:: check
		.. automethod:: isvalid
		.. automethod:: isline
		.. automethod:: isloop
		
	.. collapse:: selection methods
	
		.. automethod:: pointnear
		.. automethod:: pointat
		.. automethod:: groupnear
		.. automethod:: edgenear
		.. automethod:: group
		.. automethod:: replace
		.. automethod:: qualify
		.. automethod:: qualified_indices
		.. automethod:: qualified_groups
		
	.. collapse:: extraction methods
	
		.. automethod:: maxnum
		.. automethod:: precision
		.. automethod:: length
		.. automethod:: surface
		.. automethod:: barycenter
		.. automethod:: barycenter_points
		.. automethod:: box
		
		.. automethod:: usepointat
		.. automethod:: edgepoints
		.. automethod:: edgedirection
		
		.. automethod:: extremities
		.. automethod:: groupextremtities
		.. automethod:: frontiers
		.. automethod:: arcs
		
		.. automethod:: islands
		.. automethod:: groupislands
		.. automethod:: segmented
		
		.. automethod:: flip
