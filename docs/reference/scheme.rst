.. _scheme:

scheme   - annotation functionalities
=====================================

.. automodule:: madcad.scheme

schematics system
-----------------

	.. autoclass:: Scheme
	
		.. automethod:: set
		.. automethod:: add
		.. automethod:: component
		.. automethod:: __add__
		
	The following functions are commonly used spaces in madcad schematics
			
	.. py:data:: view
	.. py:data:: screen
	.. py:data:: world
	.. autofunction:: halo_world
	.. autofunction:: halo_view
	.. autofunction:: halo_screen
	.. autofunction:: scale_screen
	.. autofunction:: scale_view


annotation functions
--------------------

	.. autofunction:: note_leading
		
		.. image:: /screenshots/note_leading.png
	
	.. autofunction:: note_floating
	
		.. image:: /screenshots/note_floating.png
	
	.. autofunction:: note_distance
		
		.. image:: /screenshots/note_distance.png
	
	.. autofunction:: note_distance_planes
		
		.. image:: /screenshots/note_distance_planes.png
		
	.. autofunction:: note_distance_set
		
		.. image:: /screenshots/note_distance_set.png
		
	.. autofunction:: note_bounds
	
		.. image:: /screenshots/note_bounds.png
	
	.. autofunction:: note_radius
	
		.. image:: /screenshots/note_radius.png
		
	.. autofunction:: note_angle
		
		.. image:: /screenshots/note_angle.png
		
	.. autofunction:: note_angle_planes
	.. autofunction:: note_angle_edge
		
		.. image:: /screenshots/note_angle_edge.png
		
	.. autofunction:: note_label
		
		.. image:: /screenshots/note_label.png
	
measuring tools
---------------

	.. autofunction:: mesh_curvature_radius
	.. autofunction:: mesh_curvatures
