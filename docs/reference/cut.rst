.. _cut:

cut    - functions for cutting meshes at edges or points, like chamfer
======================================================================

.. automodule:: madcad.cut

end-user functions
------------------

	.. autofunction:: chamfer
		
		This is a chamfer on edges around a cube corner 
		
		.. image:: /screenshots/chamfer.png
			:width: 300

	.. autofunction:: bevel

		This is a bevel on edges around a cube corner
		
		.. image:: /screenshots/bevel.png
			:width: 300

	.. autofunction:: multicut

		This is the result on edges around a cube corner
		
		.. image:: /screenshots/multicut.png
			:width: 300



.. _cutter:

cutters (cut methods)
---------------------

	.. image:: /schemes/cutter.svg

	.. autofunction:: cutter_width

	.. autofunction:: cutter_distance

		.. warning::
			this cut method can be unstable at the moment

	.. autofunction:: cutter_depth

		.. warning::
			this cut method can be unstable at the moment
			
	.. autofunction:: cutter_angle

		.. warning::
			this cut method can be unstable at the moment

helpers
-------

	.. autofunction:: cut
	.. _planeoffsets:
	.. autofunction:: planeoffsets
	.. autofunction:: tangentjunction
	.. autofunction:: tangentcorner
	.. autofunction:: tangentend
