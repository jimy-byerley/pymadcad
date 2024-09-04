.. _bevel:

bevel    - Functions for cutting meshes at edges or points, like chamfer
========================================================================

.. automodule:: madcad.bevel

	.. image:: /schemes/bevel-cutter.svg

End-user functions
------------------

.. autofunction:: chamfer
	
	.. subfigure:: AB
		:layout-sm: AB
		:gap: 0px
	
		.. image:: /schemes/bevel-chamfer.svg

		.. image:: /screenshots/bevel-chamfer.png
			:width: 300

.. autofunction:: filet

	.. subfigure:: AB
		:layout-sm: AB
		:gap: 0px
	
		.. image:: /schemes/bevel-filet.svg

		.. image:: /screenshots/bevel-filet.png
			:width: 300

.. autofunction:: edgecut

	.. subfigure:: AB
		:layout-sm: AB
		:gap: 0px
	
		.. image:: /schemes/bevel-edgecut.svg

		.. image:: /screenshots/bevel-edgecut.png
			:width: 300



.. _cutter:

Cutters (cut methods)
---------------------

.. autofunction:: cutter_width

.. autofunction:: cutter_distance

.. autofunction:: cutter_depth
	
.. autofunction:: cutter_radius
