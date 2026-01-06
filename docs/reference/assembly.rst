.. _assembly:

assembly   - functions to group and move together 3d objects
============================================================

.. automodule:: madcad.assembly
	
.. autoclass:: Solid

	.. autoproperty:: pose

	.. automethod:: transform
	.. automethod:: place
	
	.. automethod:: loc
	.. automethod:: deloc
	
	.. automethod:: set
	.. automethod:: append
	.. automethod:: __getitem__
	.. automethod:: __setitem__
	.. automethod:: display
	


.. autofunction:: placement

	suppose we have those parts to assemble and it's hard to guess the precise pose transform between them
	
	.. image:: /screenshots/placement-before.png
	
	placement gives the pose for the screw to make the selected surfaces coincide
	
	.. image:: /screenshots/placement-after.png

.. autofunction:: explode

	before operation
	
	.. image:: /screenshots/explode-before.png
	
	after operation
	
	.. image:: /screenshots/explode-after.png

.. autofunction:: explode_offsets
