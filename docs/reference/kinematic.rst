.. _kinematic:

kinematic   - Kinematic solver/constraint system
================================================

.. automodule:: madcad.kinematic
	
.. autoclass:: KinematicError

.. autoclass:: Kinematic
	
	.. automethod:: normalize
	.. automethod:: direct
	.. automethod:: inverse
	.. automethod:: grad
	.. automethod:: cycles
	.. automethod:: solve
	.. automethod:: freedom
	.. automethod:: parts
	.. automethod:: display
	
	.. automethod:: to_chain

.. autoclass:: Joint
	
	.. automethod:: normalize
	.. automethod:: direct
	.. automethod:: inverse
	.. automethod:: grad
	
	.. automethod:: transmit
	.. automethod:: scheme
	.. automethod:: display
	
.. autoclass:: Weld
	
.. autoclass:: Free

.. autoclass:: Reverse

.. autoclass:: Chain

	.. automethod:: normalize
	.. automethod:: direct
	.. automethod:: inverse
	.. automethod:: grad
	.. automethod:: parts
	
	.. automethod:: to_kinematic
	.. automethod:: to_dh
	.. automethod:: from_dh
	
	.. automethod:: display


.. autofunction:: arcs
.. autofunction:: depthfirst
.. autofunction:: cycles
.. autofunction:: shortcycles

.. autoclass:: Solid

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



.. automodule:: madcad.kinematic.displays
