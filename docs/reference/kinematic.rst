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
	
.. 	.. automethod:: transmit
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

.. automodule:: madcad.kinematic.displays
