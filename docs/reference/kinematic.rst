.. _kinematic:

kinematic   - Kinematic solver/constraint system
================================================

.. automodule:: madcad.kinematic
	
.. autoclass:: KinematicError

.. autoclass:: Kinematic

	.. autoproperty:: default
	.. autoproperty:: bounds
	
	.. automethod:: solve
	.. automethod:: direct
	.. automethod:: inverse
	.. automethod:: grad
	.. automethod:: parts
	.. automethod:: display

.. autoclass:: Joint
	
	.. automethod:: direct
	.. automethod:: inverse
	
	.. automethod:: grad
	.. automethod:: transmit
	.. automethod:: schemes
	.. automethod:: display
	
.. autoclass:: Weld
	
.. autoclass:: Free

.. autoclass:: Reverse

.. autoclass:: Chain

	.. automethod:: parts
	
	.. automethod:: to_kinematic
	.. automethod:: to_dh
	.. automethod:: from_dh
	
	.. automethod:: display

.. autoclass:: Kinemanip


.. autofunction:: arcs
.. autofunction:: depthfirst
.. autofunction:: cycles
.. autofunction:: shortcycles

.. autoclass:: Solid

	.. autoproperty:: pose
	
	.. automethod:: transform
	.. automethod:: place
	
	.. automethod:: set
	.. automethod:: add
	.. automethod:: __getitem__
	.. automethod:: __setitem__
	.. automethod:: display
	
	

.. autoclass:: Screw
	:members: locate, transform
	
	Of course, as any vector variables, ``Screw`` implements ``+ -`` with other ``Torsor``, and ``* /`` with ``float``

.. autofunction:: comomentum


	

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

