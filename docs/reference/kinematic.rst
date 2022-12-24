.. _kinematic:

kinematic   - Kinematic solver/constraint system
================================================

.. automodule:: madcad.kinematic


.. autoclass:: Solid

	.. autoproperty:: pose
	
	.. automethod:: transform
	.. automethod:: place
	
	.. automethod:: set
	.. automethod:: add
	.. automethod:: __getitem__
	.. automethod:: __setitem__
	.. automethod:: display

.. autoclass:: Kinematic

	.. autoproperty:: pose
	
	.. automethod:: transform
	.. automethod:: itransform
	
	.. automethod:: solve
	
	.. automethod:: __copy__
	.. automethod:: __add__
	
	.. automethod:: display
	
.. autoclass:: Joint

.. autofunction:: isjoint

.. autofunction:: solvekin

	See :ref:`the joints module<joints>` for joints definitions.
	
.. autoclass:: Screw
	:members: locate, transform
	
	Of course, as any vector variables, ``Screw`` implements ``+ -`` with other ``Torsor``, and ``* /`` with ``float``


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

