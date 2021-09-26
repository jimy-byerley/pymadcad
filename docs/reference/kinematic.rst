.. _kinematic:

kinematic   - Kinematic solver/constraint system
================================================

.. automodule:: madcad.kinematic


.. autoclass:: Screw
	:members: locate, transform
	
	Of course, as any vector variables, ``Screw`` implements ``+ -`` with other ``Torsor``, and ``* /`` with ``float``

.. autoclass:: Solid
	:members: pose, transform, place, add, set

.. autoclass:: Kinematic
	:members: solve, pose

.. autofunction:: solvekin

	See :ref:`the joints module<joints>` for joints definitions.

.. autofunction:: makescheme

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
