.. _kinematic:

kinematic   - Kinematic solver/constraint system
================================================

.. automodule:: madcad.kinematic


.. autoclass:: Screw
	:members: locate, transform
	
	Of course, as any vector variables, ``Screw`` implements ``+ -`` with other ``Torsor``, and ``* /`` with ``float``

.. autoclass:: Solid
	:members: pose, transform

.. autoclass:: Kinematic
	:members: solve, pose

.. autofunction:: solvekin

	See :ref:`the joints module<joints>` for joints definitions.

.. autofunction:: makescheme

