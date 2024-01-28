.. _joints:
.. py:module:: madcad.joints

joints	- Kinematic Joints definition
=====================================

Usual joints
------------
	
.. autoclass:: madcad.joints.Ball

	.. image:: /screenshots/joints-ball.png

.. autoclass:: madcad.joints.Pivot

	.. image:: /screenshots/joints-pivot.png

.. autoclass:: madcad.joints.Gliding

	.. image:: /screenshots/joints-gliding.png

.. autoclass:: madcad.joints.Track

	.. image:: /screenshots/joints-track.png

.. autoclass:: madcad.joints.Punctiform

	.. image:: /screenshots/joints-punctiform.png

.. autoclass:: madcad.joints.Planar

	.. image:: /screenshots/joints-planar.png

.. autoclass:: madcad.joints.Punctiform
.. autoclass:: madcad.joints.Ring
.. autoclass:: madcad.joints.Hinge


Complex joints
--------------

.. autoclass:: madcad.joints.Project
.. autoclass:: madcad.joints.Rack

.. autoclass:: madcad.joints.Gear

	.. note::
		As solids don't hold more data than only their pose (position and orientation), there is no way to distinguish the gear in a pose, and in the same pose rotated by :math:`2 \pi`.
		
		Therefore rotation jumps can occurs when several gears are tied. 

.. autoclass:: madcad.joints.Helicoid

	.. note::
		As solids orientation is in :math:`[0;~  2 \pi]`, when the gap in the helicoid is bigger than `step` then the joints solids only rotates by :code:`gap/step * 2*pi % 2*pi`
		
		If the helicoid is involved in any gearings or reductors, the result might be not what was expected.
		
	.. image:: /screenshots/joints-helicoid.png

Mesh - based joints
-------------------

.. autoclass:: madcad.joints.Cam
.. autoclass:: madcad.joints.Contact
