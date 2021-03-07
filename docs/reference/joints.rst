.. _joints:
.. py:module:: madcad.joints

joints	- Kinematic Joints definition
=====================================

Simple joints
-------------

.. autoclass:: madcad.joints.Pivot

.. autoclass:: madcad.joints.Gliding

.. autoclass:: madcad.joints.Track

.. autoclass:: madcad.joints.Punctiform

.. autoclass:: madcad.joints.Planar

Complex joints
--------------

.. autoclass:: madcad.joints.Gear

	.. note::
		As solids don't hold more data than only their pose (position and orientation), there is no way to distinguish the gear in a pose, and in the same pose rotated by :math:`2 \pi`.
		
		Therefore rotation jumps can occurs when several gears are tied. 

.. autoclass:: madcad.joints.Helicoid

	.. note::
		As solids orientation is in :math:`[0;~  2 \pi]`, when the gap in the helicoid is bigger than `step` then the joints solids only rotates by :code:`gap/step * 2*pi % 2*pi`
		
		If the helicoid is involved in any gearings or reductors, the result might be not what was expected.
		
