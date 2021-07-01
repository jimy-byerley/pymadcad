.. _gear:

gear   - generation of gears, racks, etc
========================================

.. automodule:: madcad.gear


tooth profiles generation
-------------------------

The following functions are focussing on involute gears. If you want more details on how involutes gears are defined and computed, you can take a look at the `algorithm section <https://pymadcad.readthedocs.io/en/latest/algorithms/gearprofile.html>`_

.. autofunction:: rackprofile
.. autofunction:: gearprofile

	The result is the following curve (the white one):
	
	.. image:: /screenshots/gearprofile.png

gear generation
---------------

.. autofunction:: gear

	a simple use:
	
		>>> gear(3, 12, 4, bore_radius=1.5)
	
	.. image:: /screenshots/gear-minimal.png
	
	a more advanced use:
	
		>>> gear(3, 30, 4, bore_radius=2, pattern='rounded', patterns=6, int_height=1, chamfer=radians(20))
		
	.. image:: /screenshots/gear-advanced.png

.. autofunction:: geargather
.. autofunction:: gearexterior

	.. image:: /screenshots/gearexterior.png

.. autofunction:: gearstructure

	.. image:: /screenshots/gearstructure.png

.. autofunction:: gearhub


helper tools
------------

.. autofunction:: gearcircles
.. autofunction:: involute
.. autofunction:: involuteat
.. autofunction:: involuteof
.. autofunction:: repeat_circular

structure patterns
------------------

Those are the functions generating usual structures ready to used in `geargather`.
	
.. autofunction:: pattern_circle
.. autofunction:: pattern_full
.. autofunction:: pattern_rect
.. autofunction:: pattern_rounded
