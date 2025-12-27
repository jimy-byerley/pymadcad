.. _gear:

gear   - Generation of gears, racks, etc
========================================

.. automodule:: madcad.gear


Tooth profiles generation
-------------------------

The following functions are focussing on involute gears. If you want more details on how involutes gears are defined and computed, you can take a look at the `algorithm section <https://pymadcad.readthedocs.io/en/latest/algorithms/gearprofile.html>`_

.. autofunction:: rackprofile

	example:
	
		>>> rackprofile(1)
		
	.. image:: /screenshots/rackprofile.png

.. autofunction:: gearprofile

	example:
		
		>>> gearprofile(1, 12)

	.. image:: /screenshots/gearprofile.png
	
.. autofunction:: spherical_rackprofile

	example:
	
		>>> spherical_rackprofile(12)
	
	.. image:: /screenshots/spherical_rackprofile.png

.. autofunction:: spherical_gearprofile

	example:
	
		>>> spherical_gearprofile(12, pi/4)

	.. image:: /screenshots/spherical_gearprofile.png

Gear generation
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

Bevel gear generation
---------------------

.. autofunction:: bevelgear

	example:
	
		>>> bevelgear(1, 12, pi/3)

	.. image:: /screenshots/bevelgear.png


Helper tools
------------

.. autofunction:: gearcircles
.. autofunction:: involute
.. autofunction:: involuteat
.. autofunction:: involuteof
.. autofunction:: spherical_involute
.. autofunction:: spherical_involuteof

Structure patterns
------------------

Those are the functions generating usual structures ready to used in `geargather`.
	
.. autofunction:: pattern_circle
.. autofunction:: pattern_full
.. autofunction:: pattern_rect
.. autofunction:: pattern_rounded
