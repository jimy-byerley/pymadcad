.. _gear:

gear   - generation of gears, racks, etc
========================================

.. automodule:: madcad.gear


tooth profiles generation
-------------------------

The following functions are focussing on involute gears. If you want more details on how involutes gears are defined and computed, you can take a look at the `algorithm section <https://pymadcad.readthedocs.io/en/latest/algorithms/gearprofile.html>`_

.. autofunction:: rackprofile
.. autofunction:: gearprofile

gear generation
---------------

.. autofunction:: gear
.. autofunction:: geargather
.. autofunction:: gearexterior
.. autofunction:: gearstructure
.. autofunction:: gearhub


helper tools
------------

.. autofunction:: gearcircles
.. autofunction:: involute
.. autofunction:: involuteat
.. autofunction:: involuteof

structure patterns
------------------
	
.. autofunction:: pattern_circle
.. autofunction:: pattern_full
.. autofunction:: pattern_rect
.. autofunction:: pattern_rounded
