.. _hashing:

hashing    - Fast access to space associated data
=================================================

.. automodule:: madcad.hashing

Connectivity
------------

.. autofunction:: edgekey
.. autofunction:: facekeyo
.. autofunction:: arrangeface
.. autofunction:: arrangeedge
.. autofunction:: connpp
.. autofunction:: connef
.. autofunction:: connpe
.. autofunction:: connexity

.. autofunction:: suites

Specific Hashmaps
-----------------

.. autoclass:: PositionMap
	:members: keysfor, update, add, get, display, __contains__
	:undoc-members:

.. autofunction:: meshcellsize
	
.. autoclass:: PointSet
	:members: keyfor, update, difference_update, add, remove, discard, contains, __getitem__, __contains__, __add__, __sub__, __iadd__, __isub__
	:undoc-members:

.. autoclass:: Asso
	:members: __getitem__, __contains__, add, remove, discard, update, __add__, clear, items, keys, values, connexity
	:undoc-members:
