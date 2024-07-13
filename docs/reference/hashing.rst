.. _hashing:

hashing    - Fast access to space associated data
=================================================

.. automodule:: madcad.hashing

Connectivity
------------

.. autofunction:: madcad.mesh.edgekey
.. autofunction:: madcad.mesh.facekeyo
.. autofunction:: madcad.mesh.arrangeface
.. autofunction:: madcad.mesh.arrangeedge
.. autofunction:: madcad.mesh.connpp
.. autofunction:: madcad.mesh.connef
.. autofunction:: madcad.mesh.connpe
.. autofunction:: madcad.mesh.connexity

.. autofunction:: madcad.mesh.suites

.. autoclass:: PositionMap
	:members: keysfor, update, add, get, display, __contains__
	:undoc-members:

.. autofunction:: meshcellsize
	
.. autoclass:: PointSet
	:members: keyfor, update, difference_update, add, remove, discard, contains, __getitem__, __contains__, __add__, __sub__, __iadd__, __isub__
	:undoc-members:

.. autoclass:: Asso
	:automembers:
	:undoc-members:
