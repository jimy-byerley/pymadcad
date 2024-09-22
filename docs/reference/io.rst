.. _io:
.. py:module:: madcad.io

io    - Read/write mesh or data files
=====================================

simple read/write files
-----------------------

.. autoclass:: FileFormatError

.. autofunction:: madcad.io.read

.. autofunction:: madcad.io.write

.. autofunction:: madcad.io.module

caching
-------

.. data:: cachedir

   the default folder for cache files

.. autofunction:: madcad.io.cached

.. autofunction:: madcad.io.cachedmodule
