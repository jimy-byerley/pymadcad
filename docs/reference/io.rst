.. _io:
.. py:module:: madcad.io

io    - Read/write mesh or data files
=====================================

.. autofunction:: madcad.io.read

.. autofunction:: madcad.io.write

.. autofunction:: madcad.io.cache

.. py:data:: caches

	dict containing the data objects, associated to their filename.
	
	.. code-block:: python
		
		{'filename': (read_time, data_loaded)}

.. autoclass:: FileFormatError
