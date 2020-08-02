Installation
============


From PyPI
---------

.. code-block:: sh

	pip install pymadcad

Optionnal dependencies
~~~~~~~~~~~~~~~~~~~~~~

There is some optionnal dependencies you can choose to install by yourself to enable some features of pymadcad.

- `plyfile <https://github.com/dranjan/python-plyfile>`_		to read/write ``.ply`` files
- `stl-numpy <https://github.com/pywavefront/PyWavefront>`_		to read/write ``.stl`` files
- `pywavefront <https://github.com/pywavefront/PyWavefront>`_	to read ``.obj`` files


From source
-----------

You will need an archive of the `source directory <https://github.com/jimy-byerley/pymadcad>`. Extract it somewhere then type the following commands in the extracted directory.

Make sure you installed the dependencies:

.. code-block:: sh

	pip install moderngl pyglm cython pillow numpy scipy

You still need the PyQt5 library. As there is many possible sources, you have to install it manually so you can use the version/source you prefer.
Choose one of the following:

- Qt from PyPI
	
	.. code-block:: sh
		
		pip install PyQt5
		
- Qt from the linux repositories

	.. code-block:: sh
	
		sudo apt install python3-pyqt5 python3-pyqt5.qtopengl

Compile the module:		

.. code-block:: sh

	python setup.py build_ext --inplace

The source directory can now be used as is.


For Debian/Ubuntu
-----------------

.. warning::
	There is no download page yet.

Download the `.deb` file, then

.. code-block:: sh

	sudo dpkg -i python3-pymadcad.deb
	sudo apt -f install

For Windows
-----------

the ``pip`` command comes with most of the python distributions, so refers to the PyPI section below.

