Installation
============

From source
-----------

You will need an archive of the source directory. Extract it somewhere then type the following commands in the extracted directory.

.. code-block:: sh

	python3 setup.py build_ext --inplace

The source directory can now be used as is.

You still need the PyQt5 library. As there is many possible sources, you have to install it manually so you can use the version/source you prefer.
Choose one of the following:

- from PyPI
	
	.. code-block:: sh
		
		pip3 install PyQt5
		
- from the linux repositories

	.. code-block:: sh
	
		sudo apt install python3-pyqt5 python3-pyqt5.qtopengl

Optionnal dependencies
~~~~~~~~~~~~~~~~~~~~~~

There is some optionnal dependencies you can choose to install by yourself, that enabled some features of pymadcad.

- `plyfile <https://github.com/dranjan/python-plyfile>`_		to read/write .PLY files
- `stl-numpy <https://github.com/pywavefront/PyWavefront>`_		to read/write .STL files
- `pywavefront <https://github.com/pywavefront/PyWavefront>`_	to read .OBJ files



From PyPI
---------

.. warning::
	The project is not published yet.


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

Refers to the PyPI section below.

