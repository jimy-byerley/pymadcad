Installation
============

In order to install ``pymadcad``, you need a version of Python ``>=3.8``

From PyPI
---------

.. code-block:: sh

	pip install pymadcad
	
You can also select one or more optional dependencies :

.. code-block:: sh
	
	pip install pymadcad[stl,ply,obj]
	
This installation may require build-dependencies, so please refer to source dependencies below

Optional dependencies
~~~~~~~~~~~~~~~~~~~~~~

There is some optional dependencies you can choose to install by yourself to enable some features of pymadcad.

- `plyfile <https://github.com/dranjan/python-plyfile>`_		to read/write ``.ply`` files
- `stl-numpy <https://github.com/WoLpH/numpy-stl>`_		to read/write ``.stl`` files
- `pywavefront <https://github.com/pywavefront/PyWavefront>`_	to read ``.obj`` files

From source
-----------

Build dependencies
~~~~~~~~~~~~~~~~~~

- Debian-based distributions

	.. code-block:: sh
	
		apt install gcc binutils python3-dev
		
- Windows distributions

	You will need Visual Studio (licensed) or MSVC redistribuable (free to download) installed



Module dependencies
~~~~~~~~~~~~~~~~~~~

Make sure you installed the dependencies:

.. code-block:: sh

	pip install moderngl pyglm pillow numpy scipy pyyaml arrex

You still need the PyQt5 library. As there is many possible sources, you have to install it manually so you can use the version/source you prefer.
Choose one of the following:

- Qt from PyPI
	
	.. code-block:: sh
		
		pip install PyQt5
		
- Qt from the Linux repositories

	.. code-block:: sh
	
		sudo apt install python3-pyqt5 python3-pyqt5.qtopengl

Compile the module
~~~~~~~~~~~~~~~~~~

You will need an archive of the `source directory <https://github.com/jimy-byerley/pymadcad>`_. Extract it somewhere then type the following commands in the extracted directory.

.. code-block:: sh

	python setup.py build_ext --inplace

The source directory can now be imported.

