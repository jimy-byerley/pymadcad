Main concepts of madcad
=======================

Comparison with existing CAD softwares
--------------------------------------

Current CAD softwares are based on a full GUI oriented approach, with parametric features bases on dynamic relationship between the scene objects. This can leads to many problems:
	
	- Objects relationships are never visible, thus tricky to understand and to manage. On most of these softwares it's in fact impossible to manage it.
	
	- The software has to be very complex to handle all possible behaviors in parameters change in order to update the model.
	
	- The primitives is always limited to a small set fixed by design.
	
	- The GUI has to be very complex to allow the user to deal with every aspect of the objects (thus many tools, many object menus, sub-menus, context-dependent behaviors). The softwares are very hard to maintain, and often buggy
	
	- When a CAD file grow in complexity, we must avoid to modify it deeply or the entire build-tree fails and it's no more possible to repair.
	
	- The build tree is rarely human-readable.
	
The current library differs in its approach by the following points:

	- MADCAD is a script based CAD
		All the functions of madcad are available in the current API. Nothing is hidden.
		The python script is the build tree, it is as readable as the user makes it.
		We strive to keep all the functions simple and intuitive to allow a good readability and a fast scripting.

	- MADCAD is not a parametric software
		A madcad script is just a regular Python script. What makes the madcad files parametric is the ability to change variables and execute again the script.
		
		So madcad is generative, for implicit geometry definitions, the user has to use a solver before constructing the objects.
	
	- MADCAD is extensible
		As it's a regular script, it's possible to use from simple to complex programming concepts (if/else, loops, tables, functions, classes, module imports, generators, ...). Giving it infinite more possibilities, and in an easier way than with other softwares.

Design and concepts
--------------------

The last section describe the guidelines. Here is what these considerations brought us.

Scripting
~~~~~~~~~

The build tree is replaced by a script, thus a programming language was needed. For the best readability it had to have a very clean syntax. But for the highest flexibility it has to be very dynamic and powerful. Object orientation is necessary for transparent data manipulation, as well as dynamic typing. Functionnal programming and generators are welcome.

	Choice is `Python <https://www.python.org>`_
	
CAD Data
~~~~~~~~

The object oriented system helps to get each variable an identity depending on the true meaning of the contained data. We choose to get the classes the closest to the mathematical instrinsic meaning of the data.

	Choice is to split types across their geometric nature:
		:vector:  for both points and directions
		:axis:    for direction associated with an origin, as well as for planes
		:matrix:  for affine and linear transformations, rotations, bases
		:Mesh:    only for surfaces
		:Web:     only for lines
		:Solid:   for objects inside the same movement
		
		etc

Math library
~~~~~~~~~~~~

For efficient and simple linear algebra operations (vectors, matrices), We needed a library. It has to get very short type names (as often repeated in scripts and library). Few but universal operations. Compiled runtime and memory.

It appeared that the GPU `shader languages <https://docs.gl/sl4/all>`_ already fixed that for us, with a sort of universal API for 2-4 dimension types.

	Choice is `PyGLM <http://github.com/Zuzu-Typ/PyGLM>`_

Mesh Data
~~~~~~~~~

It's always a difficult way to stop a decision on the way to represent 3D objects. In most cases we represent it by its exterior surface, but the problem of the polygons remains: `ngons`, `quads`, `triangles`, `bezier`, ... ? And the way to put it into memory also: `simple unit elements`, `dynamically typed polygons`, `halfedges`, `quadedges`, ... Or even the way to access it: `arrays`, `chained lists`, `octree`, `hashmaps`.

The choice was not easy but was in favor of simplicity:
	
	- array storage:   ``typedlist``
	- simple unit elements:  ``vec3`` for points and ``uvec2``, ``uvec3`` for edges and triangles
	- surfaces are only made of triangles:  ``uvec3(a,b,c)``

This structure is very simple and common in the world or 3D computing, and it makes easy to write most of the operations. Therefore the storage is exposed to the user: The ``Mesh`` class is just referencing buffers the user can hack into. and a mesh can be created from arbitrary buffers of points and faces.

Points and faces were initially stored in regular python lists. It was convenient to write python algorithms using it, but any interoperability with compiled code, or with libraries using different data structures than list/pyglm. This is why `arrex <https://github.com/jimy-byerley/arrex>`_ was created. It behaves just like a python list, but stores its elements as in a buffer, and so can be easily converted or shared through numpy.

Display library
~~~~~~~~~~~~~~~

As 3D manipulation can be complex, it's mandatory to visualize any kind of objects. A part of the library is dedicated to display the objects in an OpenGL scene. But to allow user interactions and allow to embed the scene widget, we had to focus on one only paneling library.

We preferred to bet on the most - advanced and feature-complete - free library despite its differences with the python environment.

	Choice is `Qt <https://www.qt.io/>`_ but with `moderngl <https://github.com/moderngl/moderngl>`_ for the render pipeline
	
.. note::
	Qt5 is not very pythonic (it's designed for C++, no functional programming, huge inheritance instead of python protocols, string typing, enums, camelcase, ...). But since the release of `Qt6 (guidelines) <https://www.qt.io/blog/2019/08/07/technical-vision-qt-6>`_ more pythonicity can be found, that madcad will benefit from once upgraded to Qt6

References
~~~~~~~~~~

- `Other kind of mesh structures <https://en.wikipedia.org/wiki/Polygon_mesh>`_
- `CSG (Constructive Solid Geometry) - the data structure of the basic CAD softwares <https://en.wikipedia.org/wiki/Constructive_solid_geometry>`_
- `BREP (Bounded Representation) - the data structure of the advanced CAD softwares <https://fr.wikipedia.org/wiki/B-Rep>`_
