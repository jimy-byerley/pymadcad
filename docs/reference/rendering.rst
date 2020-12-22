.. _rendering:

.. currentmodule:: madcad.rendering

rendering   - 3D interface
==========================

This module provides a render pipeline system centered around class 'Scene' and a Qt widget 'View' for window integration and user interaction. 'Scene' is only to manage the objects to render (almost every madcad object). Most of the time you won't need to deal with it directly. The widget is what actually displays it on the screen.
The objects displayed can be of any type but must implement the display protocol
	
display protocol
----------------
a displayable is an object that implements the signatue of Display:

.. code-block:: python
	
		class display:
			box (Box)                      # delimiting the display, can be an empty or invalid box
			pose (fmat4)                   # local transformation
			
			stack(scene)                   # rendering routines (can be methods, or any callable)
			duplicate(src,dst)             # copy the display object for an other scene if possible
			upgrade(scene,displayable)     # upgrade the current display to represent the given displayable
			control(...)                   # handle events
			
			__getitem__                    # access to subdisplays if there is
	
For more details, see class Display below
	
.. note::
	As the GPU native precision is *f4* (float 32 bits), all the vector stuff regarding rendering is made using simple precision types: `fvec3, fvec4, fmat3, fmat4, ...` instead of the usual double precision `vec3`

.. note::
	There is some restrictions using the widget. This is due to some Qt limitations (and design choices), that Qt is using separated opengl contexts for each independent widgets or window.
	
	- a View should not be reparented once displayed
	- a View can't share a scene with Views from an other window
	- to share a Scene between Views, you must activate 
	
		.. code-block:: python
			
			QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)

Rendering system
----------------

.. autofunction:: show

.. py:data:: overrides
	
	dictionnary of callables used by `Scene.display` to override the classes `.display(scene)` method

.. py:data:: global_context

	shared open gl context, None if not yet initialized
	
.. py:data:: opengl_version

	minimum opengl required version

.. autoclass:: Display
	
	- Mendatory part for the scene
	
		.. automethod:: display
		.. automethod:: stack
		
		.. automethod:: duplicate
		.. automethod:: __getitem__
		.. automethod:: update
	
	- Optional part for Qt interaction
	
		:selected (bool):  flag set to True by left-clicking on the object
		
		.. automethod:: control
		


.. autoclass:: Scene
	:members:


.. autoclass:: View

	* Methods to get items on the screen
	
		.. automethod:: somenear
		.. automethod:: ptat
		.. automethod:: ptfrom
		.. automethod:: itemat
	
	* Methods to move camera
	
		.. automethod:: look
		.. automethod:: adjust
		.. automethod:: center
	
	* Event system
	
		.. automethod:: event
		.. automethod:: inputEvent
		.. automethod:: control
	
	* Rendering system
	
		.. automethod:: refreshmaps
		.. automethod:: identstep

view settings/interaction methods
---------------------------------

.. autoclass:: Turntable
.. autoclass:: Orbit
.. autoclass:: Perspective
.. autoclass:: Orthographic

helpers to trick into the pipeline
----------------------------------

.. autoclass:: Group
	:members: stack, __getitem__, dequeue, update, pose, world, box

.. autoclass:: Step
	:members: stack

.. autoclass:: Displayable
	:members: display

.. autofunction:: displayable
