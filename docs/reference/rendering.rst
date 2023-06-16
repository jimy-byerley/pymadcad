.. _rendering:

.. currentmodule:: madcad.rendering

rendering   - 3D user interface
===============================

This module provides a render pipeline system featuring:

- Class `Scene` to gather the data to render
- Widget `View` that actually renders the scene 
- The display protocol, that allows any object to define its `Display` subclass to be rendered in a scene.

The view is for window integration and user interaction. `Scene` is only to manage the objects to render . Almost all madcad data types can be rendered to scenes being converted into an appropriate subclass of `Display`. Since the conversion from madcad data types into display instance is automatically handled via the *display protocol*, you usually don't need to deal with displays directly.

.. autofunction:: show

.. py:data:: opengl_version

	Minimum opengl version required by the rendering pipeline
	
	
Display protocol
----------------

A displayable is an object that implements the signature of `Display`.


.. autofunction:: displayable

.. autoclass:: Display
	
	- Mendatory interface for the scene
	
		.. automethod:: display
		.. automethod:: stack
		.. automethod:: __getitem__
		
		.. automethod:: duplicate
		.. automethod:: update
	
	- Optional interface, only for Qt interaction
	
		.. py:attribute:: selected
			
			flag set to True by left-clicking on the object
		
		.. automethod:: control
		

Rendering system
----------------
	
.. note::
	As the GPU native precision is *f4* (float 32 bits), all the vector stuff regarding rendering is made using simple precision types: `fvec3, fvec4, fmat3, fmat4, ...` instead of the usual double precision `vec3`

.. py:data:: overrides
	
	Dictionary of callables used by `Scene.display` to override the display protocol method `object.display(scene)`

.. py:data:: global_context

	Shared open gl context, None if not yet initialized
	

.. autoclass:: Scene
	:members:
	
	.. automethod:: __getitem__
	.. automethod:: __setitem__

	
Views classes
-------------

.. autoclass:: View
	
	.. note::
		There is some restrictions using the widget. Due to some Qt limitations (and design choices), Qt is using separated opengl contexts for each independent widgets or window.
		
		- a View should not be reparented once displayed
		- a View can't share a scene with Views from an other window
		- to share a Scene between Views, you must activate 
		
			.. code-block:: python
				
				QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)

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
		
.. autoclass:: Offscreen

	* Methods to get items on the screen
	
		.. automethod:: somenear
		.. automethod:: ptat
		.. automethod:: ptfrom
		.. automethod:: itemat
	
	* Methods to move camera
	
		.. automethod:: look
		.. automethod:: adjust
		.. automethod:: center
	
	* Rendering system
	
		.. automethod:: refreshmaps
		.. automethod:: identstep


View projections and navigation
-------------------------------

.. autoclass:: Turntable
.. autoclass:: Orbit
.. autoclass:: Perspective
.. autoclass:: Orthographic

Helpers to trick into the pipeline
----------------------------------

.. autoclass:: Group
	:members: stack, __getitem__, dequeue, update
	
	.. autoproperty:: pose
	.. autoproperty:: world
	.. autoproperty:: box

.. autoclass:: Step

.. autoclass:: Displayable
