.. _rendering:

rendering   - 3D interface
==========================

This module provides a render pipeline system centered around class 'Scene' and a Qt widget 'View' for window integration and user interaction. 'Scene' is only to manage the objects to render (almost every madcad object). Most of the time you won't need to deal with it directly. The widget is what actually displays it on the screen.
The objects displayed can be of any type but must implement the display protocol
	
display protocol
----------------
a displayable is an object that implements the signatue of Display:

.. code-block:: python
	
	class display:
		box (Box)                      delimiting the display, can be an empty or invalid box
		pose (fmat4)                    local transformation
		
		stack(scene)                   rendering routines (can be methods, or any callable)
		duplicate(src,dst)             copy the display object for an other scene if possible
		upgrade(scene,displayable)     upgrade the current display to represent the given displayable
		control(...)                   handle events
		
		__getitem__                    access to subdisplays if there is

For more details, see class Display below

.. note::
	There is some restrictions using the widget. This is due to some Qt limitations (and design choices), that Qt is using separated opengl contexts for each independent widgets or window.
	
	- a View should not be reparented once displayed
	- a View can't share a scene with Views from an other window
	- to share a Scene between Views, you must activate 
	
		.. code-block:: python
			
			QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)

Module content
--------------

.. autofunction:: madcad.show

.. autoclass:: madcad.rendering.Display
	
	- Mendatory part for the scene
	
		.. automethod:: display
		.. automethod:: stack
		
		.. automethod:: duplicate
		.. automethod:: __getitem__
		.. automethod:: update
	
	- Optional part for Qt interaction
	
		:selected(bool):  flag set to True by left-clicking on the object
		
		.. automethod:: control

.. autoclass:: madcad.rendering.Scene
	:members:

.. autoclass:: madcad.rendering.View

	* Methods to get items on the screen
	
		.. automethod:: somenear
		.. automethod:: ptat
		.. automethod:: ptfrom
		.. automethod:: itemat
	
	**Methods to move camera**
	
		.. automethod:: look
		.. automethod:: adjust
		.. automethod:: center
	
	**Event system**
	
		.. automethod:: event
		.. automethod:: inputEvent
		.. automethod:: control
	
	**Rendering system**
	
		.. automethod:: refreshmaps
		.. automethod:: identstep

.. autoclass:: madcad.rendering.Turntable
.. autoclass:: madcad.rendering.Orbit
.. autoclass:: madcad.rendering.Perspective
.. autoclass:: madcad.rendering.Orthographic
