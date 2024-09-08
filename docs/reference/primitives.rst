.. _primitives:

primitives     - 3D Primitives for Wire generation
==================================================

.. py:module:: madcad.primitives

Primitives are parametrized objects, that can be baked into a mesh/web/wire object. A primitive object must have the following signature:
	
.. code-block:: python

		class SomePrimitive:
			# method baking the primitive in some general-purpose 3D object
			# resolution is optional and defaults to the primitive settings 
			# that defaults to the current settings at call time
			def mesh(self, resolution=None) -> Mesh/Web/Wire:
				...
			
			# for the solver
			# primitive attributes the solver has to consider as variables or variable container
			slvvars = 'fields', 'for', 'solver', 'variables'
			# optional method constraining the primitive parameters (to keep points on a circle for instance)
			def fit(self) -> err**2 as float:
				...


.. autofunction:: isprimitive

Curve resolution
----------------

Some primitive types are curves, the discretization is important for visual as well as for result quality (remember that even if something looks like a perfect curves, it's still polygons).
The resolution (subdivision) of curve is done following the following criterion present in the :ref:`settings module<settings>`

Specification priority order:
	
	1. Optional argument ``resolution`` passed to ``primitive.mesh()`` or to ``web()`` or ``wire()``
	2. Optional attribute ``resolution`` of the primitive object
	3. Value of ``settings.resolution`` at bake time.

Specification format:

	.. code-block:: python
		
		('fixed', 16)   # fixed amount of 16 subdivisions
		('rad', 0.6)    # max polygon angle is 0.6 rad
		('radm', 0.6)
		('radm2', 0.6)


Primitives types
----------------


.. autoclass:: Segment

	.. autoproperty:: direction
	
	.. image:: /screenshots/primitives-segment.png
	
.. autoclass:: Circle

	.. autoproperty:: center
	
	.. autoproperty:: radius
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-circle.png

.. autoclass:: ArcCentered

	.. autoproperty:: center
	
	.. autoproperty:: radius
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-arccentered.png

.. autoclass:: ArcThrough

	.. autoproperty:: center
	
	.. autoproperty:: radius
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-arcthrough.png

.. autoclass:: ArcTangent

	.. autoproperty:: center
	
	.. autoproperty:: radius
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-arctangent.png
	
.. autoclass:: Ellipsis

	.. autoproperty:: axis
	
	.. image:: /screenshots/primitives-ellipsis.png
	
.. autoclass:: TangentEllipsis

	.. autoproperty:: center
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-tangentellipsis.png
	
.. autoclass:: Interpolated

	.. image:: /screenshots/primitives-spline-interpolated.png

.. autoclass:: Softened

	.. image:: /screenshots/primitives-spline-softened.png
