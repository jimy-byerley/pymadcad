.. _primitives:

primitives     - 3D Primitives for Wire generation
==================================================

.. py:module:: madcad.primitives

Primitives are parametrized objects, that can be baked into a mesh/web/wire object. A primitive object must have the following signature:
	
.. code-block:: python

		class SomePrimitive:
			# method baking the primitive in some general-purpose 3D object
			# resolution is optional and defaults to the primitive settings that defaults to the current settings at call time
			def mesh(self, resolution=None) -> Mesh/Web/Wire:
				...
			
			# for the solver
			# primitive attributes the solver has to consider as variables or variable container
			slvvars = 'fields', 'for', 'solver', 'variables'
			# otpional method constraining the primitive parameters (to keep points on a circle for instance)
			def fit(self) -> err**2 as float:
				...


.. autofunction:: isprimitive

Curve resolution
----------------

Some primitive types are curves, the discretisation is important for visual as well as for result quality (remember that even if something looks like a perfect curves, it's still polygons).
The resolution (subdivision) of curve is done following the following cirterions present in the :ref:`settings module<settings>`

specification priority order:
	
	1. optional argument ``resolution`` passed to ``primitive.mesh()`` or to ``web()`` or ``wire()``
	2. optional attribute ``resolution`` of the primitive object
	3. value of ``settings.primitives['curve_resolution']`` at bake time.

specification format:

	.. code-block:: python
		
		('div', 16)   # fixed amount of 16 subdivisions
		('m', 0.1)     # divide every 0.1 distance unit
		('rad', 0.6)    # divide such that the max edge angle is 0.6 rad
		('radm', 0.6)	# divide such that max (edge angle)*(triangle height) is 0.6 rad.m
		('sqradm', 0.6)  # divide such that max (edge angle)*sqrt(triangle height) is 0.6 rad.m

		



primitives types
----------------

.. py:class:: Vector

	Alias to ``vec3``
	
.. py:class:: Point

	Alias to ``vec3``
	
.. autoclass:: Axis

	.. automethod:: __getitem__
	.. automethod:: flip
	.. automethod:: offset
	.. automethod:: transform
	
	.. image:: /screenshots/primitives-axis.png
	
.. autofunction:: isaxis

.. autoclass:: Segment

	.. autoproperty:: direction
	
	.. image:: /screenshots/primitives-segment.png

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
	
.. autoclass:: Circle

	.. autoproperty:: center
	
	.. autoproperty:: radius
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-circle.png

.. autoclass:: ArcTangent

	.. autoproperty:: center
	
	.. autoproperty:: radius
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-arctangent.png
	
.. autoclass:: TangentEllipsis

	.. autoproperty:: center
	
	.. autoproperty:: axis
	
	.. automethod:: tangent
	
	.. image:: /screenshots/primitives-tangentellipsis.png
	
.. autoclass:: Interpolated

	.. image:: /screenshots/primitives-spline-interpolated.png

.. autoclass:: Softened

	.. image:: /screenshots/primitives-spline-softened.png


