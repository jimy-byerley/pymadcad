.. _constraints:

constraints    - primitive constraints
======================================

.. py:module:: madcad.constraints

This modules defines the constraints definitions and the solver tools

Constraints can be any object referencing `variables` and implementing the following signature to guide the solver in the resolution:
		
.. code-block:: python

		class SomeConstraint:
			
			# name the attributes referencing the solver variables to change
			slvvars = 'some_primitive', 'some_point'
			# function returning the squared error in the constraint
			
			#  for a coincident constraint for instance it's the squared distance
			#  the error must be a contiguous function of the parameters, and it's squared for numeric stability reasons.
			#  the solver internally use an iterative approach to optimize the sum of all fit functions.
			def fit((self):
				...

.. autofunction:: isconstraint

solver
------

.. autofunction:: solve

.. autoclass:: Problem
	:members:
	
constraints definitions
-----------------------

.. autoclass:: Distance(p1, p2)
.. autoclass:: Radius(arc, radius)
.. autoclass:: Angle(s1, s2)
.. autoclass:: Tangent(c1, c2, p)
.. autoclass:: OnPlane(axis, pts: list)
	

