.. _standard:

standard     - common standard parametric parts
===============================================

.. automodule:: madcad.standard

screw stuff
~~~~~~~~~~~

.. autofunction:: nut

	.. image:: /screenshots/hexnut.png

.. autofunction:: screw

	.. image:: /screenshots/screw.png

.. autofunction:: washer

	.. image:: /screenshots/washer.png

coilsprings
~~~~~~~~~~~

.. autofunction:: coilspring_compression

	.. image:: /screenshots/coilspring-compression.png

.. autofunction:: coilspring_tension

	.. image:: /screenshots/coilspring-tension.png

.. autofunction:: coilspring_torsion

	.. image:: /screenshots/coilspring-torsion.png

bearings
~~~~~~~~

.. autofunction:: bearing

**examples**

	By default `bearing()` provides a simplified representation of the bearing, showing mostly its functionnal surfaces (interfaces with other parts). To show all the interior details, use the `detail=True` option.
	
	.. code::
		
		# bounded representation of a ball bearing
		bearing(12)
		
	.. image:: /screenshots/bearing-bounded.png

**overview**
	
	* ball bearing
	
		Those are extremely flexible in the force direction, way to mount, and quite cheap because widely used.

		.. code::

			# detailed representation of a ball bearing
			bearing(12, detail=True)
			
		.. image:: /screenshots/bearing-ball.png

	* roller bearing
	
		Those can hold a much bigger force than ball bearings, but must be carefully mounted in the direction of that force to always be in pressure (requiring extra design for this)

		.. code::

			# roller bearing 
			bearing(12, circulating='roller', contact=radians(20), detail=True)


		.. image:: /screenshots/bearing-roller.png

	* thrust bearing
		
		Those can hold a bigger force that radial ball bearings, but must be carefully mounted to always stay in pressure. They also are less precise than the two previous.

		.. code::

			# thrust bearing
			bearing(12, contact=radians(90), detail=True)
			
		.. image:: /screenshots/bearing-thrust.png
		
		
