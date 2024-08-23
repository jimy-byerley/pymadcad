.. _standard:

standard     - Common standard parametric parts
===============================================

.. automodule:: madcad.standard

Screw stuff
~~~~~~~~~~~

.. autofunction:: nut

	.. image:: /screenshots/hexnut.png

.. autofunction:: screw

	.. image:: /screenshots/screw.png

.. autofunction:: washer

	.. image:: /screenshots/washer.png
	
.. autofunction:: bolt

	.. image:: /screenshots/bolt.png

Coilsprings
~~~~~~~~~~~

.. autofunction:: coilspring_compression

	.. image:: /screenshots/coilspring-compression.png

.. autofunction:: coilspring_tension

	.. image:: /screenshots/coilspring-tension.png

.. autofunction:: coilspring_torsion

	.. image:: /screenshots/coilspring-torsion.png

Bearings
~~~~~~~~

.. autofunction:: bearing

**Examples**

	By default `bearing()` provides a simplified representation of the bearing, showing mostly its functionnal surfaces (interfaces with other parts). To show all the interior details, use the `detail=True` option.
	
	.. code-block:: python
		
		# bounded representation of a ball bearing
		bearing(12)
		
	.. image:: /screenshots/bearing-bounded.png

**Overview**
	
	* Ball bearing
	
		Those are extremely flexible in the force direction, way to mount, and quite cheap because widely used.

		.. code-block:: python

			# detailed representation of a ball bearing
			bearing(12, detail=True)
			
		.. image:: /screenshots/bearing-ball.png

	* Roller bearing
	
		Those can hold a much bigger force than ball bearings, but must be carefully mounted in the direction of that force to always be in pressure (requiring extra design for this)

		.. code-block:: python

			# roller bearing 
			bearing(12, circulating='roller', contact=radians(20), detail=True)


		.. image:: /screenshots/bearing-roller.png

	* Thrust bearing
		
		Those can hold a bigger force that radial ball bearings, but must be carefully mounted to always stay in pressure. They also are less precise than the two previous.

		.. code-block:: python

			# thrust bearing
			bearing(12, contact=radians(90), detail=True)
			
		.. image:: /screenshots/bearing-thrust.png
		
		
.. autofunction:: slidebearing

**Examples**

	* With slight breach
	
		.. code-block:: python
			
			# put an opening to better fit bad bore diameter
			slidebearing(10, 12, 0.5, open=True)
			
		.. image:: /screenshots/slidebearing-opened.png
	
	* With shoulder
	
		.. code-block:: python
		
			# put a shoulder to support a slight thrust
			slidebearing(10, 12, shoulder=3)
			
		.. image:: /screenshots/slidebearing-shoulder.png
	
Standard sections for extrusion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: section_tslot

	.. image:: /screenshots/section_tslot.png

.. autofunction:: section_s

	.. image:: /screenshots/section_s.png

.. autofunction:: section_w

	.. image:: /screenshots/section_w.png
	
.. autofunction:: section_l

	.. image:: /screenshots/section_l.png
	
.. autofunction:: section_c

	.. image:: /screenshots/section_c.png
	

Numeric helpers
~~~~~~~~~~~~~~~

.. autofunction:: stfloor
.. autofunction:: stceil
.. py:data:: standard_digits


convenient shapes to integrate standard parts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autofunction:: screw_slot

	.. image:: /screenshots/screw_slot.png

.. autofunction:: bolt_slot

	.. image:: /screenshots/bolt_slot.png

.. autofunction:: bearing_slot_exterior

	.. image:: /screenshots/bearing_slot_exterior-evade.png
	.. image:: /screenshots/bearing_slot_exterior-expand.png

.. autofunction:: bearing_slot_interior

	.. image:: /screenshots/bearing_slot_interior-evade.png
	.. image:: /screenshots/bearing_slot_interior-expand.png
	
.. autofunction:: circular_screwing
.. autofunction:: grooves_profile
.. autofunction:: grooves

kinematics
~~~~~~~~~~

.. autofunction:: scara

	.. image:: /screenshots/standard-scara.png

.. autofunction:: serial6

	.. image:: /screenshots/standard-serial6.png

.. autofunction:: serial7

	.. image:: /screenshots/standard-serial7.png

.. autofunction:: delta3

	.. image:: /screenshots/standard-delta3.png
