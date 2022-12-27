Display
=======

To display a `Mesh`:

Without any option
------------------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)
    show([m])
    
.. image:: /screenshots/display/normal.png

With triangles
--------------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)
    show([m], options={"display_wire":True})

.. image:: /screenshots/display/wire.png

With transparency
-----------------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)
    show([m], options={"display_faces":False})

.. image:: /screenshots/display/faces.png

With points
-----------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)
    show([m], options={"display_points":True})

.. image:: /screenshots/display/points.png

Color
-----

.. code-block:: python

    from madcad import *

    m = screw(10, 20)
    # m["part"] is a `Mesh`
    m["part"].option(color=vec3(70, 130, 180) / 255) # RGB
    show([m])

.. image:: /screenshots/display/color.png

.. note::
   `screw` returns a `Solid`. `Solid` does not have the method `.option()`.

To get more information, please refer to :ref:`rendering<rendering>`.
