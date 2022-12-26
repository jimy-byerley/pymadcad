.. _display:

Display
=======

To display a `Mesh`:

Without any option
------------------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)["part"]
    show([m])
    
.. image:: /images/display/normal.png

With triangles
--------------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)["part"]
    show([m], options={"display_wire":True})

.. image:: /images/display/wire.png

With transparency
-----------------

.. code-block:: python

    from madcad import *

    m = screw(10, 20)["part"]
    show([m], options={"display_faces":False})

.. image:: /images/display/faces.png


Color
-----

.. code-block:: python

    from madcad import *

    m = screw(10, 20)["part"]
    m.option(color=vec3(70, 130, 180) / 255) # RGB
    show([m])

.. image:: /images/display/color.png

.. note::
   `screw` returns a `Solid`. `Solid` does not have the method `.option()` but other options work on `Solid` also.

To get more information, please refer to :ref:`rendering<rendering>`.
