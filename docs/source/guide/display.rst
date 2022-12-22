Display
=======

To display a `Mesh`:

Without any option
------------------

.. code-block:: python

    from madcad import *

    mesh = screw(10, 20)
    show([mesh])
    
.. image:: /images/display/normal.png

With triangles
--------------

.. code-block:: python

    from madcad import *

    mesh = screw(10, 20)
    show([mesh], options={"display_wire":True})

.. image:: /images/display/wire.png

With transparency
-----------------

.. code-block:: python

    from madcad import *

    mesh = screw(10, 20)
    show([mesh], options={"display_faces":False})

.. image:: /images/display/faces.png


Color
-----

.. code-block:: python

    from madcad import *

    mesh = screw(10, 20)
    mesh.option(color=vec3(70, 130, 180) / 255) # RGB
    show([mesh])

.. image:: /images/display/color.png

To get more information, please refer to :ref:`rendering<rendering>`.
