.. _settings:

settings    - Global and default settings
=========================================

Quick configuration
-------------------

You can customize `madcad` with settings.
You simply start to write a configuration file, then `madcad` does the rest for you.

.. code-block:: python
   
    import madcad
    madcad.settings.install() # it will create a file in ~/.config/madcad/pymadcad.yaml


Now, you can edit the file `~/.config/madcad/pymadcad.yaml`:

.. code-block:: yaml

    controls: {navigation: Turntable, snap_dist: 10}
    display:
        annotation_color: [0.2, 0.7, 1.0]
        background_color: [0.0, 0.0, 0.0]
        field_of_view: 0.5235987755982988
        highlight_color: [0.1, 0.2, 0.2]
        line_color: [0.9, 0.9, 0.9]
        line_width: 1.0
        point_color: [0.9, 0.9, 0.9]
        schematics_color: [0.3, 0.8, 1.0]
        select_color_face: [0.01, 0.05, 0.03]
        select_color_line: [0.5, 1.0, 0.6]
        sharp_angle: 0.5235987755982988
        solid_color: [0.2, 0.2, 0.2]
        solid_color_front: 1.0
        solid_color_side: 0.2
        solid_reflect: skybox-white.png
        solid_reflectivity: 6
        solver_error_color: [1.0, 0.3, 0.2]
        system_theme: true
        view_font_size: 8
    primitives:
        curve_resolution: [rad, 0.19634954084936207]
    scene: {debug_faces: false, debug_groups: false,
        debug_points: false, display_annotations: true,
        display_faces: true, display_grid: true,
        display_groups: true, display_points: false,
        display_wire: false, lock_solids: true,
        projection: Perspective, surface_shading: true}


.. note::
   Values of colors are going from 0 to 1. 

If you want, you can change this configuration folder by using :

.. code-block:: python
    
    import madcad
    myfile = open("myfile.yaml", "w")
    madcad.settings.dump(myfile)


Then, in your script, load it :


.. code-block:: python
    
    from madcad import *
    
    myfile = open("myfile.yaml")
    settings.load(myfile)
    mesh = screw(10, 20)
    show([mesh])

Module
------

.. automodule:: madcad.settings
	
	All the settings presents here are loaded at start or on demand from `~/.config/madcad/pymadcad.json`
	
	.. autofunction:: madcad.settings.load
	.. autofunction:: madcad.settings.dump
	.. autofunction:: madcad.settings.install
	.. autofunction:: madcad.settings.clean
	
	.. autofunction:: madcad.settings.use_qt_colors
	.. autofunction:: madcad.settings.curve_resolution

	
