# assembly -- Functions to group and move together 3D objects

::: madcad.assembly
    options:
      show_root_heading: false
      members: false

::: madcad.assembly.Solid
    options:
      members:
        - pose
        - transform
        - place
        - loc
        - deloc
        - set
        - append
        - __getitem__
        - __setitem__
        - display

::: madcad.assembly.placement
::: madcad.assembly.explode
::: madcad.assembly.explode_offsets
