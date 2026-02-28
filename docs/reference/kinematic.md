# kinematic -- Kinematic solver/constraint system

::: madcad.kinematic
    options:
      show_root_heading: false
      members: false

::: madcad.kinematic.KinematicError

::: madcad.kinematic.Kinematic
    options:
      members:
        - normalize
        - direct
        - inverse
        - grad
        - cycles
        - solve
        - freedom
        - parts
        - display
        - to_chain

::: madcad.kinematic.Joint
    options:
      members:
        - normalize
        - direct
        - inverse
        - grad
        - scheme
        - display

::: madcad.kinematic.Weld
::: madcad.kinematic.Free
::: madcad.kinematic.Reverse

::: madcad.kinematic.Chain
    options:
      members:
        - normalize
        - direct
        - inverse
        - grad
        - parts
        - to_kinematic
        - to_dh
        - from_dh
        - display

::: madcad.kinematic.arcs
::: madcad.kinematic.depthfirst
::: madcad.kinematic.cycles
::: madcad.kinematic.shortcycles

::: madcad.kinematic.displays
    options:
      show_root_heading: true
