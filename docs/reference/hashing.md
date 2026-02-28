# hashing -- Fast access to space associated data

::: madcad.hashing
    options:
      show_root_heading: false
      members: false

## Connectivity

::: madcad.hashing.edgekey
::: madcad.hashing.facekeyo
::: madcad.hashing.arrangeface
::: madcad.hashing.arrangeedge
::: madcad.hashing.connpp
::: madcad.hashing.connef
::: madcad.hashing.connpe
::: madcad.hashing.connexity
::: madcad.hashing.suites

## Specific Hashmaps

::: madcad.hashing.PositionMap
    options:
      members:
        - keysfor
        - update
        - add
        - get
        - display
        - __contains__

::: madcad.hashing.meshcellsize

::: madcad.hashing.PointSet
    options:
      members:
        - keyfor
        - update
        - difference_update
        - add
        - remove
        - discard
        - contains
        - __getitem__
        - __contains__
        - __add__
        - __sub__
        - __iadd__
        - __isub__

::: madcad.hashing.Asso
    options:
      members:
        - __getitem__
        - __contains__
        - add
        - remove
        - discard
        - update
        - __add__
        - clear
        - items
        - keys
        - values
        - connexity
