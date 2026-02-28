# constraints -- Geometric constraints on primitives

::: madcad.constraints
    options:
      show_root_heading: false
      members: false

This modules defines the constraints definitions and the solver tools

Constraints can be any object referencing `variables` and implementing the following signature to guide the solver in the resolution:

```python
class SomeConstraint:

    # name the attributes referencing the solver variables to change
    slvvars = 'some_primitive', 'some_point'
    # function returning the squared error in the constraint

    #  for a coincident constraint for instance it's the squared distance
    #  the error must be a contiguous function of the parameters, and it's squared for numeric stability reasons.
    #  the solver internally use an iterative approach to optimize the sum of all fit functions.
    def fit(self):
        ...
```

::: madcad.constraints.isconstraint

## Solver

::: madcad.constraints.solve

::: madcad.constraints.Problem

## Constraints definitions

::: madcad.constraints.Distance
::: madcad.constraints.Radius
::: madcad.constraints.Angle
::: madcad.constraints.Tangent
::: madcad.constraints.OnPlane
