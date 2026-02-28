# primitives -- 3D Primitives for Wire generation

::: madcad.primitives
    options:
      show_root_heading: false
      members: false

Primitives are parametrized objects, that can be baked into a mesh/web/wire object. A primitive object must have the following signature:

```python
class SomePrimitive:
    # method baking the primitive in some general-purpose 3D object
    # resolution is optional and defaults to the primitive settings
    # that defaults to the current settings at call time
    def mesh(self, resolution=None) -> Mesh/Web/Wire:
        ...

    # for the solver
    # primitive attributes the solver has to consider as variables or variable container
    slvvars = 'fields', 'for', 'solver', 'variables'
    # optional method constraining the primitive parameters (to keep points on a circle for instance)
    def fit(self) -> err**2 as float:
        ...
```

::: madcad.primitives.isprimitive

## Curve resolution

Some primitive types are curves, the discretization is important for visual as well as for result quality (remember that even if something looks like a perfect curves, it's still polygons).
The resolution (subdivision) of curve is done following the following criterion present in the [settings module](settings.md)

Specification priority order:

1. Optional argument `resolution` passed to `primitive.mesh()` or to `web()` or `wire()`
2. Optional attribute `resolution` of the primitive object
3. Value of `settings.resolution` at bake time.

Specification format:

```python
('fixed', 16)   # fixed amount of 16 subdivisions
('rad', 0.6)    # max polygon angle is 0.6 rad
('radm', 0.6)
('radm2', 0.6)
```

## Primitives types

::: madcad.primitives.Segment
::: madcad.primitives.Circle
::: madcad.primitives.ArcCentered
::: madcad.primitives.ArcThrough
::: madcad.primitives.ArcTangent
::: madcad.primitives.Ellipsis
::: madcad.primitives.TangentEllipsis
::: madcad.primitives.Interpolated
::: madcad.primitives.Softened
