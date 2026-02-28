# mathutils -- All the basic math types and functions

Most of the names here are coming from the [glm](http://github.com/Zuzu-Typ/PyGLM) module.
But the goal is to harvest at one point all the basic math functions and objects that are used all around madcad.

!!! tip
    All the present functions and types are present in the root madcad module.

## Most common glm types

All following objects implement the common operators `+ - * / <> ==`

::: madcad.mathutils.vec3
    options:
      members: false
::: madcad.mathutils.mat3
    options:
      members: false
::: madcad.mathutils.mat4
    options:
      members: false
::: madcad.mathutils.quat
    options:
      members: false

All `glm` types exists with several element types and in several precision:

| Prefix | Precision |
|--------|-----------|
| **d** | f64 aka double precision floating point |
| **f** | f32 aka simple precision floating point |
| **i** | i32 aka integer |
| **i16** | 16 bits integer |
| **i8** | 8 bits integer aka byte |
| **u** | unsigned integer (also declines in u16 and u32) |
| **b** | bit aka boolean |

- Precision specification is put as prefix: `dvec3, fvec3, imat4`. Notation without prefix refers to the madcad implementation precision: **float64** (prefix 'd').
- Object dimension is put as suffix.

In this documentation, when we refer to a 'vector' without explicit type, we obviously mean a `vec3` aka. `dvec3`.

!!! note
    The default glm_ precision type is **float32** (prefix 'f'). For convenience, these are overriden in madcad to use **float64** for a better precision.

## Common vector operations

::: madcad.mathutils.dot
::: madcad.mathutils.cross
::: madcad.mathutils.length
::: madcad.mathutils.distance
::: madcad.mathutils.anglebt
::: madcad.mathutils.arclength
::: madcad.mathutils.project
::: madcad.mathutils.noproject
::: madcad.mathutils.unproject
::: madcad.mathutils.reflect
::: madcad.mathutils.perp

See [the glm complete reference](https://github.com/Zuzu-Typ/PyGLM/blob/master/wiki/function-reference/README.md)

## Transformations

::: madcad.mathutils.transform
::: madcad.mathutils.translate
::: madcad.mathutils.rotate
::: madcad.mathutils.scaledir
::: madcad.mathutils.scale
::: madcad.mathutils.rotatearound
::: madcad.mathutils.dirbase
::: madcad.mathutils.transpose
::: madcad.mathutils.inverse
::: madcad.mathutils.affineInverse
::: madcad.mathutils.angle
::: madcad.mathutils.axis
::: madcad.mathutils.angleAxis

## Interpolation functions

::: madcad.mathutils.mix
::: madcad.mathutils.hermite
::: madcad.mathutils.step
::: madcad.mathutils.linstep
::: madcad.mathutils.smoothstep
::: madcad.mathutils.clamp
::: madcad.mathutils.lerp
::: madcad.mathutils.slerp
::: madcad.mathutils.linrange
::: madcad.mathutils.intri_smooth
::: madcad.mathutils.intri_sphere
::: madcad.mathutils.intri_parabolic

## Distances

::: madcad.mathutils.distance_pa
::: madcad.mathutils.distance_pe
::: madcad.mathutils.distance_aa
::: madcad.mathutils.distance_ae

## Locally defined data types

::: madcad.mathutils.Axis
::: madcad.mathutils.isaxis
::: madcad.mathutils.Box
::: madcad.mathutils.Screw
::: madcad.mathutils.comoment
::: madcad.mathutils.skew
::: madcad.mathutils.unskew
