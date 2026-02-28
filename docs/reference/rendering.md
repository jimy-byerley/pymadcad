# rendering -- 3D user interface

This module provides a render pipeline system featuring:

- Class `Scene` to gather the data to render
- Widget `View` that actually renders the scene
- The display protocol, that allows any object to define its `Display` subclass to be rendered in a scene.

The view is for window integration and user interaction. `Scene` is only to manage the objects to render. Almost all madcad data types can be rendered to scenes being converted into an appropriate subclass of `Display`. Since the conversion from madcad data types into display instance is automatically handled via the *display protocol*, you usually don't need to deal with displays directly.

::: madcad.rendering.show

## Display protocol

A displayable is an object that implements the signature of `Display`.

::: madcad.rendering.Display
    options:
      members:
        - display
        - stack
        - __getitem__
        - update
        - control

## Rendering system

!!! note
    As the GPU native precision is *f4* (float 32 bits), all the vector stuff regarding rendering is made using simple precision types: `fvec3, fvec4, fmat3, fmat4, ...` instead of the usual double precision `vec3`

::: madcad.rendering.Scene

## Views classes

::: madcad.rendering.GLView3D

::: madcad.rendering.Offscreen3D

## View projections and navigation

::: madcad.rendering.Turntable
::: madcad.rendering.Orbit
::: madcad.rendering.Perspective
::: madcad.rendering.Orthographic

## Helpers to trick into the pipeline

::: madcad.rendering.Group
    options:
      members:
        - stack
        - __getitem__
        - dequeue
        - update
        - local
        - world
        - box

::: madcad.rendering.Step
::: madcad.rendering.Displayable
