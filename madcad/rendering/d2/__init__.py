import moderngl as mgl

from ...mathutils import uvec2, fvec2, fvec3, fmat2, fmat3, Box
from ..base import Scene



# class View:
#     ''' pure openGL 3D view over a madcad scene '''
#     scene: Scene
#     ''' madcad scene to render, it must contain 3D displays (at least a the root) '''
#     screen: mgl.FrameBuffer
#     ''' color map (and depth) openGL buffer '''
#     ident: mgl.FrameBuffer|None
#     ''' identification map (and depth) openGL buffer '''
#     targets: list
#     ''' exposed for `Scene` '''
#     uniforms: dict
#     ''' exposed for `Scene` '''
#     view: fmat3
#     ''' current view matrix, this will be the default for next rendering '''
#     def render(size=None, view=None)
# 
# class Offscreen(View):
#     color: ndarray
#     depth: ndarray|None
#     ident: ndarray|None
#     def render(size=None, view=None)
# 
# class QView(QWidget, View):
#     navigation: Pan|None
#     toolbar: QWidget|None
#     def control(key, event)
# 
# class Pan:
#     position: vec2
#     scale: float
#     orientation: complex
#     def adjust(box)
#     def center(vec3)
#     def zoom(ratio: float)
#     def pan(offset: vec2)
#     def rotate(offset: float)
#     def matrix() -> mat3

# class Projection(Display, View):
#     ''' 3d view in a 2d scene '''
#     world: mat3
#     size: vec2
#     limits: convex
# 
# class Frame(Display):
#     ''' rectangular frame carying few drawing specifications '''
# 
