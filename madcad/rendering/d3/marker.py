"""Displays for visual markers: points, arrows, texts, ... """

from ..base import Display
from ...mathutils import *

class AnnotationDisplay(Display):
    def __init__(self, scene, points, color):
        self.color = fvec3(color or settings.display['annotation_color'])
        self.selected = False
        self.box = npboundingbox(points)
        # load shader
        def load(scene):
            return scene.ctx.program(
                        vertex_shader=open(resourcedir+'/shaders/annotation.vert').read(),
                        fragment_shader=open(resourcedir+'/shaders/annotation.frag').read(),
                        )
        self.shader = scene.resource('shader_annotation', load)
        # allocate buffers
        self.vb_pts = scene.ctx.buffer(points)
        self.va = scene.ctx.vertex_array(self.shader, [(self.vb_pts, '3f f', 'v_position', 'v_alpha')])
        self.va_ident = scene.ctx.vertex_array(scene.resource('shader_ident'), [(self.vb_pts, '3f 4x', 'v_position')])

    def __del__(self):
        self.va.release()
        self.va_ident.release()
        self.vb_pts.release()

    @staticmethod
    def buff_ptsalpha(points, alpha):
        return np.hstack((
                np.array([tuple(p) for p in points], 'f4'), 
                np.array(alpha, 'f4', ndmin=2).transpose(),
                ))

    def render(self, view):
        self.shader['proj'].write(view.uniforms['proj'])
        self.shader['view'].write(view.uniforms['view'] * self.world)
        self.shader['color'].write(self.color if not self.selected else fvec3(settings.display['select_color_line']))
        self.va.render(mgl.LINES)

    def identify(self, view):
        shader = view.scene.resource('shader_ident')
        shader['proj'].write(view.uniforms['proj'])
        shader['view'].write(view.uniforms['view'] * self.world)
        shader['ident'] = view.identstep(1)
        self.va_ident.render(mgl.LINES)

    def stack(self, scene):
        return ( ((), 'ident', 2, self.identify),
                 ((), 'screen', 2, self.render)) 

class PointDisplay(Display):
    def __init__(self, scene, position, size=10, color=None):
        self.position = fvec3(position)
        self.size = size
        self.selected = False
        self.color = fvec3(color or settings.display['line_color'])

        def load(scene):
            img = Image.open(resourcedir+'/textures/point.png')
            texture = scene.ctx.texture(img.size, 1, img.convert('L').tobytes())
            #self.texture = scene.resource('pointtex', load)
            shader = scene.ctx.program(
                        vertex_shader=open(resourcedir+'/shaders/pointhalo.vert').read(),
                        fragment_shader=open(resourcedir+'/shaders/pointhalo.frag').read(),
                        )
            shader['halotex'].value = 0
            ident_shader = scene.ctx.program(
                        vertex_shader=open(resourcedir+'/shaders/pointhalo-ident.vert').read(),
                        fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
                        )
            #self.shader = scene.resource('pointhalo', load)
            vb = scene.ctx.buffer(np.array([(0,0), (0,1), (1,1), (0,0), (1,1), (1,0)], 'f4'))
            va = scene.ctx.vertex_array(shader, [(vb, '2f', 'v_uv')])
            va_ident = scene.ctx.vertex_array(ident_shader, [(vb, '2f', 'v_uv')])
            return texture, shader, va, ident_shader, va_ident

        (   self.texture,
            self.shader,
            self.va,
            self.ident_shader,
            self.va_ident     ) = scene.resource('pointhalo', load)

    @property
    def box(self):
        return Box(center=self.position, width=fvec3(0))

    def render(self, view):
        self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
        self.shader['position'].write(fvec3(self.world * fvec4(self.position,1)))
        self.shader['view'].write(view.uniforms['view'])
        self.shader['proj'].write(view.uniforms['proj'])
        self.shader['ratio'] = (
                self.size / view.width(),
                self.size / view.height(),
                )
        self.texture.use(0)
        self.va.render(mgl.TRIANGLES)

    def identify(self, view):
        self.ident_shader['ident'] = view.identstep(1)
        self.ident_shader['position'].write(fvec3(self.world * fvec4(self.position,1)))
        self.ident_shader['view'].write(view.uniforms['view'])
        self.ident_shader['proj'].write(view.uniforms['proj'])
        self.ident_shader['ratio'] = (
                1.5 * self.size / view.width(),
                1.5 * self.size / view.height(),
                )
        self.va_ident.render(mgl.TRIANGLES)

    def stack(self, scene):
        return (((), 'ident', 2, self.identify),
                ((), 'screen', 2, self.render))


class TextDisplay:
    pass


class AxisDisplay(Display):
    pattern = [0, 0.25, 0.45, 0.55, 0.75, 1]
    repetitions = 3
    def __init__(self, scene, axis, interval=None, color=None, pose=fmat4(1)):
        self.origin = fvec3(axis[0])
        self.direction = fvec3(axis[1])
        self.interval = interval
        self.color = fvec3(color or settings.display['line_color'])
        self.selected = False
        self.box = Box(center=self.origin, width=fvec3(0))

        self.shader, self.va, self.ident_shader, self.va_ident = scene.resource('axis', self.load)

    def load(self, scene):
        shader = scene.ctx.program(
                    vertex_shader=open(resourcedir+'/shaders/axis.vert').read(),
                    fragment_shader=open(resourcedir+'/shaders/axis.frag').read(),
                    )
        ident_shader = scene.ctx.program(
                    vertex_shader=open(resourcedir+'/shaders/axis-ident.vert').read(),
                    fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
                    )
        pts = []
        for i in range(-1, self.repetitions+1):
            for pt in self.pattern:
                if i == -1:                    alpha = pt
                elif i == self.repetitions:    alpha = 1-pt
                else:                        alpha = 1
                pts.append(((pt+i)/self.repetitions, alpha))
        vb = scene.ctx.buffer(np.array(pts, 'f4'))
        va = scene.ctx.vertex_array(shader, [(vb, 'f f', 'v_absciss', 'v_alpha')])
        va_ident = scene.ctx.vertex_array(ident_shader, [(vb, 'f 12x', 'v_absciss')])
        return shader, va, ident_shader, va_ident

    def _disp_interval(self, view):
        if self.interval:    return self.interval
        else:
            size = -view.uniforms['view'][3][2]/6
            return (-0.5*size, size)

    def render(self, view):
        self.shader['projview'].write(view.uniforms['projview'])
        self.shader['world'].write(self.world)
        self.shader['origin'].write(self.origin)
        self.shader['direction'].write(self.direction)
        self.shader['interval'] = self._disp_interval(view)
        self.shader['color'].write(fvec3(settings.display['select_color_line']) if self.selected else self.color)
        self.va.render(mgl.LINES)

    def identify(self, view):
        self.ident_shader['projview'].write(view.uniforms['projview'])
        self.ident_shader['world'].write(self.world)
        self.ident_shader['origin'].write(self.origin)
        self.ident_shader['direction'].write(self.direction)
        self.ident_shader['interval'] = self._disp_interval(view)
        self.ident_shader['ident'] = view.identstep(1)
        self.va_ident.render(mgl.LINES)

    def stack(self, scene):
        return (((), 'ident', 2, self.identify),
                ((), 'screen', 2, self.render))


class BoxDisplay(AnnotationDisplay):
    def __init__(self, scene, box, color=None):
        # place points
        x,y,z = box.width
        c = 0.1*min(x,y,z)    # corner
        o = 0.4 # wire alpha
        pts = np.array([
            # corner
            (0,0,0,1),(c,0,0,1),
            (0,0,0,1),(0,c,0,1),
            (0,0,0,1),(0,0,2*c,1),

            # box wire
            (0,0,0,o),(x,0,0,o),
            (0,0,0,o),(0,y,0,o),
            (0,0,0,o),(0,0,z,o),

            (x,0,0,o),(x,y,0,o),
            (x,0,0,o),(x,0,z,o),
            (0,y,0,o),(x,y,0,o),
            (0,y,0,o),(0,y,z,o),
            (0,0,z,o),(x,0,z,o),
            (0,0,z,o),(0,y,z,o),

            (x,y,0,o),(x,y,z,o),
            (x,0,z,o),(x,y,z,o),
            (0,y,z,o),(x,y,z,o),
            ], dtype='f4')
        pts += (*box.min, 0)
        self.textplace = box.min

        super().__init__(scene, pts, color)


class SplineDisplay(Display):
    ''' display for spline curve, with handles around'''
    def __init__(self, scene, handles, curve, color=None):
        self.color = color or fvec4(settings.display['line_color'], 1)
        self.color_handles = fvec4(settings.display['annotation_color'], 0.6)
        self.box = npboundingbox(handles)
        ctx = scene.ctx
        self.vb_handles = ctx.buffer(handles)
        self.vb_curve = ctx.buffer(curve)

        self.shader = scene.resource('shader_uniformcolor', shader_uniformcolor)
        self.va_handles = ctx.vertex_array(self.shader, [(self.vb_handles, '3f4', 'v_position')])
        self.va_curve = ctx.vertex_array(self.shader, [(self.vb_curve, '3f4', 'v_position')])

        self.shader_ident = scene.resource('shader_ident')
        self.va_ident = ctx.vertex_array(self.shader_ident, [(self.vb_curve, '3f4', 'v_position')])

    def __del__(self):
        self.va_handles.release()
        self.va_curve.release()
        self.va_ident.release()
        self.vb_handles.release()
        self.vb_curve.release()

    def render(self, view):
        self.shader['view'].write(view.uniforms['view'] * self.world)
        self.shader['proj'].write(view.uniforms['proj'])
        view.scene.ctx.point_size = 4

        self.shader['layer'] = -2e-6
        self.shader['color'].write(self.color_handles if not self.selected else fvec4(settings.display['select_color_line'],self.color_handles[3]))
        self.va_handles.render(mgl.POINTS)
        self.va_handles.render(mgl.LINE_STRIP)

        self.shader['layer'] = -1e-6
        self.shader['color'].write(self.color if not self.selected else fvec4(settings.display['select_color_line'],self.color[3]))
        self.va_curve.render(mgl.LINE_STRIP)

    def identify(self, view):
        self.shader_ident['ident'] = view.identstep(1)
        self.shader_ident['view'].write(view.uniforms['view'] * self.world)
        self.shader_ident['proj'].write(view.uniforms['proj'])
        self.va_ident.render(mgl.LINE_STRIP)

    def stack(self, scene):
        return (    ((), 'screen', 1, self.render),
                    ((), 'ident', 1, self.identify)    )
