"""Displays for mesh data:  cloud points, webs, meshes, ... """

from . import settings
import numpy.core as np
from ...rendering.base import Display, npboundingbox
from ...mathutils import *
from PIL import Image
from ...common import resourcedir
import moderngl as mgl

class MeshDisplay(Display):
    # {{{
    """Display render Meshes"""

    def __init__(self, scene, hatched_part, filled_part, color=None):
        self.box = npboundingbox(filled_part[0])
        self.options = scene.options

        s = settings.display
        color = fvec3(color or s["solid_color"])
        line_color = (
            length(s["line_color"])
            + dot(color - s["solid_color"], s["line_color"] - s["solid_color"])
        ) * normalize(color + 1e-6)

        self.hatched_display = HatchedMeshDisplay(scene, *hatched_part, line_color, color=None)
        self.filled_display = FilledMeshDisplay(scene, *filled_part, line_color, color=None)
        self.vertices = self.filled_display.vertices

    def stack(self, scene):
        yield ((), "screen", -1, self.hatched_display.vertices.prerender)
        yield ((), "screen", -1, self.filled_display.vertices.prerender)
        if self.options["display_faces"]:
            yield ((), "screen", 0, self.hatched_display.disp_faces.render)
            yield ((), "screen", 0, self.filled_display.disp_faces.render)
            yield ((), "ident", 0, self.hatched_display.disp_faces.identify)
            yield ((), "ident", 0, self.filled_display.disp_faces.identify)
        else:
            yield ((), "screen", 1, self.hatched_display.disp_ghost.render)
            yield ((), "screen", 1, self.filled_display.disp_ghost.render)
            yield ((), "ident", 0, self.hatched_display.disp_ghost.identify)
            yield ((), "ident", 0, self.filled_display.disp_ghost.identify)
        if self.options["display_groups"]:
            yield ((), "screen", 1, self.hatched_display.disp_groups.render)
            yield ((), "screen", 1, self.filled_display.disp_groups.render)
        if self.options["display_points"]:
            yield ((), "screen", 2, self.hatched_display.disp_points.render)
            yield ((), "screen", 2, self.filled_display.disp_points.render)
        if self.options["display_wire"]:
            yield ((), "screen", 2, self.hatched_display.disp_wire.render)
            yield ((), "screen", 2, self.filled_display.disp_wire.render)

    @property
    def world(self):
        return self.filled_display.vertices.world

    @world.setter
    def world(self, value):
        for hatched_display in self.hatched_display:
            hatched_display.vertices.world = value
        self.filled_display.vertices.world = value
    # }}}

# Classes Display {{{
class HatchedMeshDisplay(Display):

    def __init__(self, scene, positions, normals, faces, lines, idents, line_color, color=None):
        self.box = npboundingbox(positions)
        self.vertices = Vertices(scene.ctx, positions, idents)
        self.disp_faces = HatchedFacesDisplay(scene, self.vertices, normals, faces, color=color, layer=0)
        self.disp_ghost = GhostDisplay(scene, self.vertices, normals, faces, color=line_color, layer=0)
        self.disp_groups = LinesDisplay(scene, self.vertices, lines, color=line_color, alpha=1, layer=-2e-6)
        self.disp_points = PointsDisplay(scene, self.vertices, range(len(positions)), layer=-3e-6)

        wire = []
        for f in faces:
            wire.append((f[0], f[1]))
            wire.append((f[1], f[2]))
            wire.append((f[2], f[0]))
        self.disp_wire = LinesDisplay(scene, self.vertices, wire, color=line_color, alpha=0.3, layer=-1e-6)


class FilledMeshDisplay(Display):

    def __init__(self, scene, positions, normals, faces, lines, idents, line_color, color=None):
        self.box = npboundingbox(positions)
        self.vertices = Vertices(scene.ctx, positions, idents)
        self.disp_faces = FilledFacesDisplay(scene, self.vertices, normals, faces, color=color, layer=0)
        self.disp_ghost = GhostDisplay(scene, self.vertices, normals, faces, color=line_color, layer=0)
        self.disp_groups = LinesDisplay(scene, self.vertices, lines, color=line_color, alpha=1, layer=-2e-6)
        self.disp_points = PointsDisplay(scene, self.vertices, range(len(positions)), layer=-3e-6)

        wire = []
        for f in faces:
            wire.append((f[0], f[1]))
            wire.append((f[1], f[2]))
            wire.append((f[2], f[0]))
        self.disp_wire = LinesDisplay(scene, self.vertices, wire, color=line_color, alpha=0.3, layer=-1e-6)
# }}}

class WebDisplay(Display):
    # {{{
    """Display to render Webs"""

    def __init__(self, scene, positions, lines, points, idents, color=None):
        self.box = npboundingbox(positions)
        self.options = scene.options
        color = color or settings.display["line_color"]
        self.vertices = Vertices(scene.ctx, positions, idents)
        self.disp_edges = LinesDisplay(
            scene, self.vertices, lines, color=color, alpha=1, layer=-2e-6
        )
        self.disp_groups = PointsDisplay(scene, self.vertices, points, layer=-3e-6)
        self.disp_points = PointsDisplay(
            scene, self.vertices, range(len(positions)), layer=-1e-6
        )

    def stack(self, scene):
        yield ((), "screen", -1, self.vertices.prerender)
        if self.options["display_groups"]:
            yield ((), "screen", 2, self.disp_groups.render)
        if self.options["display_points"]:
            yield ((), "screen", 2, self.disp_points.render)
        yield ((), "screen", 1, self.disp_edges.render)
        yield ((), "ident", 1, self.disp_edges.identify)

    @property
    def world(self):
        return self.vertices.world

    @world.setter
    def world(self, value):
        self.vertices.world = value
    #  }}}

class Vertices(object):
    # {{{
    """Convenient class to share vertices between SolidDisplay, WebDisplay, PointsDisplay"""

    def __init__(self, ctx, positions, idents):
        self.idents = idents
        self.nident = int(max(idents)) + 1
        self.flags = np.zeros(len(positions), dtype="u1")
        self.flags_updated = False
        assert len(idents) == len(positions)
        self.vb_positions = ctx.buffer(
            np.array(positions, dtype="f4", copy=False, order="C")
        )
        self.vb_idents = ctx.buffer(np.array(idents, dtype="u2", copy=False, order="C"))
        self.vb_flags = self.vb_flags = ctx.buffer(self.flags, dynamic=True)
        self.world = fmat4(1)

    def __del__(self):
        self.vb_positions.release()
        self.vb_idents.release()
        self.vb_flags.release()

    def prerender(self, view):
        if self.flags_updated:
            self.vb_flags.write(self.flags)
            self.flags_updated = False

    def selectsub(self, sub):
        for i, id in enumerate(self.idents):
            self.flags[i] ^= id == sub
        self.flags_updated = True
    # }}}

class GhostDisplay:
    # {{{
    def __init__(self, scene, vertices, normals, faces, color, layer=0):
        self.color = color
        self.layer = layer
        self.vertices = vertices

        # load the shader
        def load(scene):
            shader = scene.ctx.program(
                vertex_shader=open(resourcedir + "/shaders/solid.vert").read(),
                fragment_shader=open(resourcedir + "/shaders/ghost.frag").read(),
            )
            # setup some uniforms
            return shader

        self.shader = scene.resource("shader_ghost", load)
        self.ident_shader = scene.resource("shader_subident")
        # allocate buffers
        if faces is not None and len(faces) and vertices.vb_positions:
            self.vb_faces = scene.ctx.buffer(
                np.array(faces, "u4", copy=False, order="C")
            )
            self.vb_normals = scene.ctx.buffer(
                np.array(normals, "f4", copy=False, order="C")
            )
            self.va = scene.ctx.vertex_array(
                self.shader,
                [
                    (vertices.vb_positions, "3f", "v_position"),
                    (self.vb_normals, "3f", "v_normal"),
                    (vertices.vb_flags, "u1", "v_flags"),
                ],
                self.vb_faces,
            )

            self.va_ident = scene.ctx.vertex_array(
                self.ident_shader,
                [
                    (vertices.vb_positions, "3f", "v_position"),
                    (vertices.vb_idents, "u2", "item_ident"),
                ],
                self.vb_faces,
            )
        else:
            self.va = None

    def __del__(self):
        if self.va:
            self.va.release()
            self.va_ident.release()
            self.vb_faces.release()
            self.vb_normals.release()

    def render(self, view):
        if self.va:
            # setup uniforms
            self.shader["select_color"].write(settings.display["select_color_line"])
            self.shader["normal_color"].write(self.color)
            self.shader["world"].write(self.vertices.world)
            self.shader["view"].write(view.uniforms["view"])
            self.shader["proj"].write(view.uniforms["proj"])
            self.shader["layer"] = self.layer
            view.scene.ctx.disable(mgl.DEPTH_TEST)
            # render on self.context
            self.va.render(mgl.TRIANGLES)
            view.scene.ctx.enable(mgl.DEPTH_TEST)

    def identify(self, view):
        if self.va:
            self.ident_shader["start_ident"] = view.identstep(self.vertices.nident)
            self.ident_shader["view"].write(view.uniforms["view"] * self.vertices.world)
            self.ident_shader["proj"].write(view.uniforms["proj"])
            self.ident_shader["layer"] = self.layer
            # render on self.context
            self.va_ident.render(mgl.TRIANGLES)
    # }}}

class LinesDisplay:
    # {{{
    def __init__(self, scene, vertices, lines, color, alpha=1, layer=0):
        self.layer = layer
        self.color = fvec4(fvec3(color), alpha)
        self.select_color = fvec4(settings.display["select_color_line"], alpha)
        self.vertices = vertices

        # load the line shader
        self.shader = scene.resource("shader_wire", shader_wire)
        self.ident_shader = scene.resource("shader_subident")
        if lines is not None and len(lines) and vertices.vb_positions:
            # allocate buffers
            self.vb_lines = scene.ctx.buffer(
                np.array(lines, dtype="u4", copy=False, order="C")
            )
            self.va = scene.ctx.vertex_array(
                self.shader,
                [
                    (vertices.vb_positions, "3f", "v_position"),
                    (vertices.vb_flags, "u1", "v_flags"),
                ],
                self.vb_lines,
            )
            self.va_ident = scene.ctx.vertex_array(
                self.ident_shader,
                [
                    (vertices.vb_positions, "3f", "v_position"),
                    (vertices.vb_idents, "u2", "item_ident"),
                ],
                self.vb_lines,
            )
        else:
            self.va = None

    def __del__(self):
        if self.va:
            self.va.release()
            self.va_ident.release()
            self.vb_lines.release()

    def render(self, view):
        if self.va:
            self.shader["color"].write(self.color)
            self.shader["select_color"].write(self.select_color)
            self.shader["view"].write(view.uniforms["view"] * self.vertices.world)
            self.shader["proj"].write(view.uniforms["proj"])
            self.shader["layer"] = self.layer
            self.va.render(mgl.LINES)

    def identify(self, view):
        if self.va:
            self.ident_shader["start_ident"] = view.identstep(self.vertices.nident)
            self.ident_shader["view"].write(view.uniforms["view"] * self.vertices.world)
            self.ident_shader["proj"].write(view.uniforms["proj"])
            self.ident_shader["layer"] = self.layer
            self.va_ident.render(mgl.LINES)
    # }}}

class PointsDisplay:
    # {{{
    def __init__(self, scene, vertices, indices=None, color=None, ptsize=3, layer=0):
        self.color = fvec4(color or settings.display["point_color"], 1)
        self.select_color = fvec4(settings.display["select_color_line"], 1)
        self.ptsize = ptsize
        self.layer = layer
        self.vertices = vertices

        # load the line shader
        self.shader = scene.resource("shader_wire", shader_wire)
        self.ident_shader = scene.resource("shader_subident")
        # allocate GPU objects
        if indices is not None and len(indices) and vertices.vb_positions:
            self.vb_indices = scene.ctx.buffer(
                np.array(indices, dtype="u4", copy=False, order="C")
            )
            self.va = scene.ctx.vertex_array(
                self.shader,
                [
                    (vertices.vb_positions, "3f", "v_position"),
                    (vertices.vb_flags, "u1", "v_flags"),
                ],
                self.vb_indices,
            )
            self.va_ident = scene.ctx.vertex_array(
                self.ident_shader,
                [
                    (vertices.vb_positions, "3f", "v_position"),
                    (vertices.vb_idents, "u2", "item_ident"),
                ],
                self.vb_indices,
            )
        else:
            self.va = None

    def __del__(self):
        if self.va:
            self.va.release()
            self.va_ident.release()
            self.vb_indices.release()

    def render(self, view):
        if self.va:
            self.shader["layer"] = self.layer
            self.shader["color"].write(self.color)
            self.shader["select_color"].write(self.select_color)
            self.shader["view"].write(view.uniforms["view"] * self.vertices.world)
            self.shader["proj"].write(view.uniforms["proj"])
            view.scene.ctx.point_size = self.ptsize
            self.va.render(mgl.POINTS)

    def identify(self, view):
        if self.va:
            scene.subident_shader["layer"] = self.layer
            scene.subident_shader["start_ident"] = view.identstep(self.vertices.nident)
            scene.subident_shader["view"].write(
                view.uniforms["view"] * self.vertices.world
            )
            scene.subident_shader["proj"].write(view.uniforms["proj"])
            view.ctx.point_size = self.ptsize
            self.va_ident.render(mgl.POINTS)
    # }}}

class HatchedFacesDisplay:
    # {{{
    def __init__(self, scene, vertices, normals, faces, color, layer=0):
        s = settings.display
        self.color = fvec3(color or s["background_color"])
        self.layer = layer
        self.vertices = vertices
        self.va = None

        # load the shader
        def load(scene):
            return scene.ctx.program(
                vertex_shader=open(resourcedir+'/shaders/filled.vert').read(),
                fragment_shader=open(resourcedir+'/shaders/hatched.frag').read(),
            )
        self.shader = scene.resource('shader_hatched', load)
        self.ident_shader = scene.resource('shader_subident')
        # allocate buffers
        if faces is not None and len(faces) and vertices.vb_positions:
            self.vb_faces = scene.ctx.buffer(np.array(faces, 'u4', copy=False, order='C'))
            self.vb_normals = scene.ctx.buffer(np.array(normals, 'f4', copy=False, order='C'))
            self.va = scene.ctx.vertex_array(
                self.shader,
                [
                    (vertices.vb_positions, '3f', 'v_position'), 
                    (vertices.vb_flags, 'u1', 'v_flags')
                ],
                self.vb_faces,
            )
            self.va_ident = scene.ctx.vertex_array(
                self.ident_shader, 
                [
                    (vertices.vb_positions, '3f', 'v_position'),
                    (vertices.vb_idents, 'u2', 'item_ident')
                ],
                self.vb_faces,
            )

    def __del__(self):
        if self.va:
            self.va.release()
            self.va_ident.release()
            self.vb_faces.release()
            self.vb_normals.release()

    def render(self, view):
        if self.va:
            # setup uniforms
            # self.shader['select_color'].write(settings.display['select_color_face'])
            self.shader['user_color'].write(self.color)
            self.shader["layer"] = self.layer
            self.shader["world"].write(self.vertices.world)
            self.shader["view"].write(view.uniforms["view"])
            self.shader["proj"].write(view.uniforms["proj"])
            # render on self.context
            self.va.render(mgl.TRIANGLES)

    def identify(self, view):
        if self.va:
            self.ident_shader["layer"] = self.layer
            self.ident_shader["start_ident"] = view.identstep(self.vertices.nident)
            self.ident_shader["view"].write(view.uniforms["view"] * self.vertices.world)
            self.ident_shader["proj"].write(view.uniforms["proj"])
            # render on self.context
            self.va_ident.render(mgl.TRIANGLES)
    # }}}

class FilledFacesDisplay:
    # {{{
    def __init__(self, scene, vertices, normals, faces, color, layer=0):
        s = settings.display
        self.color = fvec3(color or s["background_color"])
        self.layer = layer
        self.vertices = vertices
        self.va = None

        # load the shader
        def load(scene):
            return scene.ctx.program(
                vertex_shader=open(resourcedir+'/shaders/filled.vert').read(),
                fragment_shader=open(resourcedir+'/shaders/filled.frag').read(),
            )
        self.shader = scene.resource('shader_filled', load)
        self.ident_shader = scene.resource('shader_subident')
        # allocate buffers
        if faces is not None and len(faces) and vertices.vb_positions:
            self.vb_faces = scene.ctx.buffer(np.array(faces, 'u4', copy=False, order='C'))
            self.vb_normals = scene.ctx.buffer(np.array(normals, 'f4', copy=False, order='C'))
            self.va = scene.ctx.vertex_array(
                self.shader,
                [
                    (vertices.vb_positions, '3f', 'v_position'), 
                    (vertices.vb_flags, 'u1', 'v_flags')
                ],
                self.vb_faces,
            )
            self.va_ident = scene.ctx.vertex_array(
                self.ident_shader, 
                [
                    (vertices.vb_positions, '3f', 'v_position'),
                    (vertices.vb_idents, 'u2', 'item_ident')
                ],
                self.vb_faces,
            )

    def __del__(self):
        if self.va:
            self.va.release()
            self.va_ident.release()
            self.vb_faces.release()
            self.vb_normals.release()

    def render(self, view):
        if self.va:
            # setup uniforms
            # self.shader['select_color'].write(settings.display['select_color_face'])
            self.shader['user_color'].write(self.color)
            self.shader["layer"] = self.layer
            self.shader["world"].write(self.vertices.world)
            self.shader["view"].write(view.uniforms["view"])
            self.shader["proj"].write(view.uniforms["proj"])
            # render on self.context
            self.va.render(mgl.TRIANGLES)

    def identify(self, view):
        if self.va:
            self.ident_shader["layer"] = self.layer
            self.ident_shader["start_ident"] = view.identstep(self.vertices.nident)
            self.ident_shader["view"].write(view.uniforms["view"] * self.vertices.world)
            self.ident_shader["proj"].write(view.uniforms["proj"])
            # render on self.context
            self.va_ident.render(mgl.TRIANGLES)
    # }}}


def shader_wire(scene):
    return scene.ctx.program(
        vertex_shader=open(resourcedir + "/shaders/wire.vert").read(),
        fragment_shader=open(resourcedir + "/shaders/wire.frag").read(),
    )


def shader_uniformcolor(scene):
    return scene.ctx.program(
        vertex_shader=open(resourcedir + "/shaders/uniformcolor.vert").read(),
        fragment_shader=open(resourcedir + "/shaders/uniformcolor.frag").read(),
    )
