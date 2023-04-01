"""Displays for mechanical workflow """

from ..base import Display

class SolidDisplay(Display):
    ''' Display render Meshes '''
    def __init__(self, scene, positions, normals, faces, lines, idents, color=None):
        self.box = npboundingbox(positions)
        self.options = scene.options
        
        s = settings.display
        color = fvec3(color or s['solid_color'])
        line = (    (length(s['line_color']) + dot(color-s['solid_color'], s['line_color']-s['solid_color']))
                    * normalize(color + 1e-6)  )
        #if length(s['line_color']) > length(color)
        reflect = normalize(color + 1e-6) * s['solid_reflectivity']
        
        self.vertices = Vertices(scene.ctx, positions, idents)
        self.disp_faces = FacesDisplay(scene, self.vertices, normals, faces, color=color, reflect=reflect, layer=0)
        self.disp_ghost = GhostDisplay(scene, self.vertices, normals, faces, color=line, layer=0)
        self.disp_groups = LinesDisplay(scene, self.vertices, lines, color=line, alpha=1, layer=-2e-6)
        self.disp_points = PointsDisplay(scene, self.vertices, range(len(positions)), layer=-3e-6)
        wire = []
        for f in faces:
            wire.append((f[0], f[1]))
            wire.append((f[1], f[2]))
            wire.append((f[2], f[0]))
        self.disp_wire = LinesDisplay(scene, self.vertices, wire, color=line, alpha=0.3, layer=-1e-6)
        
    def stack(self, scene):
        yield ((), 'screen', -1, self.vertices.prerender)
        if self.options['display_faces']:    
            yield ((), 'screen', 0, self.disp_faces.render)
            yield ((), 'ident', 0, self.disp_faces.identify)
        else:                                
            yield ((), 'screen', 1, self.disp_ghost.render)
            yield ((), 'ident', 0, self.disp_ghost.identify)
        if self.options['display_groups']:    yield ((), 'screen', 1, self.disp_groups.render)
        if self.options['display_points']:    yield ((), 'screen', 2, self.disp_points.render)
        if self.options['display_wire']:    yield ((), 'screen', 2, self.disp_wire.render)
    
    @property
    def world(self):    return self.vertices.world
    @world.setter
    def world(self, value):    self.vertices.world = value

    # def control(self, view, key, sub, evt):
    #     if evt.type() == QEvent.MouseButtonRelease and evt.button() == Qt.LeftButton:
    #         sub = sub[0]
    #         flags, idents = self.vertices.flags, self.vertices.idents
    #         for i in range(len(idents)):
    #             flags[i] ^= idents[i] == sub
    #         self.vertices.flags_updated = True
    #         view.update()
    #         evt.accept()


class Kinemanip:
    pass

class Dispatcher(object):
    ''' Iterable object that holds a generator built by passing self as first argument
        it allows the generator code to dispatch references to self.
        NOTE:  at contrary to current generators, the code before the first yield is called at initialization
    '''
    __slots__ = 'generator', 'value'
    def __init__(self, func=None, *args, **kwargs):
        self.func = func
        self.generator = self._run(func, *args, **kwargs)
        # run the generator until the first yield
        next(self.generator, None)
    def _run(self, func, *args, **kwargs):
        self.value = yield from func(self, *args, **kwargs)
        
    def __repr__(self):
        return '<{} on {}>'.format(type(self).__name__, self.func)
        
    def send(self, value):    return self.generator.send(value)
    def __iter__(self):        return self.generator
    def __next__(self):        return next(self.generator)

class Tool(Dispatcher):
    ''' Generator wrapping an yielding function, that unregisters from view.tool once the generator is over '''
    def _run(self, func, *args, **kwargs):
        try:    
            self.value = yield from func(self, *args, **kwargs)
        except StopTool:
            pass
        try:    
            args[0].tool.remove(self)
        except ValueError:    
            pass
    
    def __call__(self, evt):
        try:    return self.send(evt)
        except StopIteration:    pass
        
    def stop(self):
        if self.generator:
            try:    self.generator.throw(StopTool())
            except StopTool:    pass
            except StopIteration:    pass
            self.generator = None
    def __del__(self):
        self.stop()
    
class StopTool(Exception):
    ''' Used to stop a tool execution '''
    pass
