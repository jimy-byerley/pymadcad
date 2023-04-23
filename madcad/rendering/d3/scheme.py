"""Displays for annotations: labels, markups, sizing, ... """

class Scheme:
   ''' An object containing schematics. 
    
        This is a buffer object, it isnot intended to be useful to modify a scheme.
        
        Attributes:
        
            spaces (list):       a space is any function giving a mat4 to position a point on the screen (OpenGL convensions as used)
            
            vertices (list):
            
                a vertex is a tuple
                
                `(space id, position, normal, color, layer, track, flags)`
            
            primitives (list):   
            
                list of buffers (of point indices, edges, triangles, depending on the exact primitive type), associated to each supported shader in the scheme
                
                currently supported shaders are:
                
                - `'line'`  uniform opaque/transparent lines
                - `'fill'`  uniform opaque/transparent triangles
                - `'ghost'` triangles of surface that fade when its normal is close to the view
            
            components (list):   objects to display setting their local space to one of the spaces
            
            current (dict):     last vertex definition, implicitly reused for convenience
    '''
    def __init__(self, vertices=None, spaces=None, primitives=None, **kwargs):
        self.vertices = vertices or [] # list of vertices
        self.spaces = spaces or []    # definition of each space
        self.primitives = primitives or {} # list of indices for each shader
        self.components = []    # displayables associated to spaces
        # for creation: last vertex inserted
        self.current = {'color':fvec4(settings.display['annotation_color'],1), 'flags':0, 'layer':0, 'space':world, 'shader':'wire', 'track':0, 'normal':fvec3(0)}
        self.set(**kwargs)
        
    def __iadd__(self, other):
        ''' Concatenate the content of an other scheme in the current one
            
            The current settings stay as before concatenation
        '''
        ls = len(self.spaces)
        lv = len(self.vertices)
        lt = max(self.tracks)+1
        self.spaces.extend(other.spaces)
        self.components.extend(other.components)
        self.vertices.extend([
                            v[0] + ls, 
                            *v[1:5],
                            v[5] + lt, 
                            v[6],
                        ]  for v in other.vertices)
        for shader, prims in other.primitives:
            if shader not in self.primitives:
                self.primitives[shader] = []
            self.primitives[shader].extend(tuple(i+lv  for i in p) for p in prims)
        
        return self
        
    def __add__(self, other):
        ''' Return the union of the two schemes.
        
            The current settings are those from first scheme
        '''
        r = deepcopy(self)
        r += other
        return r
        
    def set(self, *args, **kwargs):
        ''' Change the specified attributes in the current default vertex definition '''
        if args:
            if len(args) == 1 and isinstance(args[0], dict):
                kwargs = args[0]
            else:
                raise TypeError('Scheme.set expects keywords argument or one unique dictionnary argument')
        self.current.update(kwargs)
        # register the space if not already known
        if not isinstance(self.current['space'], int):
            try:    i = self.spaces.index(self.current['space'])
            except ValueError:    
                i = len(self.spaces)
                self.spaces.append(self.current['space'])
            self.current['space'] = i
        if not isinstance(self.current['color'], fvec4):
            self.current['color'] = fvec4(self.current['color'])
        
        return self
    
    def add(self, obj, **kwargs):
        ''' Add an object to the scheme.
            If it is a mesh, it's merged in the current buffers. 
            Else it is added as a component to the current space.
        '''
        self.set(kwargs)
        if self.current['shader'] not in self.primitives:
            self.primitives[self.current['shader']] = indices = []
        else:
            indices = self.primitives[self.current['shader']]
        l = len(self.vertices)
        
        if isinstance(obj, (Mesh,Web)):
            self.vertices.extend([
                                self.current['space'], 
                                fvec3(p), 
                                self.current['normal'], 
                                self.current['color'], 
                                self.current['layer'], 
                                self.current['track'], 
                                self.current['flags'],
                            ]  for p in obj.points)
        if isinstance(obj, Mesh):
            indices.extend(((a+l, b+l, c+l)  for a,b,c in obj.faces))
            for f, track in zip(obj.faces, obj.tracks):
                for p in f:
                    self.vertices[p+l][5] = track
            for i,n in enumerate(obj.vertexnormals()):
                self.vertices[i+l][2] = n
        elif isinstance(obj, Web):
            indices.extend(((a+l, b+l)  for a,b in obj.edges))
            for e, track in zip(obj.edges, obj.tracks):
                for p in e:
                    self.vertices[p+l][5] = track
        
        elif hasattr(obj, '__iter__'):
            n = len(self.vertices)
            for obj in obj:
                if isinstance(obj, (fvec3, vec3)):
                    self.vertices.append([
                                self.current['space'], 
                                fvec3(obj), 
                                self.current['normal'], 
                                self.current['color'], 
                                self.current['layer'], 
                                self.current['track'], 
                                self.current['flags'],
                                ])
                    n += 1
                else:
                    self.add(obj)
            indices.extend((i,i+1)    for i in range(l, n-1))
        else:
            self.component(obj)
            
        return self
    
    def component(self, obj, **kwargs):
        ''' Add an object as component associated to the current space '''
        self.set(**kwargs)
        self.components.append((self.current['space'], obj))
        
        return self
    
    class display(Display):
        ''' Display for schemes
            
            Attributes:
            :spaces:       numpy array of matrices for each space, sent as uniform to the shader
            :vb_vertices:  vertex buffer for vertices
            :vas:          vertex array associated to each shader
        '''
        max_spaces = 32  # this maximum size of the spaces array must match the size in the shader
        
        def __init__(self, scene, sch):
            ctx = scene.ctx
            
            # load the resources
            self.shaders, self.shader_ident = scene.resource('scheme', self.load)
            
            # switch to array indexed spaces
            self.spaces = typedlist.full(fmat4(0), self.max_spaces)
            self.spacegens = list(sch.spaces)
            if len(self.spacegens) > self.max_spaces:
                print('warning: the number of local spaces exceeds the arbitrary build-in limit of {}'.format(self.max_spaces))
            
            self.components = [(space,scene.display(obj))    for space,obj in sch.components]
            
            
            self.nidents = max(v[5] for v in sch.vertices)+1
            self.box = boundingbox(
                        (fvec3(v[1]) for v in sch.vertices if self.spacegens[v[0]] is world), 
                        default=Box(center=fvec3(0), width=fvec3(-inf)),
                        )
            
            # prepare the buffer of vertices
            vertices = np.empty(len(sch.vertices), 'u1, 3f4, 3f4, 4u1, f4, u2, u1')
            for i,v in enumerate(sch.vertices):
                vertices[i] = (
                    *v[:3],
                    u8vec4(v[3]*255), 
                    *v[4:]
                    )            
            self.vb_vertices = ctx.buffer(vertices)
            verticesdef = [(self.vb_vertices, 'u1 3f4 3f4 4f1 f4 u2 u1', 
                                'space', 
                                'v_position', 
                                'v_normal', 
                                'v_color', 
                                'v_layer', 
                                'v_ident', 
                                'v_flags')]
            
            # prepare the rending commands
            ident_triangles = []
            ident_lines = []
            self.vas = {}
            self.vai_triangles = None
            self.vai_lines = None
            for shname,batch in sch.primitives.items():
                if not batch:    continue
                if shname not in self.shaders:    raise KeyError('no shader for name {}'.format(repr(shname)))
                
                prim, shader = self.shaders[shname]
                vb_indices = ctx.buffer(np.array(batch, 'u4'))
                self.vas[shname] = (prim, ctx.vertex_array(shader, verticesdef, vb_indices, skip_errors=True))
                if prim == mgl.LINES:            ident_lines.extend(batch)
                elif prim == mgl.TRIANGLES:        ident_triangles.extend(batch)
            
            if ident_triangles:    self.vai_triangles    = ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_triangles, 'u4')), skip_errors=True)
            if ident_lines:        self.vai_lines         = ctx.vertex_array(self.shader_ident, verticesdef, ctx.buffer(np.array(ident_lines, 'u4')), skip_errors=True)
            
        def __del__(self):
            for prim, va in self.vas.values():
                va.release()
            self.vas.clear()
            if self.vai_triangles:
                self.vai_triangles.release()
            if self.vai_lines:
                self.vai_lines.release()
            if self.vb_vertices:
                self.vb_vertices.release()
                self.vb_vertices = None
            
        def load(self, scene):
            ''' Load shaders and all static data for the current OpenGL context '''
            vert = open(resourcedir+'/shaders/scheme.vert').read()
            shader_ident = scene.ctx.program(
                        vertex_shader=vert,
                        fragment_shader=open(resourcedir+'/shaders/scheme-ident.frag').read(),
                        )
            shaders = {
                'line': (mgl.LINES, scene.ctx.program(
                        vertex_shader=vert,
                        fragment_shader=open(resourcedir+'/shaders/scheme-uniform.frag').read(),
                        )),
                'fill': (mgl.TRIANGLES, scene.ctx.program(
                        vertex_shader=vert,
                        fragment_shader=open(resourcedir+'/shaders/scheme-uniform.frag').read(),
                        )),
                'ghost': (mgl.TRIANGLES, scene.ctx.program(
                        vertex_shader=vert,
                        fragment_shader=open(resourcedir+'/shaders/scheme-ghost.frag').read(),
                        )),
                }
            return shaders, shader_ident
            
        def compute_spaces(self, view):
            ''' Compute the new spaces for this frame.
                This is meant to be overriden when new spaces are required 
            '''
            view.uniforms['world'] = self.world
            for i,gen in enumerate(self.spacegens):
                self.spaces[i] = gen(view)
            if self.components:
                invview = affineInverse(view.uniforms['view'])
                for space,disp in self.components:
                    disp.world = invview * self.spaces[space]
        
        def render(self, view):
            ''' Render each va in self.vas '''
            #self.compute_spaces(view)
            for name in self.vas:
                shader = self.shaders[name][1]
                prim, va = self.vas[name]
                shader['spaces'].write(self.spaces)
                shader['proj'].write(view.uniforms['proj'])
                shader['highlight'].write( fvec4(fvec3(settings.display['select_color_line']), 0.5) if self.selected else fvec4(0) )
                va.render(prim)
        
        def identify(self, view):
            ''' Render all the triangles and lines for identification '''
            self.shader_ident['startident'] = view.identstep(self.nidents)
            self.shader_ident['spaces'].write(self.spaces)
            self.shader_ident['proj'].write(view.uniforms['proj'])
            
            if self.vai_lines:        self.vai_lines.render(mgl.LINES)
            if self.vai_triangles:    self.vai_triangles.render(mgl.TRIANGLES)
        
        def stack(self, scene):
            yield ((), 'screen', -1, self.compute_spaces)
            yield ((), 'screen', 2, self.render) 
            yield ((), 'ident', 2, self.identify)
            for space,disp in self.components:
                yield from disp.stack(scene)

class SchemeDisplay:
    pass

def world(view):
    return view.uniforms['view'] * view.uniforms['world']

def screen(view):
    return fmat4(view.target.width/2,0,0,0,
                0,view.target.height/2,0,0,
                0,0,1,0,
                0,0,-1,1)

def halo_world(position):
    position = fvec4(position,1)
    def mat(view):
        center = view.uniforms['view'] * (view.uniforms['world'] * position)
        m = fmat4(1)
        m[3] = center
        return m
    return mat
def halo_view(position):
    position = fvec4(position,1)
    def mat(view):
        center = view.uniforms['view'] * (view.uniforms['world'] * position)
        proj = view.uniforms['proj']
        e = proj * fvec4(1,1,center.z,1)
        e /= e[3]
        m = fmat4(1/e[1])
        m[3] = center
        return m
    return mat
def halo_screen(position):
    position = fvec4(position,1)
    def mat(view):
        center = view.uniforms['view'] * (view.uniforms['world'] * position)
        e = view.uniforms['proj'] * fvec4(1,1,center.z,1)
        e /= e[3]
        m = fmat4(2/(e[1]*view.target.height))
        m[3] = center
        return m
    return mat
