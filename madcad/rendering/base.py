class SubView:
    ''' Common base for Qt's View rendering and Offscreen rendering. It provides common methods to render and interact with a view.
        
        You should always use one of its subclass.
    '''
    def __init__(self, scene, projection=None, navigation=None):
        # interaction methods
        self.projection = projection or globals()[settings.scene['projection']]()
        self.navigation = navigation or globals()[settings.controls['navigation']]()

        # render parameters
        self.scene = scene if isinstance(scene, Scene) else Scene(scene)
        self.uniforms = {'proj':fmat4(1), 'view':fmat4(1), 'projview':fmat4(1)}    # last frame rendering constants
        self.targets = []
        self.steps = []
        self.step = 0
        self.stepi = 0

        # dump targets
        self.map_depth = None
        self.map_idents = None
        self.fresh = set()    # set of refreshed internal variables since the last render

    # -- internal frame system --

    def refreshmaps(self):
        ''' Load the rendered frames from the GPU to the CPU

            - When a picture is used to GPU rendering it's called 'frame'
            - When it is dumped to the RAM we call it 'map' in this library
        '''
        if 'fb_ident' not in self.fresh:
            self.makeCurrent()    # set the scene context as current opengl context
            with self.scene.ctx as ctx:
                #ctx.finish()
                self.fb_ident.read_into(self.map_ident, viewport=self.fb_ident.viewport, components=2)
                self.fb_ident.read_into(self.map_depth, viewport=self.fb_ident.viewport, components=1, attachment=-1, dtype='f4')
            self.fresh.add('fb_ident')
            #from PIL import Image
            #Image.fromarray(self.map_ident*16, 'I;16').show()

    def render(self):
        # prepare the view uniforms
        w, h = self.fb_screen.size
        self.uniforms['view'] = view = self.navigation.matrix()
        self.uniforms['proj'] = proj = self.projection.matrix(w/h, self.navigation.distance)
        self.uniforms['projview'] = proj * view
        self.fresh.clear()

        # call the render stack
        self.scene.render(self)

    def identstep(self, nidents):
        ''' Updates the amount of rendered idents and return the start ident for the calling rendering pass?
            Method to call during a renderstep
        '''
        s = self.step
        self.step += nidents
        self.steps[self.stepi] = self.step-1
        self.stepi += 1
        return s

    def setup_ident(self):
        # steps for fast fast search of displays with the idents
        self.stepi = 0
        self.step = 1
        if 'ident' in self.scene.stacks and len(self.scene.stacks['ident']) != len(self.steps):
            self.steps = [0] * len(self.scene.stacks['ident'])
        # ident rendering setup
        ctx = self.scene.ctx
        ctx.multisample = False
        ctx.enable_only(mgl.DEPTH_TEST)
        ctx.blend_func = mgl.ONE, mgl.ZERO
        ctx.blend_equation = mgl.FUNC_ADD
        self.target.clear(0)

    def setup_screen(self):
        # screen rendering setup
        ctx = self.scene.ctx
        ctx.multisample = True
        ctx.enable_only(mgl.BLEND | mgl.DEPTH_TEST)
        ctx.blend_func = mgl.SRC_ALPHA, mgl.ONE_MINUS_SRC_ALPHA
        ctx.blend_equation = mgl.FUNC_ADD
        
        background = settings.display['background_color']
        if len(background) == 3:
            self.target.clear(*background, alpha=1)
        elif len(background) == 4:
            self.target.clear(*background)
        else:
            raise ValueError(f"background_color must be a RGB or RGBA tuple, currently {background}")

    def preload(self):
        ''' Internal method to load common resources '''
        ctx, resources = self.scene.ctx, self.scene.resources
        resources['shader_ident'] = ctx.program(
                    vertex_shader=open(resourcedir+'/shaders/object-ident.vert').read(),
                    fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
                    )

        resources['shader_subident'] = ctx.program(
                    vertex_shader=open(resourcedir+'/shaders/object-item-ident.vert').read(),
                    fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
                    )

    # -- methods to deal with the view --

    def somenear(self, point: ivec2, radius=None) -> ivec2:
        ''' Return the closest coordinate to coords, (within the given radius) for which there is an object at
            So if objnear is returning something, objat and ptat will return something at the returned point
        '''
        if radius is None:
            radius = settings.controls['snap_dist']
        self.refreshmaps()
        for x,y in snailaround(point, (self.map_ident.shape[1], self.map_ident.shape[0]), radius):
            ident = int(self.map_ident[-y, x])
            if ident:
                return uvec2(x,y)

    def ptat(self, point: ivec2) -> fvec3:
        ''' Return the point of the rendered surfaces that match the given window coordinates '''
        self.refreshmaps()
        viewport = self.fb_ident.viewport
        depthred = float(self.map_depth[-point.y,point.x])
        x =  (point.x/viewport[2] *2 -1)
        y = -(point.y/viewport[3] *2 -1)

        if depthred == 1.0:
            return None
        else:
            view = self.uniforms['view']
            proj = self.uniforms['proj']
            a,b = proj[2][2], proj[3][2]
            depth = b/(depthred + a) * 0.5    # TODO get the true depth  (can't get why there is a strange factor ... opengl trick)
            #near, far = self.projection.limits  or settings.display['view_limits']
            #depth = 2 * near / (far + near - depthred * (far - near))
            #print('depth', depth, depthred)
            return vec3(fvec3(affineInverse(view) * fvec4(
                        depth * x /proj[0][0],
                        depth * y /proj[1][1],
                        -depth,
                        1)))

    def ptfrom(self, point: ivec2, center: fvec3) -> fvec3:
        ''' 3D point below the cursor in the plane orthogonal to the sight, with center as origin '''
        view = self.uniforms['view']
        proj = self.uniforms['proj']
        viewport = self.fb_ident.viewport
        x =  (point.x/viewport[2] *2 -1)
        y = -(point.y/viewport[3] *2 -1)
        depth = (view * fvec4(fvec3(center),1))[2]
        return vec3(fvec3(affineInverse(view) * fvec4(
                    -depth * x /proj[0][0],
                    -depth * y /proj[1][1],
                    depth,
                    1)))

    def itemat(self, point: ivec2) -> 'key':
        ''' Return the key path of the object at the given screen position (widget relative).
            If no object is at this exact location, None is returned
        '''
        self.refreshmaps()
        point = uvec2(point)
        ident = int(self.map_ident[-point.y, point.x])
        if ident and 'ident' in self.scene.stacks:
            rdri = bisect(self.steps, ident)
            if rdri == len(self.steps):
                print('internal error: object ident points out of idents list')
            while rdri > 0 and self.steps[rdri-1] == ident:    rdri -= 1
            if rdri > 0:    subi = ident - self.steps[rdri-1] - 1
            else:            subi = ident - 1
            
            if rdri >= len(self.scene.stacks['ident']):
                print('wrong identification index', ident, self.scene.stacks['ident'][-1])
                nprint(self.scene.stacks['ident'])
                return
            
            return (*self.scene.stacks['ident'][rdri][0], subi)

    # -- view stuff --

    def look(self, position: fvec3=None):
        ''' Make the scene navigation look at the position.
            This is changing the camera direction, center and distance.
        '''
        if not position:    position = self.scene.box().center
        dir = position - fvec3(affineInverse(self.navigation.matrix())[3])
        if not dot(dir,dir) > 1e-6 or not isfinite(position):    return

        if isinstance(self.navigation, Turntable):
            self.navigation.yaw = atan2(dir.x, dir.y)
            self.navigation.pitch = -atan2(dir.z, length(dir.xy))
            self.navigation.center = position
            self.navigation.distance = length(dir)
        elif isinstance(self.navigation, Orbit):
            focal = self.orient * fvec3(0,0,1)
            self.navigation.orient = quat(dir, focal) * self.navigation.orient
            self.navigation.center = position
            self.navigation.distance = length(dir)
        else:
            raise TypeError("navigation {} is not supported by 'look'".format(type(self.navigation)))

    def adjust(self, box:Box=None):
        ''' Make the navigation camera large enough to get the given box in .
            This is changing the zoom level
        '''
        if not box:    box = self.scene.box()
        if box.isempty():    return

        # get the most distant point to the focal axis
        invview = affineInverse(self.navigation.matrix())
        camera, look = fvec3(invview[3]), fvec3(invview[2])
        dist = length(noproject(box.center-camera, look)) + max(glm.abs(box.width))/2 * 1.1
        if not dist > 1e-6:    return

        # adjust navigation distance
        if isinstance(self.projection, Perspective):
            self.navigation.distance = dist / tan(self.projection.fov/2)
        elif isinstance(self.projection, Orthographic):
            self.navigation.distance = dist / self.projection.size
        else:
            raise TypeError('projection {} not supported'.format(type(self.projection)))

    def center(self, center: fvec3=None):
        ''' Relocate the navigation to the given position .
            This is translating the camera.
        '''
        if not center:    center = self.scene.box().center
        if not isfinite(center):    return

        self.navigation.center = center
