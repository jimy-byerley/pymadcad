import moderngl as mgl

class Display:
    ''' Blanket implementation for displays.
        This class signature is exactly the display protocol specification
        
        Attributes:
        
            world(fmat4):  matrix from local space to parent space
            box(Box):      boudingbox of the display in local space
            
        These attributes are variable members by default but can be overridden as properties if needed.
    '''
    
    # mendatory part of the protocol
    
    box = Box(center=0, width=fvec3(-inf))    # to inform the scene and the view of the object size
    world = fmat4(1)        # set by the display containing this one if it is belonging to a group
    
    def display(self, scene) -> 'self':
        ''' Displays are obviously displayable as themselves '''
        return self
    def stack(self, scene) -> '[(key, target, priority, callable)]':
        ''' Rendering functions to insert in the render pipeline.
        
            The expected result can be any iterable providing tuples `(key, target, priority, callable)` such as:
            
            :key:        a tuple with the successive keys in the displays tree, most of the time implementations set it to `()`  because it doesn't belong to a subpart of the Display.
            :target:    the name of the render target in the view that will be rendered (see View)
            :priority:  a float that is used to insert the callable at the proper place in the rendering stack
            :callable:  a function that renders, signature is `func(view)`
            
            The view contains the uniforms, rendering targets and the scene for common resources
        '''
        return ()
    def duplicate(self, src, dst) -> 'display/None':
        ''' Duplicate the display for an other scene (other context) but keeping the same memory buffers when possible.
            
            Return None if not possible or not implemented.
        '''
        return None
    def __getitem__(self, key) -> 'display':
        ''' Get a subdisplay by its index/key in this display (like in a scene) '''
        raise IndexError('{} has no sub displays'.format(type(self).__name__))
    def update(self, scene, displayable) -> bool:
        ''' Update the current displays internal data with the given displayable .
            
            If the display cannot be upgraded, it must return False to be replaced by a fresh new display created from the displayable
        '''
        return False
    
    # optional part for usage with Qt
    
    selected = False
    
    def control(self, view, key, sub, evt: 'QEvent'):
        ''' Handle input events occuring on the area of this display (or of one of its subdisplay).
            For subdisplay events, the parents control functions are called first, and the sub display controls are called only if the event is not accepted by parents
            
            Parameters:
                key:    the key path for the current display
                sub:    the key path for the subdisplay
                evt:    the Qt event (see Qt doc)
        '''
        pass

class Group(Display):
    ''' A group is like a subscene '''
    def __init__(self, scene, objs:'dict/list'=None, pose=1):
        self._world = fmat4(1)
        self._pose = fmat4(pose)
        self.displays = {}
        if objs:    self.dequeue(scene, objs)
    
    def __getitem__(self, key):
        return self.displays[key]
    def __iter__(self):
        return iter(self.displays.values())
        
    def update(self, scene, objs):
        if isinstance(objs, dict):         objs = objs
        elif hasattr(objs, 'keys'):        objs = dict(objs)
        elif hasattr(objs, '__iter__'):    objs = dict(enumerate(objs))
        else:
            return False
        
        # update displays
        sub = self._world * self._pose
        with scene.ctx:
            scene.ctx.finish()
            for key, obj in objs.items():
                if not displayable(obj):    continue
                try:
                    self.displays[key] = disp = scene.display(obj, self.displays.get(key))
                    disp.world = sub
                except:
                    print('tried to display', object.__repr__(obj))
                    traceback.print_exc()
            for key in self.displays.keys() - objs.keys():
                del self.displays[key]
        scene.touch()
        return True
    dequeue = update
    
    def stack(self, scene):
        for key,display in self.displays.items():
            for sub,target,priority,func in display.stack(scene):
                yield ((key, *sub), target, priority, func)
    
    @writeproperty
    def pose(self, pose):
        ''' Pose of the group relatively to its parents '''
        sub = self._world * self._pose
        for display in self.displays.values():
            display.world = sub
            
    @writeproperty
    def world(self, world):
        ''' Update children's world matrix applying the current pose in addition to world '''
        sub = self._world * self._pose
        for display in self.displays.values():
            display.world = sub
            
    @property
    def box(self):
        ''' Computes the boundingbox of the scene, with the current object poses '''
        box = Box(center=fvec3(0), width=fvec3(-inf))
        for display in self.displays.values():
            box.union_update(display.box)
        return box.transform(self._pose)

class Scene:
    ''' Rendering pipeline for madcad displayable objects 
        
        This class is gui-agnostic, it only relies on OpenGL, and the context has to be created by the user.
        
        When an object is added to the scene, a Display is not immediately created for it, the object is put into the queue and the Display is created at the next render.
        If the object is removed from the scene before the next render, it is dequeued.
        
        Attributes:
        
            ctx:                  moderngl Context (must be the same for all views using this scene)
            resources (dict):    dictionary of scene resources (like textures, shaders, etc) index by name
            options (dict):       dictionary of options for rendering, initialized with a copy of `settings.scene`
            
            displays (dict):      dictionary of items in the scheme `{'name': Display}`
            stacks (list):        lists of callables to render each target `{'target': [(key, priority, callable(view))]}`
            setup (dict):         setup of each rendering target `{'target': callable}`
            
            touched (bool):       flag set to True if the stack must be recomputed at the next render time (there is a change in a Display or in one of its children)
    '''
    overrides = {}
    
    def __init__(self, objs=(), options=None, ctx=None, setup=None):
        # context variables
        self.ctx = ctx
        self.resources = {}    # context-related resources, shared across displays, but not across contexts (shaders, vertexarrays, ...)
        
        # rendering options
        self.options = deepcopy(settings.scene)
        if options:    self.options.update(options)
        
        # render elements
        self.queue = {}    # list of objects to display, not yet loaded on the GPU
        self.displays = {} # displays created from the inserted objects, associated to their insertion key
        self.stacks = {}    # dict of list of callables, that constitute the render pipeline:  (key,  priority, callable)
        self.setup = setup or {}    # callable for each target
        
        self.touched = False
        self.update(objs)
    
    # methods to manage the rendering pipeline
    
    def add(self, displayable, key=None) -> 'key':
        ''' Add a displayable object to the scene, if key is not specified, an unused integer key is used 
            The object is not added to the render pipeline yet, but queued for next rendering.
        '''
        if key is None:
            for i in range(len(self.displays)+len(self.queue)+1):
                if i not in self.displays and i not in self.queue:    key = i
        self.queue[key] = displayable
        return key

    def __setitem__(self, key, value):
        ''' Equivalent with self.add with a key '''
        self.queue[key] = value
    def __getitem__(self, key) -> 'display':
        ''' Get the displayable for the given key, raise when there is no object or when the object is still in queue. '''
        return self.displays[key]
    def __delitem__(self, key):
        ''' Remove an item from the scene, at the root level '''
        if key in self.displays:
            del self.displays[key]
        if key in self.queue:
            del self.queue[key]
        for stack in self.stacks.values():
            for i in reversed(range(len(stack))):
                if stack[i][0][0] == key:
                    stack.pop(i)
                    
    def item(self, key):
        ''' Get the Display associated with the given key, descending the parenting tree 
        
            The parents must all make their children accessible via `__getitem__`
        '''
        disp = self.displays
        for i in range(1,len(key)):
            disp = disp[key[i-1]]
        return disp
    
    def update(self, objs:dict):
        ''' Rebuild the scene from a dictionary of displayables 
            Update former displays if possible instead of replacing it
        '''
        self.queue.update(objs)
        self.touch()
    
    def sync(self, objs:dict):
        ''' Update the scene from a dictionary of displayables, the former values that cannot be updated are discarded '''
        for key in list(self.displays):
            if key not in objs:
                del self.displays[key]
        self.update(objs)
    
    def touch(self):
        ''' Shorthand for `self.touched = True` '''
        self.touched = True
        
    def dequeue(self):
        ''' Load all pending objects to insert into the scene.
            This is called automatically by the next `render()` if `touch()` has been called
        '''
        if self.queue:
            with self.ctx:
                self.ctx.finish()
                # update displays
                for key,displayable in self.queue.items():
                    try:    
                        self.displays[key] = self.display(displayable, self.displays.get(key))
                    except:
                        print('\ntried to display', object.__repr__(displayable))
                        traceback.print_exc()
                self.touched = True
                self.queue.clear()
        
        if self.touched:
            self.restack()
            
    def restack(self):
        ''' Update the rendering calls stack from the current scene's displays.
            This is called automatically on `dequeue()`
        '''
        # recreate stacks
        for stack in self.stacks.values():
            stack.clear()
        for key,display in self.displays.items():
            for frame in display.stack(self):
                if len(frame) != 4:
                    raise ValueError('wrong frame format in the stack from {}\n\t got {}'.format(display, frame))
                sub,target,priority,func = frame
                if target not in self.stacks:    self.stacks[target] = []
                stack = self.stacks[target]
                stack.append(((key,*sub), priority, func))
        # sort the stack using the specified priorities
        for stack in self.stacks.values():
            stack.sort(key=itemgetter(1))
        self.touched = False
    
    def render(self, view):
        ''' Render to the view targets. 
            
            This must be called by the view widget, once the OpenGL context is set.
        '''
        empty = ()
        with self.ctx:
            # apply changes that need opengl runtime
            self.dequeue()
            # render everything
            for target, frame, setup in view.targets:
                view.target = frame
                frame.use()
                setup()
                for key, priority, func in self.stacks.get(target,empty):
                    func(view)
    
    def box(self):
        ''' Computes the boundingbox of the scene, with the current object poses '''
        box = Box(center=fvec3(0), width=fvec3(-inf))
        for display in self.displays.values():
            box.union_update(display.box.transform(display.world))
        return box
    
    def resource(self, name, func=None):
        ''' Get a resource loaded or load it using the function func.
            If func is not provided, an error is raised
        '''
        if name in self.resources:    
            return self.resources[name]
        elif callable(func):
            with self.ctx as ctx:  # set the scene context as current opengl context
                res = func(self)
                self.resources[name] = res
                return res
        else:
            raise KeyError("resource {} doesn't exist or is not loaded".format(repr(name)))
                    
    def display(self, obj, former=None):
        ''' Create a display for the given object for the current scene.
        
            This is the actual function converting objects into displays.
            You don't need to call this method if you just want to add an object to the scene, use add() instead
        '''
        if former and former.update(self, obj):
            return former
        if type(obj) in overrides:
            disp = overrides[type(obj)](self, obj)
        elif hasattr(obj, 'display'):
            if isinstance(obj.display, type):
                disp = obj.display(self, obj)
            elif callable(obj.display):
                disp = obj.display(self)
            else:
                raise TypeError("member 'display' must be a method or a type, on {}".format(type(obj).__name__))
        else:
            raise TypeError('type {} is not displayable'.format(type(obj).__name__))
        
        if not isinstance(disp, Display):
            raise TypeError('the display for {} is not a subclass of Display: {}'.format(type(obj).__name__, type(disp)))
        return disp

class SubView:
    ''' Common base for Qt's View rendering and Offscreen rendering. 
    It provides common methods to render and interact with a view.
        
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
