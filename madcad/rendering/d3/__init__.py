from .base import *

class Scene3D:
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


class SubView3D(SubView, QOpenGLWidget):
    ''' Qt widget to render and interact with displayable objects.
        It holds a scene as renderpipeline.

        Attributes:

            scene:        the `Scene` object displayed
            projection:   `Perspective` or `Orthographic`
            navigation:   `Orbit` or `Turntable`
            tool:         list of callables in priority order to receive events

            targets:     render targets matching those requested in `scene.stacks`
            uniforms:    parameters for rendering, used in shaders
    '''
    def __init__(self, scene, projection=None, navigation=None, parent=None):
        # super init
        QOpenGLWidget.__init__(self, parent)
        fmt = QSurfaceFormat()
        fmt.setVersion(*opengl_version)
        fmt.setProfile(QSurfaceFormat.CoreProfile)
        fmt.setSamples(4)
        self.setFormat(fmt)
        
        # ugly trick to receive interaction events in a different function than QOpenGLWidget.event (that one is locking the GIL during the whole rendering, killing any possibility of having a computing thread aside)
        # that event reception should be in the current widget ...
        self.handler = GhostWidget(self)
        self.handler.setFocusPolicy(Qt.StrongFocus)
        self.handler.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.setFocusProxy(self.handler)

        ViewCommon.__init__(self, scene, projection=projection, navigation=navigation)
        self.tool = [Tool(self.navigation.tool, self)] # tool stack, the last tool is used for input events, until it is removed 

    def init(self):
        w, h = self.width(), self.height()

        ctx = self.scene.ctx
        assert ctx, 'context is not initialized'

        # self.fb_screen is already created and sized by Qt
        self.fb_screen = ctx.detect_framebuffer(self.defaultFramebufferObject())
        self.fb_ident = ctx.simple_framebuffer((w, h), components=3, dtype='f1')
        self.targets = [ ('screen', self.fb_screen, self.setup_screen),
                         ('ident', self.fb_ident, self.setup_ident)]
        self.map_ident = np.empty((h,w), dtype='u2')
        self.map_depth = np.empty((h,w), dtype='f4')

    def render(self):
        # set the opengl current context from Qt (doing it only from moderngl interferes with Qt)
        self.makeCurrent()
        ViewCommon.render(self)

    
    # -- view stuff --

    def look(self, position: fvec3=None):
        ViewCommon.look(self, position)
        self.update()

    def adjust(self, box:Box=None):
        ViewCommon.adjust(self, box)
        self.update()

    def center(self, center: fvec3=None):
        ViewCommon.center(self, center)
        self.update()
    
    def somenear(self, point: QPoint, radius=None) -> QPoint:
        some = ViewCommon.somenear(self, qt_2_glm(point), radius)
        if some:
            return glm_to_qt(some)

    def ptat(self, point: QPoint) -> fvec3:
        return ViewCommon.ptat(self, qt_2_glm(point))

    def ptfrom(self, point: QPoint, center: fvec3) -> fvec3:
        return ViewCommon.ptfrom(self, qt_2_glm(point), center)

    def itemat(self, point: QPoint) -> 'key':
        return ViewCommon.itemat(self, qt_2_glm(point))
    

    # -- event system --

    def inputEvent(self, evt):
        ''' Default handler for every input event (mouse move, press, release, keyboard, ...)
            When the event is not accepted, the usual matching Qt handlers are used (mousePressEvent, KeyPressEvent, etc).

            This function can be overwritten to change the view widget behavior.
        '''
        # send the event to the current tools using the view
        if self.tool:
            for tool in reversed(self.tool):
                tool(evt)
                if evt.isAccepted():    return

        # send the event to the scene objects, descending the item tree
        if isinstance(evt, QMouseEvent) and evt.type() in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease, QEvent.MouseButtonDblClick, QEvent.MouseMove):
            pos = self.somenear(evt.pos())
            if pos:
                key = self.itemat(pos)
                if key:
                    self.control(key, evt)
                    if evt.isAccepted():    return

            # if clicks are not accepted, then some following keyboard events may not come to the widget
            # NOTE this also discarding the ability to move the window from empty areas
            if evt.type() == QEvent.MouseButtonPress:
                evt.accept()

    def control(self, key, evt):
        ''' Transmit a control event successively to all the displays matching the key path stages.
            At each level, if the event is not accepted, it transmits to sub items

            This function can be overwritten to change the interaction with the scene objects.
        '''
        disp = self.scene.displays
        stack = []
        for i in range(1,len(key)):
            disp = disp[key[i-1]]
            disp.control(self, key[:i], key[i:], evt)
            if evt.isAccepted(): return
            stack.append(disp)

        if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
            disp = stack[-1]
            # select what is under cursor
            if type(disp).__name__ in ('SolidDisplay', 'WebDisplay'):
                disp.vertices.selectsub(key[-1])
                disp.selected = any(disp.vertices.flags & 0x1)
            else:
                disp.selected = not disp.selected
            # make sure that a display is selected if one of its sub displays is
            for disp in reversed(stack):
                if hasattr(disp, '__iter__'):
                    disp.selected = any(sub.selected    for sub in disp)
            self.update()

    # -- Qt things --

    def initializeGL(self):
        # retrieve global shared context if available
        global global_context
        
        if QApplication.testAttribute(Qt.AA_ShareOpenGLContexts):
            if not global_context:
                global_context = mgl.create_context()
            self.scene.ctx = global_context
        # or create a context
        else:
            self.scene.ctx = mgl.create_context()
        self.init()
        self.preload()

    def paintGL(self):
        self.makeCurrent()
        self.render()

    def resizeEvent(self, evt):
        QOpenGLWidget.resizeEvent(self, evt)
        self.handler.resize(self.size())
        self.init()
        self.update()

    def changeEvent(self, evt):
        # detect theme change
        if evt.type() == QEvent.PaletteChange and settings.display['system_theme']:
            settings.use_qt_colors()
        return QOpenGLWidget.changeEvent(self, evt)


class Offscreen3D(SubView):
    ''' Object allowing to perform offscreen rendering, navigate and get information from screen as for a normal window 
    '''
    def __init__(self, scene, size=uvec2(400,400), projection=None, navigation=None):
        global global_context
        
        super().__init__(scene, projection=projection, navigation=navigation)
        
        if global_context:
            self.scene.ctx = global_context
        else:
            self.scene.ctx = global_context = mgl.create_standalone_context(requires=opengl_version)
        self.scene.ctx.line_width = settings.display["line_width"]

        self.init(size)
        self.preload()

    def init(self, size):
        w, h = size

        ctx = self.scene.ctx
        assert ctx, 'context is not initialized'

        # self.fb_frame is already created and sized by Qt
        self.fb_screen = ctx.simple_framebuffer(size)
        self.fb_ident = ctx.simple_framebuffer(size, components=3, dtype='f1')
        self.targets = [ ('screen', self.fb_screen, self.setup_screen),
                         ('ident', self.fb_ident, self.setup_ident)]
        self.map_ident = np.empty((h,w), dtype='u2')
        self.map_depth = np.empty((h,w), dtype='f4')
        
    @property
    def size(self):        
        return self.fb_screen.size
        
    def width(self):
        return self.fb_screen.size[0]
        
    def height(self):
        return self.fb_screen.size[1]
    
    def resize(self, size):
        if size != self.fb_screen.size:
            self.ctx.finish()
            self.init(size)

    def render(self):
        super().render()
        return Image.frombytes('RGBA', tuple(self.size), self.fb_screen.read(components=4), 'raw', 'RGBA', 0, -1)



try:
    from .qt import *
except ImportError:
    pass
else:

    class QView3D(QOpenGLWidget):
        """Onscreen rendering using a QOpenGLWidget"""

        def __init__(self, scene, scene, projection=None, navigation=None, **options):
            pass

        def look(self, position: fvec3 = None):
            pass

        def adjust(self, box: Box = None):
            pass

        def center(self, center: fvec3 = None):
            pass

        def inputEvent(self, event):
            pass
