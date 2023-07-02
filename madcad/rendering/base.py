from PIL import Image
from ..mathutils import *
from math import inf
from copy import deepcopy
from .. import settings
from .qt import QWidget
import numpy.core as np
import traceback
from operator import itemgetter

def writeproperty(func):
    ''' Decorator to create a property that has only an action on variable write '''
    fieldname = '_' + func.__name__
    def getter(self):   return getattr(self, fieldname)
    def setter(self, value):
        setattr(self, fieldname, value)
        func(self, value)
    return property(getter, setter, doc=func.__doc__)

def npboundingbox(points):
    ''' boundingbox for numpy arrays of points on the 3 first components '''
    return Box(
        fvec3([float(np.min(points[:,i])) for i in range(3)]),
        fvec3([float(np.max(points[:,i])) for i in range(3)]),
    )

class Display:
    ''' Blanket implementation for displays.
        This class signature is exactly the display protocol specification

        Attributes:

            world(fmat4):  matrix from local space to parent space
            box(Box):      boudingbox of the display in local space

        These attributes are variable members by default but can be overridden as properties if needed.
    '''

    # Mandatory part of the protocol
    box = Box(center=0, width=fvec3(-inf))  # to inform the scene and the view of the object size
    world = fmat4(1)                        # set by the display containing this one if it is belonging to a group

    def display(self, scene) -> 'self':
        ''' Displays are obviously displayable as themselves '''
        return self

    def stack(self, scene) -> '[(key, target, priority, callable)]':
        ''' Rendering functions to insert in the render pipeline.

            The expected result can be any iterable providing tuples `(key, target, priority, callable)` such as:

            :key:       a tuple with the successive keys in the displays tree, most of the time implementations set it to `()`  because it doesn't belong to a subpart of the Display.
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

class Displayable:
    ''' Simple displayable initializing the given Display class with arguments 
        
        At the display creation time, it will simply execute `build(*args, **kwargs)`
    '''
    __slots__ = 'build', 'args', 'kwargs'
    def __init__(self, build, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.build = build

    def __repr__(self):
        return '{}({}, {}, {})'.format(type(self).__name__, repr(self.args[1:-1]), repr(self.kwargs)[1:-1])

    def display(self, scene):
        return self.build(scene, *self.args, **self.kwargs)

class Group(Display):
    ''' A group is like a subscene '''
    def __init__(self, scene, objs:'dict/list'=None, pose=1):
        self._world = fmat4(1)
        self._pose = fmat4(pose)
        self.displays = {}
        if objs:
            self.dequeue(scene, objs)

    def __getitem__(self, key):
        return self.displays[key]

    def __iter__(self):
        return iter(self.displays.values())

    def update(self, scene, objs):
        if isinstance(objs, dict):          objs = objs
        elif hasattr(objs, 'keys'):         objs = dict(objs)
        elif hasattr(objs, '__iter__'):     objs = dict(enumerate(objs))
        else:                               return False

        # update displays
        sub = self._world * self._pose
        with scene.ctx:
            scene.ctx.finish()
            for key, obj in objs.items():
                if not displayable(obj):
                    continue
                try:
                    self.displays[key] = disp = scene.display(obj, self.displays.get(key))
                    disp.world = sub
                except:
                    print('Tried to display', object.__repr__(obj))
                    traceback.print_exc()
            for key in self.displays.keys() - objs.keys():
                del self.displays[key]
        scene.touch()
        return True
    dequeue = update

    def stack(self, scene):
        for key,display in self.displays.items():
            for sub, target, priority, func in display.stack(scene):
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

# dictionary to store procedures to override default object displays
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
                        print('Tried to display', object.__repr__(displayable))
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
        if type(obj) in self.overrides:
            disp = self.overrides[type(obj)](self, obj)
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

class SubView(Display):
    '''A SubView is like a Group with its own rendering settings and display queue'''
    def __init__(self, scene, **options):
        raise NotImplementedError(f"__init__(self, scene, **options) is not implemented for {self.__class__.__name__}")

    def stack(self, scene, **options) -> '[(key, target, priority, callable)]':
        raise NotImplementedError(f"stack(self, scene, **options) is not implemented for {self.__class__.__name__}")

    def control(self, view, key, sub, event):
        raise NotImplementedError(f"control(self, view, key, sub, event) is not implemented for {self.__class__.__name__}")

class Offscreen:
    '''
    An Offscreen view is like a View but renders on offscreen buffers.
    The result can be retrieved to an image or be copied to an other rendering target
    '''
    def __init__(self, scene, projection=None, navigation=None, **options):
        raise NotImplementedError(f"__init__(self, scene, projection=None, navigation=None, **options) is not implemented for {self.__class__.__name__}")

    def init(self, size):
        w, h = size

        ctx = self.scene.ctx
        assert ctx, 'context is not initialized'

        # self.fb_frame is already created and sized by Qt
        self.fb_screen = ctx.simple_framebuffer(size)
        self.fb_ident = ctx.simple_framebuffer(size, components=3, dtype='f1')
        self.targets = [
            ('screen', self.fb_screen, self.setup_screen),
            ('ident',  self.fb_ident,  self.setup_ident),
        ]
        self.map_ident = np.empty((h, w), dtype='u2')
        self.map_depth = np.empty((h, w), dtype='f4')

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

class GhostWidget(QWidget):
    def __init__(self, parent):
        super().__init__(parent)

    def event(self, evt):
        if isinstance(evt, QInputEvent):
            # set the opengl current context from Qt (doing it only from moderngl interferes with Qt)
            #self.makeCurrent()
            evt.ignore()
            self.parent().inputEvent(evt)
            if evt.isAccepted():    return True
        elif isinstance(evt, QFocusEvent):
            self.parent().event(evt)
        return super().event(evt)

def snail(radius):
	''' Generator of coordinates snailing around 0,0 '''
	x = 0
	y = 0
	for r in range(radius):
		for x in range(-r,r):		yield ivec2(x,-r)
		for y in range(-r,r):		yield ivec2(r, y)
		for x in reversed(range(-r,r)):	yield ivec2(x, r)
		for y in reversed(range(-r,r)):	yield ivec2(-r,y)

def snailaround(pt, box, radius):
	''' Generator of coordinates snailing around pt, coordinates that goes out of the box are skipped '''
	cx,cy = pt
	mx,my = box
	for rx,ry in snail(radius):
		x,y = cx+rx, cy+ry
		if 0 <= x and x < mx and 0 <= y and y < my:
			yield ivec2(x,y)
