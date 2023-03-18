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
    '''A Group contains other displays and its subdisplays are moved with the group'''
    @setter
    def world(self, world):
        raise NotImplementedError(f"world(self, world) is not implemented for {self.__class__.__name__}")

    @setter
    def pose(self, world):
        raise NotImplementedError(f"pose(self, world) is not implemented for {self.__class__.__name__}")

    def stack(self, scene) -> '[(key, target, priority, callable)]':
        raise NotImplementedError(f"stack(self, scene) is not implemented for {self.__class__.__name__}")

    def __setitem__(self, key, displayable):
        raise NotImplementedError(f"__setitem__(self, key, value) is not implemented for {self.__class__.__name__}")

    def __getitem__(self, key):
        raise NotImplementedError(f"__getitem__(self, key) is not implemented for {self.__class__.__name__}")

    def __delitem__(self, key):
        raise NotImplementedError(f"__delitem__(self, key) is not implemented for {self.__class__.__name__}")

class Scene:
    '''Abstract class for Scene'''
    overrides = {}
    stack = {}

    def __setitem__(self, key, value):
        raise NotImplementedError(f"__setitem__(self, key, value) is not implemented for {self.__class__.__name__}")

    def __getitem__(self, key) -> 'display':
        raise NotImplementedError(f"__getitem__(self, key) is not implemented for {self.__class__.__name__}")

    def __delitem__(self, key):
        raise NotImplementedError(f"__delitem__(self, key) is not implemented for {self.__class__.__name__}")

    def restack(self):
        raise NotImplementedError(f"restack(self) is not implemented for {self.__class__.__name__}")

    def render(self, view):
        raise NotImplementedError(f"render(self) is not implemented for {self.__class__.__name__}")

    def display(self, obj, former=None):
        raise NotImplementedError(f"display(self) is not implemented for {self.__class__.__name__}")

class SubView(Display):
    '''A SubView is like a Group with its own rendering settings and display queue'''
    def __init__(self, scene, **options):
        raise NotImplementedError(f"__init__(self, scene, **options) is not implemented for {self.__class__.__name__}")

    def stack(self, scene, **options) -> '[(key, target, priority, callable)]':
        raise NotImplementedError(f"stack(self, scene, **options) is not implemented for {self.__class__.__name__}")

    def control(self, view, key, sub, event):
        raise NotImplementedError(f"control(self, view, key, sub, event) is not implemented for {self.__class__.__name__}")

class Offscreen:
    '''An Offscreen view is like a View but renders on offscreen buffers.
    The result can be retrieved to an image or be copied to an other rendering target
    '''
    def __init__(self, scene, projection=None, navigation=None, **options):
        raise NotImplementedError(f"__init__(self, scene, projection=None, navigation=None, **options) is not implemented for {self.__class__.__name__}")

    def render(self):
        raise NotImplementedError(f"render(self) is not implemented for {self.__class__.__name__}")
