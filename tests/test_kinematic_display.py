from madcad import *
from madcad.kinematic import *
from madcad.joints import *

# show([Pivot((0,1), Axis(O,X))])
show([
    Revolute((0,1), Axis(O,X)), 
    Revolute((1,2), Axis(Y,X)), 
    Planar((0,3), Axis(-Z,Y)), 
    Prismatic((0,3), Axis(-2*Z,X)),
    Cylindrical((3,4), Axis(-Z+Y,X)),
    Ball((0,4), -Z-Y),
    PointSlider((5,6), Axis(-2*Y,X)),
    EdgeSlider((7,6), translate(-Z-2*Y)),
    ])
