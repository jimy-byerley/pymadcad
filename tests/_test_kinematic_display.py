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
    Gear((0,1), -1, 1, Axis(-0*Z+2*Y,Z), Axis(-0*Z+2*Y,Z)),
    Gear((0,1), -0.5, 1, Axis(-1*Z+2*Y,Z), Axis(-1*Z+2*Y,Z)),
    Gear((0,1), 0.5, 0.5, Axis(-2*Z+2*Y,Z), Axis(-2*Z+2*Y,Z)),
    Gear((0,1), -1, translate(-Z) * rotate(pi/4,Y) * translate(Z), Axis(-3*Z+2*Y,Z), Axis(-3*Z+2*Y,Z)),
    Rack((0,1), 0.5, Axis(-0*Z+3*Y,Y), Axis(-0*Z+3*Y,Z)),
    ])
