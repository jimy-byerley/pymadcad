from madcad import *
from madcad.kinematic import *
from madcad.joints import *

# show([Pivot((0,1), Axis(O,X))])
show([Pivot((0,1), Axis(O,X)), Pivot((1,2), Axis(Y,X)), Planar((0,3), Axis(-Z,Y))])
