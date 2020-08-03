from madcad.mathutils import *
from madcad.gear import *
from madcad import quickdisplay
import numpy as np
from matplotlib import pyplot as plt
from time import time

m = 1
z = 10
b = 3

start = time()
linear = racktooth(radians(20), 0.4, 0.3)
angular = geartooth(linear, 2*pi/z)
print('computation time', time()-start)

if True:
	# display profile
	plt.plot([p[0] for p in linear], [p[1] for p in linear], label='rack')
	plt.plot([p[0] for p in angular], [p[1] for p in angular], '.-', label='gear')
	plt.axes().set_aspect('equal')
	plt.figlegend()
	plt.show()

if True:
	# display gear surfaces
	res = surfgear(angular, m, z, b)
	#res = surfgear(angular, m, z, b, spin=radians(30))
	#res = surfscrewgear(angular, m, z, b, 0.6*b, 3)
	res.check()
	assert res.issurface()
	
	quickdisplay([res])
