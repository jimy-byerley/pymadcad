import os
from pytest import skip

from madcad import *
from madcad.reverse import segmentation
from . import visualcheck

@visualcheck
def test_placement():
	skip('using old api')
	
	def load(name):
		folder = os.path.dirname(__file__) + '/cycloid-gearbox/'
		part = read(folder+name)
		part.mergeclose()
		part.strippoints()
		return segmentation(part)

	top = Solid(part=load('base_top_r0.stl'))
	stepper = Solid(part=load('base_stepper_r0.stl'))
	stepper = stepper.place(
		(stepper['part'].group(41), top['part'].group(48)),
		(stepper['part'].group(9), top['part'].group(6)),
		)

	screw = Solid(part=standard.screw(3, 18))
	screw = screw.transform(angleAxis(-pi/2,Y))

	stepper['screw'] = screw.place(
		(screw['part'].group(0), stepper['part'].group(55), angleAxis(pi,X)),
		(screw['part'].group(4), stepper['part'].group(25), angleAxis(pi,X)),
		)

	return [stepper, top]
