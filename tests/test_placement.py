from madcad import *
from madcad.reverse import segmentation

def load(name):
	folder = os.path.dirname(__file__) + '/cycloid-gearbox/'
	part = read(folder+name)
	part.mergeclose()
	part.strippoints()
	return segmentation(part)

top = Solid(part=load('base_top_r0.stl'))
stepper = Solid(part=load('base_stepper_r0.stl'))

scene = [
	stepper, top,
	#note_leading(top, text='top'), 
	#note_leading(stepper, text='stepper'),
	]

stepper.place(
	(stepper['part'].group(37), top['part'].group(48)),
	(stepper['part'].group(9), top['part'].group(6)),
	)

screw = Solid(part=standard.screw(3, 18))

screw.place(
	(screw['part'].group(0), stepper['part'].group(44), angleAxis(pi,X)),
	(screw['part'].group(4), stepper['part'].group(25), angleAxis(pi,X)),
	)
stepper['screw'] = screw
