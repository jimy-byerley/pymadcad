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

stepper.place(
	(stepper['part'].group(37), top['part'].group(48)),
	(stepper['part'].group(9), top['part'].group(6)),
	)

top['note'] = note_leading(top['part'], text='top'),
stepper['note'] = note_leading(stepper['part'], text='stepper')
show([top, stepper])
