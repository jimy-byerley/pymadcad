from madcad import *
from madcad.reverse import segmentation

folder = os.path.dirname(__file__) + '/cycloid-gearbox'
#part = read(folder+'/base_stepper_r0.stl')
part = read(folder+'/output_housing_r0.stl')
part.mergeclose()

show([
	segmentation(part, tolerance=5, sharp=0.2),
	#reverse.debug,
	], options={'display_wire':True})

