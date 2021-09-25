from madcad import *
from madcad.reverse import segmentation

folder = os.path.dirname(__file__) + '/cycloid-gearbox'
part = read(folder+'/base_top_r0.stl')
part.mergeclose()

show([
	segmentation(part),
	])

