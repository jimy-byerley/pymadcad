import os
from pytest import skip

from madcad import *
from madcad.kinematic import explode
from madcad.reverse import segmentation
from . import visualcheck

@visualcheck
def test_explode():
	skip('needs fixes')
	
	folder = os.path.dirname(__file__) + '/cycloid-gearbox'

	imported = read(folder+'/sim_cyc_nema17.stl')
	imported.mergeclose()
	print(repr(imported))
	parts = []
	for part in imported.islands():
		part.strippoints()
		part = segmentation(part)
		parts.append(Solid(part=part))
		print(repr(part))
	exploded = explode(parts)
	return exploded
