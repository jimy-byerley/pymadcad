import os
from pytest import skip

from madcad import *
from madcad.assembly import explode
from madcad.reverse import segmentation
from . import visualcheck

@visualcheck
def test_explode():
	# skip('needs fixes')
	
	folder = os.path.dirname(__file__) + '/cycloid-gearbox'

	imported = read(folder+'/sim_cyc_nema17.stl')
	imported.mergeclose()
	parts = []
	for part in imported.islands():
		part.strippoints()
		parts.append(Solid(part=segmentation(part)))
	exploded = explode(parts)
	return exploded
