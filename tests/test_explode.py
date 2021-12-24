from madcad import *
from madcad.kinematic import explode
from madcad.reverse import segmentation
import os

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

show(explode(parts), options={'display_faces':True})
