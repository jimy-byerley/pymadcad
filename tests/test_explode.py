from madcad import *
from madcad.kinematic import explode
from madcad.reverse import segmentation
import os

folder = os.path.dirname(__file__) + '/cycloid-gearbox'
#parts = []
#print('path', folder, os.listdir(folder))
#for filename in os.listdir(folder):
	#if filename.endswith('.stl'):
		#part = read(folder+'/'+filename)
		#part.mergeclose()
		#part.strippoints()
		#parts.append(Solid(part=segmentation(part)))
		
imported = read(folder+'/sim_cyc_nema17.stl')
imported.mergeclose()
print(repr(imported))
parts = []
for part in imported.islands():
	part.strippoints()
	part = segmentation(part, tolerance=4)
	parts.append(Solid(part=part))
	print(repr(part))

graph = explode(parts, factor=1)

show([graph, parts], options={'display_faces':False, 'display_wire':False})
