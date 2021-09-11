from madcad import *
from madcad.reverse import segmentation

part = read('/home/jimy/cycloid-gearbox/files/base_top_r0.stl')
part.mergeclose()

show([
	segmentation(part),
	])

