from madcad import *
from madcad.standard import *

sections = [
	section_s(),
	section_w(),
	section_l(),
	section_c(),
	section_tslot(),
	]
	
for section in sections:
	section.check()
	assert section.isvalid()
	flatsurface(section)
	
show([ section.transform(i*Z)  for i,section in enumerate(sections) ])
