from madcad import *
from madcad.standard import *
from . import visualcheck

from pytest import approx

@visualcheck
def test_bolts():
	d = 3
	return [
		screw(d,12,10, head='BH'),
		nut(d) .transform(vec3(0,0,-7)),
		washer(d) .transform(vec3(0,0,-5)),
		
		screw(5,12, head='SH') .transform(vec3(10,0,0)),
		nut(5) .transform(vec3(10,0,-5)),
		
		screw(4,12, head='VH', drive='slot') .transform(vec3(-10,0,0)),
		nut(4) .transform(vec3(-10,0,-5)),
		]
		
@visualcheck
def test_springs():
	return [
		coilspring_compression(20) .transform(vec3(0,10,0)),
		coilspring_tension(20) .transform(vec3(10,10,0)),
		coilspring_torsion(5) .transform(vec3(-10,10,0)),
		]
		
@visualcheck
def test_bearing():
	return [
		bearing(12, circulating='roller', contact=radians(20)) .transform(vec3(0,-30,0)),
		bearing(12, circulating='roller', contact=radians(20), detail=True) .transform(vec3(0,-30,20)),
		bearing(10, circulating='roller', contact=0, detail=True) .transform(vec3(0,-30,-20)),
		bearing(12, circulating='ball') .transform(vec3(30,-30,0)),
		bearing(12, circulating='ball', detail=True) .transform(vec3(30,-30,20)),
		bearing(12, contact=radians(90)) .transform(vec3(-30,-30,0)),
		bearing(12, contact=radians(90), detail=True) .transform(vec3(-30,-30,20)),
		]
		
@visualcheck
def test_slidebearing():
	return [
		slidebearing(10, shoulder=3) .transform(vec3(0, -60, 0)),
		slidebearing(10, opened=True) .transform(vec3(-20, -60, 0)),
		]

def test_roundings():
	assert stfloor(2.2343) == approx(2.2)
	assert stfloor(877.2) == approx(800)
	assert stfloor(877.2, 5e-2) == 850
	assert stfloor(877.2, 1e-2) == approx(875)
	
	assert stceil(2.2343) == approx(2.4)
	assert stceil(877.2) == approx(900)
	assert stceil(877.2, 1e-2) == approx(880)
	assert stceil(877.2, 1e-3) == approx(878)

@visualcheck
def test_slots():
	a = O
	b = 16*X

	b1 = bolt(a, b, 3)
	s1 = bolt_slot(a, b, 3)

	o = -15*Z
	b2 = screw(3, 11).transform(o)
	s2 = screw_slot(Axis(o,Z), 3, hole=11)

	b3 = bearing(16, 35, 11).transform(30*Z)
	s3 = bearing_slot_exterior(Axis(30*Z,Z), 35, 11, shouldering=True, circlip=True, evade=True)
	s4 = bearing_slot_interior(Axis(30*Z,Z), 16, 11, shouldering=True, circlip=False, evade=False)

	return [
		a, b,
		b1, s1,
		b2, s2,
		b3, s3, s4,
		]

@visualcheck
def test_sections():
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
		surface = flatsurface(section)
		surface.check()
		
	return [ section.transform(i*Z)  for i,section in enumerate(sections) ]
