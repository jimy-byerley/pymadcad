'''
	Module exposing many functions to create/visualize standard parts.
	Those are following the ISO (metric) specifications as often as possible.
	
	Most parts are as easy to create as:
	
		>>> nut(8)    # nut with nominal diameter 8mm
		>>> screw(8, 16)   # screw with match diameter 8mm, and length 16mm
		
	The parts usually have many optional parameters that default to usual recommended values. You can override this by setting the keyword arguments:
	
		>>> screw(8, 16, head='button', drive='slot')
'''

from operator import itemgetter

from .mathutils import *
from .primitives import *
from .kinematic import Solid
from .mesh import Mesh, Web, Wire, web, wire
from .generation import *
from .blending import *
from .boolean import pierce, union, difference, intersection
from .cut import *
from .io import cachefunc

#cachefunc = lambda x:x	# debug purpose


__all__ = [	'nut', 'screw', 'washer', 
			'coilspring_compression', 'coilspring_tension', 'coilspring_torsion',
			'bearing', 'slidebearing',
			'section_s', 'section_w', 'section_c', 'section_l', 'section_tslot',
			]


# --------- screw stuff -----------------------
	
	
@cachefunc
def screw(d, length, filet_length=None, head='SH', drive=None, detail=False) -> Solid:
	''' create a standard screw using the given drive and head shapes
	
	Parameters:
		d:             nominal diameter of the screw
		length:        length from the screw head to the tip of the screw
		filet_length:  length of the filet on the screw (from the tip), defaults to `length`
		
		head:          name of the screw head shape
		drive:			name of the screw drive shape
		detail:   if True, the thread surface will be generated, else it will only be a cylinder
		
	It's also possible to specify head and drive at once, using their codenames:
	
		>>> screw(5, 16, head='SHT')   # socket head and torx drive
		
	available screw heads:
		* socket (default)
		* button
		* flat (aka. cone)
		
	available screw drives:
		* hex
		* torx  *(not yet available)*
		* phillips (cross)  *(not yet available)*
		* slot
	
	All possible shapes:
		* see wikipedia for the [drive types](https://en.wikipedia.org/wiki/List_of_screw_drives)
		* see [here for a guide of what to use](https://www.homestratosphere.com/types-of-screws/)
		* see wikipedia for [standard screw thread](https://en.wikipedia.org/wiki/ISO_metric_screw_thread)
		
		
	'''
	if filet_length is None:	filet_length = length - 0.05*d
	elif length < filet_length:	raise ValueError('filet_length must be smaller than length')
	
	
	head, drive = screw_spec(head, drive)
	head = globals()['screwhead_'+head](d)
	if drive:
		drive = globals()['screwdrive_'+drive](d) .transform(boundingbox(head).max[2]*Z)
		head = intersection(head, drive)
		
	r = 0.5*d
	axis = Axis(O, Z, interval=(-length,r))
	body = revolution(2*pi, axis, wire([
					-vec3(r, 0, 0.05*d),
					-vec3(r, 0, length-filet_length),
					-vec3(r, 0, length-r*0.2),
					-vec3(r*0.8, 0, length),
					-vec3(0, 0, length),
					]) .segmented())
	screw = (body + head).finish()
	return Solid(part=screw, axis=axis)

def screwdrive_torx(d):
	indev
	
def screwdrive_hex(d):
	base = regon((-0.3*d*Z, -Z), 0.5*d, 6)
	socket = extrusion(d*Z, base) + blendloop(base, center=-0.6*d*Z, weight=-1)
	socket.mergeclose()
	return socket
	
def screwdrive_cross(d):
	indev
	
def screwdrive_slot(d):
	w = 0.15*d
	h = 0.3*d
	e = 2*d
	return extrusion(2*e*X, web([
				vec3(-e, w, h),
				vec3(-e, w, -h),
				vec3(-e, -w, -h),
				vec3(-e, -w, h),
				]) .segmented())

def screwhead_socket(d):
	''' screw head shape for socket head (SH) '''
	r = h = 0.7*d
	c = 0.05*d
	
	profile = wire([
			vec3(d/2,	0,	-c),
			vec3(d/2+c,	0,	0),
			vec3(r,     0,  0), 
			vec3(r,     0,  h-c),
			vec3(r-c,   0,  h),
			vec3(0,     0,  h),
			]) .segmented()
	head = revolution(-2*pi, (O,Z), profile)
	head.finish()
	return head
	
def screwhead_hex(d):
	''' screw head shape for hex head (HH) '''
	r = 0.9*d
	h = 0.6*d
	c = 0.05*d
	
	profile = extrusion(2*d*Z, regon((-d*Z,Z), r, 6))
	cone = revolution(2*pi, (O,Z), web([
		vec3(0,       0, h),
		vec3(0.8*r,   0, h),
		vec3(1.01*r,  0, h*0.8),
		vec3(1.01*r,  0, 0),
		vec3(0.5*d+c, 0, 0),
		vec3(0.5*d,   0, -c),
		]) .segmented())
	head = intersection(cone, profile)
	head.finish()
	return head
	
def screwhead_button(d):
	r = 0.95*d
	h = 0.5*d
	c = 0.05*d
	
	profile = [
		wire([
			vec3(0, 0, h), 
			vec3(0, 0.5*d, h),
			]),
		TangentEllipsis(
			vec3(0, 0.5*d, h), 
			vec3(0, 0.9*r, h),
			vec3(0, r, 0.1*d)),
		wire([
			vec3(0, r, 0.1*d),
			vec3(0, r, 0),
			vec3(0, 0.5*d+c, 0),
			vec3(0, 0.5*d, -c),
			]) .segmented(),
		]
	return revolution(2*pi, (O,Z), profile)
	
def screwhead_flat(d):
	r = d
	h = 0.5*d
	e = 0.1*d
	
	return revolution(2*pi, (O,Z), web([
		vec3(0, 0, h+e),
		vec3(0, r, h+e),
		vec3(0, r, h),
		vec3(0, 0.5*d, 0),
		]) .segmented() )
	
def screwhead_none(d):
	indev

	
'''
	head shapes:
	Abbreviation 	Expansion 	Comment
	BH 		button head 
	FH 		flat head 	
	OH 		oval head 	
	PH 		Phillips head 	
	RH 		round head 		
	FHP		flat head Phillips 	
	RHP 	round head Phillips 	
	SH 		socket head			Although "socket head" could logically refer to almost any female drive, it refers by convention to hex socket head unless further specified.
	SS 		set screw			The abbreviation "SS" more often means stainless steel. Therefore, "SS cap screw" means "stainless steel cap screw" but "SHSS" means "socket head set screw". As with many abbreviations, users rely on context to diminish the ambiguity, although this reliance does not eliminate it.
	VH		conic head
	
	CS 		cap screw 	
	MS 		machine screw 	
	
	BHCS 	button head cap screw 	
	BHMS 	button head machine screw 	
	FHCS 	flat head cap screw 
	FHSCS 	flat head socket cap screw 	
	FHPMS 	flat head Phillips machine screw 	
	FT 		full thread 	
	HHCS 	hex head cap screw 	
	HSHCS 	Hexalobular socket head cap screws 	
	RHMS 	round head machine screw 	
	RHPMS 	round head Phillips machine screw 	
	SBHCS 	socket button head cap screw 	
	SBHMS 	socket button head machine screw 	
	SHCS 	socket head cap screw 	
	SHSS 	socket head set screw 	Sometimes Socket Head Shoulder Screw.
	STS 	self-tapping screw
	
	[standard screw thread](https://en.wikipedia.org/wiki/ISO_metric_screw_thread)
'''

screwheads_codes = {
	'SH': 'socket',
	'HH': 'hex',
	'VH': 'flat',
	'cone': 'flat',
	'BH': 'button',
	'SBH': 'button',
	'SS': 'none',
	'FT': 'none',
	}
	
screwdrives_codes = {
	'TX': 'torx',
	'T': 'torx',
	'TR': 'torx',
	'CS': 'cap',
	'phillips': 'cross',
	'P': 'cross',
	'PH': 'cross',
	'PH': 'cross',
	}

def screw_spec(head, drive=None):
	if not drive:
		if head.isupper():
			for code in screwdrives_codes:
				if head.endswith(code):
					head, drive = head[:-len(code)], code
					break
	
	head = screwheads_codes.get(head, head)
	drive = screwdrives_codes.get(drive, drive)
	# special cases
	if head != 'hex' and not drive:
		drive = 'hex'
		
	return head, drive


# ------------------- nut stuff ----------------------

@cachefunc
def nut(d, type='hex', detail=False) -> Solid:
	''' create a standard nut model using the given shape type 
	
		Parameters:
			d:        nominal diameter of the matching screw
			type:     the nut shape
			detail:   if True, the thread surface will be generated, else it will only be a cylinder
	
		If `d` alone is given, the other parameters default to the ISO specs: https://www.engineersedge.com/iso_flat_washer.htm
		
		Currently only shape 'hex' is available.
	'''
	args = standard_hexnuts[bisect(standard_hexnuts, d, key=itemgetter(0))]
	if args[0] != d:
		raise ValueError('no standard nut for the given diameter')
	return hexnut(*args)

	
def hexnut(d, w, h) -> Solid:
	''' create an hexagon nut with custom dimensions '''
	# revolution profile
	w *= 0.5
	r = 1.01 * w/cos(radians(30))
	profile = wire([
		vec3(0.5*d,	0,	0.5*h),
		vec3(0.95*w,	0,	0.5*h),
		vec3(r,	0,	0.5*h - (r-w)),
		vec3(r,	0,	-0.5*h + (r-w)),
		vec3(0.95*w,	0,	-0.5*h),
		vec3(0.5*d,	0,	-0.5*h),
		]) .close() .segmented()
	base = revolution(2*pi, Axis(O,Z), web(profile))
	
	# exterior hexagon shape
	hexagon = regon((-h*Z,Z),  w/cos(radians(30)), 6)
	ext = extrusion(2*h*Z, hexagon)
	
	# intersect everything
	nut = intersection(base, ext)
	chamfer(nut, nut.frontiers(4,5) + nut.frontiers(0,5), ('width', d*0.1))

	nut.finish()
	return Solid(
				part=nut, 
				bottom=Axis(-0.5*h*Z, -Z, interval=(0,h)), 
				top=Axis(0.5*h*Z, Z, interval=(0,h)),
				)

''' iso hexagon nuts according to [EN ISO 4032](https://www.fasteners.eu/standards/ISO/4032/)
	columns:
	* thread
	* w
	* h
'''
standard_hexnuts = [
	(1.6,  3.2, 1.3),
	(2,    4,   1.6),
	(2.5,  5,   2),
	(3,    5.5, 2.4),
	(3.5,  6,   2.8), # non-prefered
	(4,    7,   3.2),
	(5,    8,   4.7),
	(6,    10,  5.2),
	(8,    13,  6.8),
	(10,   16,  8.4),
	(12,   18,  10.8),
	(14,   21,  12.8), # non-prefered
	(16,   24,  14.8),
	(18,   27,  15.8), # non-prefered
	(20,   30,  18),
	(22,   34,  19.4), # non-prefered
	(24,   36,  21.5),
	(27,   41,  23.8), # non-prefered
	(30,   46,  25.6),
	(33,   50,  38.7), # non-prefered
	(36,   55,  31),
	(39,   60,  33.4), # non-prefered
	(42,   65,  34),
	(45,   70,  36), # non-prefered
	(48,   75,  38),
	(52,   80,  42), # non-prefered
	(56,   85,  45),
	(60,   90,  48), # non-prefered
	(64,   95,  51),
	]
	
	
# -------------- washer stuff ----------------------
	
@cachefunc
def washer(d, e=None, h=None) -> Solid:
	''' create a standard washer.
		Washers are useful to offset screws and avoid them to scratch the mount part
		
		Parameters:
			d:        the nominal interior diameter (screw or anything else),
			           the exact washer interior is slightly bigger
			e:        exterior diameter
			h:        height/thickness
			
		If `d` alone is given, the other parameters default to the ISO specs: https://www.engineersedge.com/iso_flat_washer.htm
	'''
	if e is None and h is None:
		args = standard_washers[bisect(standard_washers, d, key=itemgetter(0))]
		if abs(args[0] - d)/d < 0.2:
			_, d, e, h = args
		else:
			raise ValueError('no standard nut for the given diameter')
	else:
		d *= 1.1
		if e is None:	e = d*2
		if h is None:	h = d*0.1
	
	surf = blendpair(
			Circle((O,-Z), d/2), 
			Circle((O,Z), e/2),
			tangents='straight',
			)
	return Solid(
				part=extrusion(h*Z, surf), 
				top=Axis(h*Z, Z, interval=(0,h)), 
				bottom=Axis(O, -Z, interval=(0,h)),
				)


''' metric washers according to https://www.engineersedge.com/iso_flat_washer.htm
	columns;
	* nominal screw
	* interior size
	* exterior size
	* thickness
'''
standard_washers	= [
	(1.6, 1.7, 4,   0.3),
	(2,   2.2, 5,   0.3),
	(2.5, 2.7, 6,   0.5),
	(2.6, 2.8, 7,   0.5),
	(3,   3.2, 7,   0.5),
	(3.5, 3.7, 8,   0.5),
	(4,   4.3, 9,   0.8),
	(5,   5.3, 10,  1),
	(6,   6.4, 12,  1.6),
	(7,   7.4, 14,  1.6),
	(8,   8.4, 16,  1.6),
	(10,  10.5, 20, 2),
	(12,  13,  24,  2.5),
	(14,  15,  28,  2.5),
	(16,  17,  30,  3),
	(18,  19,  34,  3),
	(20,  21,  37,  3),
	(22,  23,  39,  3),
	(24,  25,  44,  4),
	(27,  28,  50,  4),
	(30,  31,  56,  4),
	(33,  34,  60,  5),
	(36,  37,  66,  5),
	(39,  40,  72,  6),
	(42,  43,  78,  7),
	(45,  46,  85,  7),
	(48,  50,  92,  8),
	(52,  54,  98,  8),
	(56,  58,  105, 9),
	]
	
	
def section_s(height=1, width=None, flange=None, thickness=None) -> Web:
	''' standard S (short flange) section. Very efficient to support flexion efforts.
	'''
	if width is None:	width = 0.4 * height
	if flange is None:	flange = 0.036 * height
	if thickness is None:	thickness = 0.054 * height
	assert width > 3*thickness+2*flange and height > 2*flange+2*thickness
	
	base = wire([
		vec3(width/2, height/2, 0),
		vec3(width/2, height/2-flange, 0),
		vec3(thickness/2, height/2-flange-0.1*(width/2-thickness/2), 0),
		]) .flip()
	base = base + base.transform(scaledir(X,-1)).flip()
	base = base + base.transform(scaledir(Y,-1)).flip()
	section = web(base.close().segmented())
	bevel(section, section.frontiers(0,5,10,4,6,11), ('radius', thickness*0.4), resolution=('div',2))
	bevel(section, section.frontiers(1,0,4,3,6,7,10,9), ('radius', flange), resolution=('div',2))
	
	#notes = [
		#note_distance(base.points[0], base.points[0]*vec3(1,-1,1), offset=2*flange*X),
		#note_distance(base.points[0], base.points[1], offset=flange*X),
		#note_distance(base.points[0], base.points[0]*vec3(-1,1,1), offset=flange*Y),
		#note_distance(thickness/2*X, -thickness/2*X),
		#]
	
	return section.finish()

def section_w(height=1, width=None, flange=None, thickness=None) -> Web:
	''' standard W (wide flange) section. It is slightly different than a S section in that the flanges are straight and are usally wider.
	'''
	if width is None:	width = 0.6 * height
	if flange is None:	flange = 0.036 * height
	if thickness is None:	thickness = 0.054 * height
	assert width > 3*thickness+2*flange and height > 2*flange+2*thickness
	
	base = wire([
		vec3(width/2, height/2, 0),
		vec3(width/2, height/2-flange, 0),
		vec3(thickness/2, height/2-flange, 0),
		]) .flip()
	base = (base + base.transform(scaledir(X,-1)).flip() ).segmented()
	base = base + base.transform(scaledir(Y,-1)).flip()
	base.close()
	section = web(base)
	bevel(section, section.frontiers(0,5,10,4,6,11), ('radius', thickness*0.4), resolution=('div',2))
	bevel(section, section.frontiers(1,0,4,3,6,7,10,9), ('radius', flange*0.5), resolution=('div',2))
	return section.finish()
	
def section_l(a=1, b=None, thickness=None) -> Wire:
	''' standard L section '''
	if b is None:	b = a
	if thickness is None:	thickness = 0.05*max(a,b)
	assert a > 3*thickness and b > 3*thickness

	section = wire([
		vec3(0, 0, 0),
		vec3(0, a, 0),
		vec3(thickness, a, 0),
		vec3(thickness, thickness, 0),
		vec3(b, thickness, 0),
		vec3(b, 0, 0),
		]) .close() .segmented() .flip()

	bevel(section, [2,3,4], ('radius', thickness*0.8), resolution=('div',2))
	return section.finish()
	
def section_c(height=1, width=None, thickness=None) -> Web:
	''' standard C section '''
	if width is None:	width = 0.6*height
	if thickness is None:	thickness = 0.05*height
	assert width > 3*thickness
	
	base = wire([
		vec3(0, height/2, 0),
		vec3(width, height/2, 0),
		vec3(width, height/2-thickness, 0),
		vec3(thickness, height/2-thickness, 0),
		]) .flip()
	base = base + base.transform(scaledir(Y,-1)).flip()
	section = web(base.close().segmented())
	
	bevel(section, section.frontiers(0,1,5,7,6), ('radius', 0.8*thickness), resolution=('div',2))
	return section.finish()

def section_tslot(size=1, slot=None, thickness=None, depth=None) -> Web:
	''' standard T-Slot section. That section features slots on each side to put nuts at any position.
	'''
	if slot is None:	slot = 0.3*size
	if thickness is None:	thickness = 0.08*size
	if depth is None:	depth = 0.2*size
	b = depth-thickness
	
	center = Circle((O,-Z), size/2-depth-2*thickness)
	base = wire([
		vec3(size/2, size/2, 0),
		vec3(size/2, slot/2+thickness, 0),
		vec3(size/2-thickness, slot/2, 0),
		vec3(size/2-thickness, slot/2+b, 0),
		vec3(size/2-thickness-depth+b, slot/2+b, 0),
		vec3(size/2-thickness-depth, slot/2, 0),
		]) .flip()
	base = (base + base.transform(scaledir(normalize(vec3(1,-1,0)), -1)) .flip() ).segmented()
	base = base + base.transform(scaledir(X, -1)) .flip()
	base = base + base.transform(scaledir(Y, -1)) .flip()
	section = web([
		center,
		base.close().segmented(),
		])
	bevel(section, section.frontiers(5,7, 41,43, 31,29, 17,19), ('radius', thickness/2), resolution=('div',2))
	
	#notes = [
		#note_distance(base.points[2], base.points[5], offset=-c/2*Y),
		#note_distance(base.points[1], base.points[2], offset=-c/2*Y, project=X),
		#note_distance(base.points[2], base.points[2]*vec3(1,-1,1), offset=s*X),
		#note_distance(base.points[0], base.points[0]*vec3(1,-1,1), offset=2*s*X),
		#]
	
	return section.finish()
	
''' standard IPN sections (S sections)
columns:
	- h
	- b
	- s
	- t
'''
standard_ipn = [
	(80, 42, 3.9, 5.9),
	(100, 50, 4.5, 6.8),
	(120, 58, 5.1, 7.7),
	(140, 66, 5.7, 8.6),
	(160, 74, 6.3, 9.5),
	(180, 82, 6.9, 10.4),
	(200, 90, 7.5, 11.3),
	(220, 98, 8.1, 12.2),
	(240, 106, 8.7, 13.1),
	(260, 113, 9.4, 14.1),
	(280, 119, 10.1, 15.2),
	(300, 125, 10.8, 16.2),
	(320, 131, 11.5, 17.3),
	(340, 137, 12.2, 18.3),
	(360, 143, 13.0, 19.5),
	(380, 149, 13.7, 20.5),
	(400, 155, 14.4, 21.6),
	(450, 170, 16.2, 24.3),
	(500, 185, 18.0, 27.0),
	(550, 200, 19.0, 30.0),
	(600, 215, 21.6, 32.4),
	]
	
# --------------------- coilspring stuff ------------------------

@cachefunc
def coilspring_compression(length, d=None, thickness=None, solid=True) -> Solid:
	''' return a Mesh model of a croilspring meant for use in compression
	
		Parameters:
			length:     the distance between its two ends
			d:          the exterior diameter (the coilspring can fit in a cylinder of that diameter)
			thickness:  the wire diameter of the spring (useless if solid is disabled)
			solid:      disable it to get only the tube path of the coil, and have a `Wire` as return value
	'''
	if not d:			d = length*0.2
	if not thickness:	thickness = d*0.1
	r = d/2 - thickness		# coil radius
	e = r					# coil step
	div = settings.curve_resolution(d*pi, 2*pi)
	step = 2*pi/(div+1)
	
	t = 0
	
	t0, z0 = t, -0.5*length
	top = []
	for t in linrange(t0, t0 + 4*pi, step):
		top.append( vec3(r*cos(t), r*sin(t), z0 + (t-t0)/(2*pi) * thickness) )
	
	t0, z0 = t, -0.5*length + 2*thickness
	coil = []
	for t in linrange(t0, t0 + 2*pi * (length-4*thickness) / e, step):
		coil.append( vec3(r*cos(t), r*sin(t), z0 + (t-t0)/(2*pi) * e) )
	
	t0, z0 = t, 0.5*length - 2*thickness
	bot = []
	for t in linrange(t0, t0 + 4*pi, step):
		bot.append( vec3(r*cos(t), r*sin(t), z0 + (t-t0)/(2*pi) * thickness) )
		
	path = Wire(top) + Wire(coil).qualify('spring') + Wire(bot)
	
	if not solid:
		return path
	
	return Solid(
			part=tube(
				flatsurface(Circle(
					(path[0],Y), 
					thickness/2, 
					resolution=('div',6)
					)) .flip(), 
				path,
				),
			axis=Axis(O, Z, interval=(-length/2, length/2)),
			top=0.5*length*Z,
			bottom=-0.5*length*Z,
			)
	
@cachefunc
def coilspring_tension(length, d=None, thickness=None, solid=True) -> Solid:
	''' return a Mesh model of a croilspring meant for use in tension 
	
		Parameters:
			length:     the distance between its two hooks
			d:          the exterior diameter (the coilspring can fit in a cylinder of that diameter)
			thickness:  the wire diameter of the spring (useless if solid is disabled)
			solid:      disable it to get only the tube path of the coil, and have a `Wire` as return value
	'''
	if not d:			d = length*0.2
	if not thickness:	thickness = d*0.1
	r = d/2 - thickness		# coil radius
	e = r					# coil step
	
	# separate the coilspring in 3 parts:  the coil and the 2 hooks at both ides
	spring_length = length - 2*r
	ncoil = floor(spring_length / thickness) - 0.5
	hold = 0.5*length - r
	
	# create coil
	div = settings.curve_resolution(d*pi, 2*pi)
	step = 2*pi/(div+1)
	z0 = -0.5 * ncoil * thickness
	coil = Wire([	vec3(r*cos(t), r*sin(t), z0 + t/(2*pi) * thickness)
					for t in linrange(0, 2*pi * ncoil, step) ]) .qualify('spring')
	# create path with hooks
	path = wire([
				ArcCentered((-0.5*length*Z, X), vec3(0, -r, -0.5*length), -hold*Z),
				ArcThrough(-hold*Z, (-hold*Z + coil[0])*0.5 - 0.5*r*Y, coil[0]),
				coil,
				ArcThrough(coil[-1], (hold*Z + coil[-1])*0.5 - 0.5*r*Y, hold*Z),
				ArcCentered((0.5*length*Z, X), hold*Z, vec3(0, -r, 0.5*length)),
				])
	if not solid:
		return path
	
	return Solid(
			part=tube(
				flatsurface(Circle(
					(path[0],Z), 
					thickness/2, 
					resolution=('div',6)
					)), 
				path,
				),
			axis=Axis(O,Z, interval=(-length/2, length/2)),
			top=0.5*length*Z,
			bottom=-0.5*length*Z,
			)
	
@cachefunc
def coilspring_torsion(arm, 
			angle=radians(45), 
			d=None, 
			length=None, 
			thickness=None, 
			hook=None, 
			solid=True) -> Solid:
	''' return a Mesh model of a croilspring meant for use in torsion
	
		Parameters:
			arm:        the arms length from the coil axis
			length:     the coil length (and distance between its hooks)
			d:          the exterior diameter (the coilspring can fit in a cylinder of that diameter)
			thickness:  the wire diameter of the spring (useless if solid is disabled)
			hook:       the length of arm hooks (negative for hooks toward the interior)
			solid:      disable it to get only the tube path of the coil, and have a `Wire` as return value
	'''
	if not length:		length = arm*0.5
	if not d:			d = arm
	if not thickness:	thickness = d*0.1
	if not hook:		hook = -length
	r = d/2 - thickness		# coil radius
	e = r					# coil step
	angle = pi - angle
	
	# separate the coilspring in 3 parts:  the coil and the 2 hooks at both ides
	ncoil = ceil(length / thickness) + angle/(2*pi)
	
	# create coil
	div = settings.curve_resolution(d*pi, 2*pi)
	step = 2*pi/(div+1)
	z0 = -0.5 * ncoil * thickness
	coil = Wire([	vec3(-r*sin(t), r*cos(t), z0 + t/(2*pi) * thickness)
					for t in linrange(0, 2*pi * ncoil, step) ]) .qualify('spring')
				
	# create hooks
	c = thickness * sign(hook)
	if abs(c) > abs(hook):
		hook = c
	top = Wire([
			vec3(arm, 0, hook),
			vec3(arm, 0, c),
			vec3(arm-abs(c), 0, 0),
			vec3(0),
			]) .transform(coil[0])
	bot = (top 
			.flip() 
			.transform(scaledir(Z,-1)*scaledir(X,-1)) 
			.transform(angleAxis(angle, Z))
			)
	
	# create path
	path = top + coil + bot
	path.mergeclose()
	
	if not solid:
		return path
	
	return Solid(
			part=tube(
				flatsurface(Circle(
					(path[0], normalize(path[0]-path[1])),
					thickness/2,
					resolution=('div',6),
					)),
				path,
				),
			axis=Axis(O,Z, interval=(-length/2, length/2)),
			top=path[0],
			bottom=path[-1],
			)
		
		
		
# ----------------------- bearing stuff --------------------------

@cachefunc
def bearing(dint, dext=None, h=None, 
			circulating='ball', 
			contact=0, 
			hint=None, hext=None, 
			sealing=False, 
			detail=False) -> Solid:
	''' 
		Circulating bearings rely on rolling elements to avoid friction and widen the part life.
		Its friction depends on the rotation speed but not on the current load.
		
		see bearing specs at https://koyo.jtekt.co.jp/en/support/bearing-knowledge/
		
		Parameters:

			dint:	interior bore diameter
			dext:  exterior ring diameter
			h:     total height of the bearing
			
			hint:  height of the interior ring. Only for angled roller bearings
			hext:  height of the exterior ring. Only for angled roller bearings
		
			circulating:
			
				The type of circulating element in the bearing
				
				- ball
				- roller
				
			contact:
			
				Contact angle (aka pressure angle).
				It decided what directions of force the bearing can sustain.
				
					- `0` for radial bearing
					- `pi/2`  for thrust (axial) bearings
					- `0 < contact < pi/2` for conic bearings
					
			sealing:  True if the bearing has a sealing side. Only for balls bearings with `contact = 0`
					
			detail:   If True, the returned model will have the circulating elements and cage, if False the returned element is just a bounding representation
	'''
	if isinstance(dint, str):
		dint, dext, h = bearing_spec(dint)
	else:
		if not dext:	dext = 2*dint
		if not h:		h = 0.5*dint
		
	assert 0 < h <= dint < dext
	assert 0 <= contact

	if circulating == 'roller':
		if sealing:	
			raise ValueError('sealing must be None for roller bearings')
		return bearing_roller(dint, dext, h, contact, hint, hext, detail)
	
	elif circulating == 'ball':
		if hint or hext:
			raise ValueError('hint and hext must be None for ball bearings')
		if abs(contact) < NUMPREC:
			return bearing_ball(dint, dext, h, sealing, detail)
		if abs(contact - 0.5*pi) < NUMPREC:
			if sealing:
				raise ValueError('sealing must be none for thrust bearings')
			return bearing_thrust(dint, dext, h, detail)
		else:
			raise ValueError('ball bearings must not be angled')
			
	else:
		raise ValueError('unknown circulating element {}'.format(repr(circulating)))
	
	
def bearing_spec(code):
	# iso code
	if ' ' in code:
		indev
	# simple code
	elif re.match(r'[\d\.]+x[\d\.]+x[\d\.]+', code):
		return tuple(float(d) for d in re.split('x'))
		
bearing_color = vec3(0.5,0.4,0.35)
bearing_cage_color = vec3(0.3,0.2,0)
bearing_circulating_color = vec3(0,0.1,0.2)
		
		
def bearing_ball(dint, dext=None, h=None, sealing=False, detail=False) -> Solid:
	# convenient variables
	rint = dint/2
	rext = dext/2
	c = 0.05*h
	w = 0.5*h
	e = 0.15*(dext-dint)

	# outer rings profiles
	axis = Axis(O,Z, interval=(0,h))
	interior = Wire([
		vec3(rint+e, 0,	w), 
		vec3(rint, 0, w),
		vec3(rint,	0,	-w),
		vec3(rint+e, 0,	-w), 
		]) .segmented() .flip()
	bevel(interior, [1, 2], ('radius',c), resolution=('div',1))

	exterior = Wire([
		vec3(rext-e,	0, -w),
		vec3(rext, 0, -w),
		vec3(rext, 0, w),
		vec3(rext-e,	0, w),
		]) .segmented() .flip()
	bevel(exterior, [1,2], ('radius',c), resolution=('div',1))

	if detail and not sealing:
		rb = (dint + dext)/4	# balls path radius
		rr = 0.75*(dext - dint)/4 # balls radius

		hr = sqrt(rr**2 - (rb-rint-e)**2)	# half balls inprint width in the rings
		interior += wire(ArcCentered((rb*X,-Y), vec3(rint+e, 0, hr), vec3(rint+e, 0, -hr)))
		exterior += wire(ArcCentered((rb*X,-Y), vec3(rext-e, 0, -hr), vec3(rext-e, 0, hr)))
		interior.close()
		exterior.close()
		part = revolution(2*pi, axis, web([exterior, interior])) .option(color=bearing_color)

		nb = int(0.7 * pi*rb/rr)	# number of balls that can fit in
		balls = repeat(icosphere(rb*X, rr), nb, angleAxis(radians(360)/nb, Z)) .option(color=vec3(0,0.1,0.2))
		
		# balls cage (simplified version)
		cage_profile = Wire([vec3(
						rb*cos(t), 
						rb*sin(t), 
						(rr+0.5*c) * (1 - (0.5-0.5*cos(nb*t))**2) + c)
					for t in linrange(0, 2*pi, div=nb*20-1)])
		cage_profile.mergeclose()

		surf = extrusion(
				mat3(
					(rb-rr*0.6)/rb, 
					(rb-rr*0.6)/rb, 
					1), 
				cage_profile, 
				alignment=0.5)

		cage = thicken(
				surf + surf.transform(mat3(1,1,-1)) .flip(), 
				c) .option(color=bearing_cage_color)
				
		# asemble
		return Solid(part=part, cage=cage, balls=balls, axis=axis)

	else:
		# assemble and close rings profiles
		interior = (
			Wire([
				exterior[-1], 
				exterior[-1]+c*Z, 
				interior[0]+c*Z, 
				interior[0]]) .segmented()
			+ interior
			+ Wire([
				interior[-1], 
				interior[-1]-c*Z, 
				exterior[0]-c*Z, 
				exterior[0]]) .segmented()
			)
		return Solid(
				part=revolution(2*pi, axis, web([exterior, interior]))
						.option(color=bearing_color), 
				axis=axis)


def bearing_roller(dint, dext=None, h=None, contact=0, hint=None, hext=None, detail=False) -> Solid:
	# interior and exterior heights (automatically deduced from total height if not specified)
	if not hint:	hint = h*cos(contact)
	if not hext:	hext = h*cos(contact) * 0.8 if contact else h
	
	assert 0 < hint <= h
	assert 0 < hext <= h

	# convenient variables
	rint = dint/2
	rext = dext/2
	c = 0.05*h
	w = 0.5*h
	e = 0.08*(dext-dint)
	axis = Axis(O,Z, interval=(0,h))
	
	# cones definition points
	if contact:
		# axis on a cone using the given contact angle
		cr = vec3(mix(rint, rext, 0.5), 0, 0)
		ct = -cr.x / tan(contact) *Z
		angled = Axis(ct, vec3(sin(contact), 0, cos(contact)))

		p1 = vec3(rext-e, 0, -w+hext)
		p2 = vec3(rint+e, 0, w-hint+e)
		a1 = Axis(p1, normalize(p1-ct))
		a2 = Axis(p2, normalize(p2-ct))

		p3 = angled[0] - project(reflect(p2-angled[0], angled[1]), a1[1])

		p4 = angled[0] - reflect(p3-angled[0], angled[1])
		p5 = angled[0] - reflect(p1-angled[0], angled[1])
	else:
		# cylindrig bearing: infinite cone
		p1 = vec3(rext-e, 0, -w+hext-e)
		p3 = vec3(rext-e, 0, -w+e)
		p4 = vec3(rint+e, 0, w-hint+e)
		p5 = vec3(rint+e, 0, -w+hext-e)
		angled = Axis(0.5*(rint+rext) * X, Z)

	# ring profiles
	interior = Wire([
		p5+e*X,
		vec3(p5[0]+e, 0, w),
		vec3(rint, 0, w),
		vec3(rint,	0,	w-hint),
		vec3(p4[0]+e, 0, w-hint),
		p4+e*X,
		]) .segmented()
	exterior = Wire([
		p3,
		vec3(p3[0], 0, -w),
		vec3(rext, 0, -w),
		vec3(rext, 0, -w+hext),
		vec3(rext-e,	0, -w+hext),
		]) .segmented()
	bevel(interior, [2,3], ('radius',c), resolution=('div',1))
	bevel(exterior, [2,3], ('radius',c), resolution=('div',1))
	
	# create interior details
	if detail:
		# complete rings with their interiors
		interior += Wire([
			p4,
			p5,
			p5+e*X,
			]) .segmented()
		exterior += Wire([p3])

		part = revolution(2*pi, axis, web([
					exterior, 
					interior,
					]).flip() ) .option(color=bearing_color)
	
		# create conic rollers
		roller = revolution(2*pi, angled, Segment(mix(p1,p3,0.05), mix(p3,p1,0.05)))
		for hole in roller.outlines().islands():
			roller += flatsurface(wire(hole))

		# number of rollers that can fit in
		nb = int(pi*(rint+rext) / (2.5*distance_pa(p1,angled)))
		rollers = repeat(roller, nb, rotatearound(2*pi/nb, axis)) 
		rollers.option(color=bearing_circulating_color)

		# roller cage
		p6 = mix(p4,p3,0.6) - e*angled[1]
		cage_profile = wire([
			p6 - 1.5*e*X,
			p6,
			p6 + (distance(p1,p3)+1.8*e) * angled[1],
			])
		bevel(cage_profile, [1], ('radius',c))
		cage = revolution(2*pi, axis, cage_profile)
		cage = pierce(cage, inflate(rollers, 0.5*c), False)
		cage = thicken(cage, c) .option(color=bearing_cage_color)
		
		# assemble
		return Solid(part=part, cage=cage, rollers=rollers, axis=axis)
	
	# simply create a bounding representation
	else:
		# assemble and close rings profiles
		return Solid(
				part=revolution(2*pi, axis, 
					(exterior + interior) .close() .flip()
					) .option(color=bearing_color), 
				axis=axis)
		

def bearing_thrust(dint, dext, h, detail=False) -> Solid:
	# convenient variables
	rint = dint/2
	rext = dext/2
	c = 0.05*h	# corner radius
	w = 0.5*h	# half height
	e = 0.12*(dext-dint) # outer rings thickness

	# rings outlines
	axis = Axis(O,Z, interval=(0,h))
	top = Wire([
		vec3(rext, 0, w-e), 
		vec3(rext, 0, w),
		vec3(rint, 0, w),
		vec3(rint, 0, w-e), 
		]) .segmented() .flip()
	bevel(top, [1, 2], ('radius',c), resolution=('div',1))

	bot = Wire([
		vec3(rint, 0, -w+e),
		vec3(rint, 0, -w),
		vec3(rext, 0, -w),
		vec3(rext, 0, -w+e),
		]) .segmented() .flip()
	bevel(bot, [1,2], ('radius',c), resolution=('div',1))

	if detail:
		rb = (dint + dext)/4	# balls guide radius
		rr = 0.75*h/2		# ball radius
		
		hr = sqrt(rr**2 - (w-e)**2)		# half ball inprint width in the rings
		# complete the rings with their interior
		top += wire(ArcCentered((rb*X,-Y), vec3(rb+hr, 0, w-e), vec3(rb-hr, 0, w-e)))
		bot += wire(ArcCentered((rb*X,-Y), vec3(rb-hr, 0, -w+e), vec3(rb+hr, 0, -w+e)))
		top.close()
		bot.close()
		part = revolution(2*pi, axis, web([top, bot])) .option(color=bearing_color)
		
		# number of balls to place
		nb = int(0.8 * pi*rb/rr)
		balls = repeat(icosphere(rb*X, rr), nb, angleAxis(radians(360)/nb, Z))
		balls.option(color=bearing_circulating_color)
		
		# cage
		cage_profile = Wire([ 
			vec3(rext-c, 0, -w+e+0.1*h),
			vec3(rext-c, 0, w-e-0.1*h),
			vec3(rint+c, 0, w-e-0.1*h),
			vec3(rint+c, 0, -w+e+0.1*h),
			])
		bevel(cage_profile, [1,2], ('radius',c), resolution=('div',1))
		
		cage_surf = revolution(2*pi, axis, cage_profile)
		cage_surf = pierce(cage_surf, inflate(balls, 0.2*c), False)
		cage = thicken(cage_surf, c) .option(color=bearing_cage_color)
		
		# assemble
		return Solid(part=part, cage=cage, balls=balls, axis=axis)
		
	else:
		# assemble and close the profiles
		top = (
			Wire([
				bot[-1], 
				bot[-1]+c*X, 
				top[0]+c*X, 
				top[0]]) .segmented()
			+ top
			+ Wire([
				top[-1], 
				top[-1]-c*X, 
				bot[0]-c*X, 
				bot[0]]) .segmented()
			)

		return Solid(
				part=revolution(2*pi, axis, web([top, bot])) .option(color=bearing_color), 
				axis=axis)

	
'''
	all existing bearing dimensions according to https://koyo.jtekt.co.jp/en/support/bearing-knowledge/6-3000.html
'''
standard_bearing_ball_straight = [
	(16, 35, 11),
	]



from .selection import *

@cachefunc
def slidebearing(dint, h=None, thickness=None, shoulder=None, opened=False) -> Solid:
	'''
		Slide bearings rely on gliding parts to ensure a good pivot. It's much cheaper than circulating bearings and much more compact. But needs lubricant and has a shorter life than circulating bearings.
		Its friction depends on the rotation speed and on the load.
	
		Parameters:
			dint:       interior diameter
			h:          exterior height (under shoulder if there is)
			thickness:  shell thickness, can be automatically determined
			shoulder:   distance from bore to shoulder tip, put 0 to disable
			opened:     enable to have a slight breach that allows a better fitting to the placement hole 
	'''
	if h is None:			h = 1.2*dint
	if thickness is None:	thickness = 0.05*dint
	rint = dint/2
	
	profile = Wire([
				vec3(rint,0,-h),
				vec3(rint,0, 0),
				])
	if shoulder:
		profile += Wire([ vec3(rint+shoulder, 0, 0) ])
		profile = profile.segmented()
		bevel(profile, [1], ('radius', 1.5*thickness), resolution=('div',2))

	axis = Axis(O,Z, interval=(-h,0))
	shape = revolution(1.98*pi if opened else 2*pi, axis, profile)
	
	part = thicken(shape, thickness) .option(color=bearing_color)
	line = (  select(part, vec3(-rint,0,-h), stopangle(pi/2))
			+ select(part, vec3(dint,dint,-h), stopangle(pi/2))
			)
	if not shoulder:
		line += (  select(part, vec3(-rint,0,0), stopangle(pi/2))
				 + select(part, vec3(dint,dint,0), stopangle(pi/2))
				)
	
	chamfer(part, line, ('width',0.5*thickness))
	return Solid(part=part, axis=axis)

'''
* profilés
	+ ipn
	+ (carrés a rails)
	+ U
	+ L
* nut
	+ carrés
	+ hex
	+ auto-stopable
	+ fermés au bout
	+ support
* screw
	+ forme tete (UH, BH, VH, ...)
	+ forme empreinte (torx, hex, crux, line, ...)
* rondelle
	+ simple
	+ stobable
	+ ressort
	+ entretoise
* roulement
	+ bille, rouleau
	+ droit, oblique, radial
	+ etancheité
* coussinet
	+ droit
	+ a epaule
	+ fendu
* anneaux elastiques
	+ circlip interieur/exterieur
	+ ressort
	
* helice
	+ rotor de ventilateur ...
	
* vis/ecrou
	+ a billes
	+ a profil trapezoidal
* accouplements
	+ ressort
	+ encastrement: prisme, cannelures, clavete, meplat
* ressorts
	+ lineaire
	+ extension
	+ conique
	+ pli
	+ spiral
	+ concentrique
	+ lame
	
* tuyau
	+ embouchure
	+ droit/coude
	+ le long d'un chemin ?
	
pour tout:
	- liste des tailles standard
'''
	
