from madcad import *
from madcad.generation import matchclosest
from madcad.kinematic import makescheme

if False:
	l1 = Wire([vec3(-2,0,0), vec3(-1,0,0), vec3(0,0,0), vec3(1,0,0), vec3(2,0,0)])
	l2 = Wire([vec3(-3,0,-1), vec3(-0.9,1,-2), vec3(0,0,-2), vec3(1.5,-1,-1)])
	w1 = web(l1)
	w2 = web(l2)
	w1.options['color'] = (1,0.5,0.3)
	w2.options['color'] = (1,0.5,0.3)

	# test junction
	m5 = junction(
		matchclosest(
			l1,
			l2,
		))
	m5.check()
	assert m5.issurface()
	#m4.options.update({'debug_display': True, 'debug_points': True})
	
	show([m5, w1, w2], {'display_points':True, 'display_wire':True, 'display_groups':False})

if False:
	O = vec3(0)
	Z = vec3(0,0,1)
	Y = vec3(0,1,0)
	A = vec3(2,0,0)
	m = revolution(
		radians(180),       # 180 degrees converted into radiaus
		(O,Z),              # revolution axis
		web(Circle((A,Y), 0.5)),
		)
	
	show([m, (O,Z), A], {'display_points':False, 'display_wire':True})

if False:
	O = vec3(0)
	X = vec3(1,0,0)
	Y = vec3(0,1,0)
	Z = vec3(0,0,1)

	# we define the solids, they intrinsically have nothing particular
	base = Solid()
	s1 = Solid()
	s2 = Solid()
	s3 = Solid()
	s4 = Solid()
	s5 = Solid()
	wrist = Solid(name='wrist')     # give it a fancy name

	# the joints defines the kinematic.
	# this is a 6 DoF (degrees of freedom) robot arm
	csts = [
			Pivot(base,s1, (O,Z)),                   # pivot using axis (O,Z) both in solid base and solid 1
			Pivot(s1,s2, (vec3(0,0,1), X), (O,X)),   # pivot using different axis coordinates in each solid
			Pivot(s2,s3, (vec3(0,0,2), X), (O,X)),
			Pivot(s3,s4, (vec3(0,0,1), Z), (vec3(0,0,-1), Z)),
			Pivot(s4,s5, (O,X)),
			Pivot(s5,wrist, (vec3(0,0,0.5), Z), (O,Z)),
			]

	# the kinematic is created with some fixed solids (they interact but they don't move)
	kin = Kinematic(csts, fixed=[base])
	
	makescheme(csts)
	show([kin])
	
if True:
	# define points
	O = vec3(0)
	A = vec3(2,0,0)
	B = vec3(1,2,0)
	C = vec3(0,2,0)
	# create a list of primitives
	line = [
			Segment(O, A),          # segment from 0 to A (the direction is important for the surface generation)
			ArcThrough(A, B, C), # arc from A to C, with waypoint B
			Segment(C,O),           # segment from C to O
			]
	csts = [
        Tangent(line[0], line[1], A),   # segment and arc are tangent in A
        Tangent(line[1], line[2], C),   # arc and segment are tangent in C
        Radius(line[1], 1.5),           # radius of arc must be equal to 1.5
        ]
	#solve(csts, fixed=[0]) 
	
	show([line, O, A, B, C])

