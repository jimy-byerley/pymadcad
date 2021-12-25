# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module provide functions to generate racks and gears with many different shapes.

	The mainstream gears are involute gears (using involute of circles as contact curves). This is due to the standard shape of racks. And involute gears are the best choice for most designs, which is why the current implementation focusses on them.

	However the tools here are modular, which means you can combine them with your own functions to create customized gears.

	Examples
	--------

	As a first use, you may want to generate a fully finished spur gear
	If you do not specify optional arguments, the function will provide good defaults for it.

		>>> # fully featured gear
		>>> gear(step=3, z=12, depth=4, bore_radius=1.5)

	You may want to generate a more bizarre gear, with a a different hub for instance. You will then need to assemble a gear from its 3 components:  exterior (tooths), structure (between hub and exterior), and hub (what holds the gear on an axis)

		>>> # this assemble a gear specifying each independant sub-parts
		>>> ext_radius = (3*12)/(2*pi) - 3
		>>> int_radius = 4
		>>> geargather(
		...		gearexterior(repeat_circular(gearprofile(3, 30), 30), depth=4),
		...		gearstructure('rounded', ext_radius, int_radius, 2, patterns=6),
		...		my_hub_mesh,
		...		)

	For reuse in your custom functions, the functions used to generate the gears are exposed:

		>>> # this is the raw profile of a tooth
		>>> gearprofile(step=3, z=12)
'''

from .mathutils import *
from .mesh import Web, Wire, Mesh, web, wire
from .blending import junction, blendpair
from .generation import extrusion, revolution, repeat, extrans
from .primitives import Circle, ArcCentered, Segment
from .triangulation import triangulation
from .selection import *
from .rendering import show
from .cut import bevel, chamfer
from .boolean import intersection
from . import settings

from math import *
from functools import reduce, partial
from operator import add
from copy import copy



def rackprofile(step, h=None, offset=0, alpha=radians(20), resolution=None) -> Wire:
	''' Generate a 1-period tooth profile for a rack

		Parameters:

			step:		period length over the primitive line
			h:		    tooth half height
			offset:     rack reference line offset with the primitive line (as a distance)
			              - the primitive line is the adherence line with gears
			              - the reference line is the line half the tooth is above and half below
			alpha:		angle of the tooth sides, a.k.a  pressure angle of the contact
	'''
	if h is None:
		h = default_height(step, alpha)
	e = offset  # change name for convenience
	x = 0.5 + 2*e/step*tan(alpha)  # fraction of the tooth above the primitive circle

	return Wire([
		vec3(step*x/2 - tan(alpha) * ( h-e),   h-e,  0),
		vec3(step*x/2 - tan(alpha) * (-h-e),  -h-e,  0),
		vec3(step*(2-x)/2 + tan(alpha) * (-h-e),  -h-e,  0),
		vec3(step*(2-x)/2 + tan(alpha) * ( h-e),   h-e,  0),
		vec3(step*(2+x)/2 - tan(alpha) * ( h-e),   h-e,  0),
		], groups=['rack'])


def gearprofile(step, z, h=None, offset=0, alpha=radians(20), resolution=None, **kwargs) -> Wire:
	''' Generate a 1-period tooth profile for a straight gear


		Parameters:

			step:		period length over the primitive circle
			z:			number of tooth on the gear this profile is meant for
			h:			tooth half height
			offset:		offset distance of the matching rack profile (see above)
			alpha:		pressure angle in the contact
	'''
	if h is None:
		h = default_height(step, alpha)
	p = step*z / (2*pi)	# primitive circle
	c = p * cos(alpha)	# tangent circle to gliding axis

	e = offset  # change name for convenience
	#e = offset + 0.05*h  # the real offset is 5% (the 1*m, 1.25*m) when the standard says it is 0
	x = 0.5 + 2*offset/step*tan(alpha)  # fraction of the tooth above the primitive circle

	o0 = angle(involute(c, 0, tan(alpha)))	# offset of contact curve
	oi = atan((h-e)/p * tan(alpha))		# offset of interference curve

	l0 = involuteat(c, p+h+e)	# interval size of contact curve

	# if the tooth height is unreachable, find the intersection between the two contact curves
	t0 = -step/(2*p)*x - o0
	t = t0+l0
	if involute(c, t0, t)[1] > 0:
		# Newton solver method
		for i in range(5):
			f = sin(t) + cos(t)*(t0-t)
			J = - sin(t)*(t0-t)
			t -= f/J
		l0 = t-t0

	# if there is an interference curve
	interference = c > p-h+e
	if interference:
		# Newton solver method
		# to compute the parameters (t1, t2) of the intersection between contact line and interference line
		t0, ti = o0, oi
		# initial state
		t1 = t0 - tan(alpha)	# put contact line on primitive
		t2 = ti + sqrt(c**2 - (p-h+e)**2) /p	# put interference point on base circle
		for i in range(8):
			ct1, ct2 = cos(t1), cos(t2)
			st1, st2 = sin(t1), sin(t2)
			# function value
			f = (	c*vec2(ct1,st1) + c*(t0-t1)*vec2(-st1,ct1)
				+	(h-e-p)*vec2(ct2,st2) -p*(ti-t2)*vec2(-st2,ct2)
				)
			# jacobian matrix (f partial derivatives)
			J = mat2(
				-c*(t0-t1)*vec2(ct1,st1),
				p*(ti-t2)*vec2(ct2,st2) + (h-e)*vec2(-st2,ct2),
				)
			# iteration
			t1, t2 = vec2(t1,t2) - inverse(J)*f
		li = t2 - ti	# interval size of interference curve
		s0 = t0 - t1	# generation start of contact curve
	else:
		s0 = involuteat(c, p-h+e)

	pts = []
	n = 2 + settings.curve_resolution(h, step/p, resolution)	# number of points to place

	# parameter for first side
	place = step/(2*p)*x	# place of intersection with the primitive circle
	t0 = place + o0	# start of contact curve
	ti = place + oi	# start of interference curve
	# contact line
	for i in range(n+1):
		t = interpol1(t0-l0, t0-s0, i/n)
		v = involute(c, t0, t)
		pts.append(vec3(v,0))
	# interference line
	if interference:
		for i in range(n+1):
			t = interpol1(ti+li, ti, i/n)
			v = involuteof(p, ti, -h+e, t)
			pts.append(vec3(v,0))

	# parameters for second side
	place = step/(2*p)*(2-x)
	t0 = place - o0
	ti = place - oi
	# interference line
	if interference:
		for i in range(n+1):
			t = interpol1(ti, ti-li, i/n)
			v = involuteof(p, ti, -h+e, t)
			pts.append(vec3(v,0))
	# contact line
	for i in range(n+1):
		t = interpol1(t0+s0, t0+l0, i/n)
		v = involute(c, t0, t)
		pts.append(vec3(v,0))

	pts.append(angleAxis(step/p, vec3(0,0,1)) * pts[0])

	return Wire(pts, groups=['gear'])

def gearcircles(step, z, h=None, offset=0, alpha=radians(30)):
	''' return the convenient circles radius for a gear with the given parameters
		  return is `(primitive, base, bottom, top)`
	'''
	if h is None:
		h = default_height(step, alpha)
	e = h*e
	p = step*z / (2*pi)	# primitive circle
	c = p * cos(alpha)	# tangent circle to gliding axis
	return p, c, p-h-e, p+h-e

def default_height(step, alpha):
	return 0.5 * 2.25 * step/pi * cos(alpha)/cos(radians(20))

def involute(c, t0, t):
	''' give a point of parameter `t` on involute from circle or radius `c`, starting from `t0` on the circle

		`t` and `t0` are angular positions
	'''
	x = vec2(cos(t), sin(t))
	y = vec2(-x[1], x[0])
	return c*(x + y*(t0-t))

def involuteat(c, r):
	''' give the parameter for the involute of circle radius `c` to reach radius `r` '''
	return sqrt((r/c)**2 - 1)

def involuteof(c, t0, d, t):
	''' give a point of parameter `t` on involute with offset, from circle or radius `c`, starting from `t0` on the circle

		`t` and `t0` are angular positions
	'''
	x = vec2(cos(t), sin(t))
	y = vec2(-x[1], x[0])
	return (c+d)*x + c*y*(t0-t)


def angle(p):
	return atan2(p[1], p[0])


def repeat_circular(profile, n: int) -> Wire:
	''' Repeat n times the given Wire by rotation around (O,Z) '''
	result = repeat(profile, n, rotatearound(
		anglebt(noproject(profile[0],Z), noproject(profile[-1],Z)), 
		(O,Z),
		))
	result.mergeclose()
	return result


def pattern_full(ext_radius, int_radius, depth, int_height=0, **kwargs) -> Mesh:
	"""
	Generate two full parts of the structure (the top and the bottom).

	Return a tuple (Web, Web, None) where the first `Web` is the top of the structure and
	the second `Web` is the bottom of the structure.

	Parameters:

		ext_radius (float): float (radius of the external border of the pattern
		int_radius (float): radius of the internal border of the pattern
		depth (float): face width
		int_height (float):

			if you want a pinion with a structure thinner than the value of `depth`,
			the total height will be `total_height = depth - 2 * int_height`
	"""
	half_depth = depth / 2
	assert half_depth > int_height, "`int_height` must be smaller than `depth / 2`"

	axis = (O, Z)
	circles_ref = web(Circle(axis, ext_radius)) + web(Circle(axis, int_radius)).flip()
	top_profile = circles_ref.transform(vec3(0, 0, half_depth - int_height))
	bottom_profile = circles_ref.transform(vec3(0, 0, -half_depth + int_height)).flip()
	mesh = triangulation(top_profile) + triangulation(bottom_profile)
	mesh.mergeclose()
	return mesh


def pattern_circle(
	ext_radius,
	int_radius,
	depth,
	int_height = 0,
	ratio = 1,
	patterns: int = 5,
	circles_radius = None,
	circles_place = None,
	**kwargs,
) -> Mesh:
	"""
	Generate two parts of the structure (the top and the bottom) with `patterns` distributed on the whole structure.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters:

		ext_radius (float): radius of the external border of the structure
		int_radius (float): radius of the internal border of the structure
		depth (float): face width
		int_height (float): if you want a pinion with a structure thinner than the value of `depth`,
							the total height will be `total_height = depth - 2 * int_height`
		ratio (float): number that allows to change proportionally the radius of circles
		patterns (int): number of circles of the structure
		circles_radius (float): radius of circles
		circles_place (float): radius where the origins of circles are placed

	Note:

		- for instance, with a ratio of 1.5, the radius of circles `circles_radius` will be divided by 1.5
		- if `circles_radius` is chosen, `ratio` won't impact the radius of circles `circles_radius`
	"""
	# Parameters
	half_depth = depth / 2
	if not (circles_radius) and not (circles_place):
		assert 0.5 < ratio, "`ratio` must be in the interval ]0.5; +inf["
	if not circles_place:
		circles_place = (ext_radius + int_radius) / 2
	if not circles_radius:
		circles_radius = (ext_radius - int_radius) / (4 * ratio)

	assert circles_radius / circles_place < sin(pi / patterns), "too many circles for the specified size. Change either 'ratio', 'circle_radius' or 'patterns'"

	Z = vec3(0,0,1)
	axis = (vec3(0,0,0), Z)

	# Borders
	ext_circle = web(Circle(axis, ext_radius))
	int_circle = web(Circle(axis, int_radius)).flip()

	# Pattern
	circle_ref = web(Circle((vec3(circles_place, 0, 0), Z), circles_radius))
	angle = 2 * pi / patterns
	pattern_profile = repeat(circle_ref.flip(), patterns, rotatearound(angle, axis))

	# Profiles and surfaces
	webs_ref = (pattern_profile, ext_circle, int_circle)
	top_webs = [wire.transform(vec3(0, 0, half_depth - int_height)) for wire in webs_ref]
	bottom_webs = [wire.transform(vec3(0, 0, -half_depth + int_height)).flip() for wire in webs_ref]

	top_profile = reduce(add, top_webs)
	bottom_profile = reduce(add, bottom_webs)
	surfaces = extrusion(vec3(0, 0, depth - 2 * int_height), bottom_webs[0].flip())

	mesh = triangulation(top_profile) + triangulation(bottom_profile) + surfaces
	mesh.mergeclose()
	return mesh


def create_pattern_rect(
	ext_radius,
	int_radius,
	depth,
	int_height = 0,
	ratio = 1,
	patterns: int = 5,
	ext_thickness = None,
	int_thickness = None,
	rounded: bool = False,
	**kwargs,
) -> Mesh:
	"""
	Function used for `pattern_rect` when `pattern` = "rect"
	and `pattern_rounded` when `pattern` = "rounded"

	Parameters:

		ext_radius (float): radius of the external border of the structure
		int_radius (float):  radius of the internal border of the structure
		depth (float): face width
		int_height (float): if you want a pinion with a structure thinner than the value of `depth`,
							the total height will be `total_height = depth - 2 * int_height`
		ratio (float): it is a number which is the proportional factor for an homothety of patterns
		patterns (int): number of patterns inside the structure
		ext_thickness (float): internal radius of the pattern
		int_thickness (float): external radius of the pattern
		rounded (bool): if it is True, the pattern will be rounded

	Note:

		- for instance, if `ratio` is 1.5, the area of the pattern will be divided by 1.5.
		- if `r_int` and `r_ext` are chosen, `ratio` won't impact these parameters
	"""
	# Parameters
	half_depth = depth / 2
	offset = ratio * 0.05 * (ext_radius - int_radius)
	r_ext = ext_radius - (ext_thickness or offset)
	r_int = int_radius + (int_thickness or offset)
	thickness = ratio * 0.15 * r_ext  # thickness of "arms"
	# angle between the bottom edge of an arm and the middle of it
	theta_1 = asin(thickness / (2 * r_int))
	# angle between the top edge of an arm and the middle of it
	theta_2 = asin(thickness / (2 * r_ext))
	angle_step = 2 * pi / patterns
	O = vec3(0)
	Z = vec3(0, 0, 1)
	axis = (O, Z)

	# Borders
	ext_circle = web(Circle(axis, ext_radius))
	int_circle = web(Circle(axis, int_radius)).flip()

	#function to convert cylindrical coordinates to cartesian coordinates
	cyl2cart = lambda r, theta: vec3(r * cos(theta), r * sin(theta), 0)
	# Vertex points of the pattern
	A1 = cyl2cart(r_int, theta_1)
	A2 = cyl2cart(r_ext, theta_2)
	B1 = cyl2cart(r_int, angle_step - theta_1)
	B2 = cyl2cart(r_ext, angle_step - theta_2)

	# Primitives
	pattern_ref = web([
					Segment(B2, B1),
					ArcCentered((O, -Z), B1, A1),
					Segment(A1, A2),
					ArcCentered((O, Z), A2, B2),
					])

	# Multiply the pattern of reference
	if rounded:  # rounded case
		bevel(	pattern_ref, 
				pattern_ref.frontiers(0,1,2), 
				('radius', min(0.3*(r_ext - r_int), r_int*(angle_step/2-theta_1))),
				)
		bevel(	pattern_ref, 
				pattern_ref.frontiers(2,3,0), 
				('radius', min(0.3*(r_ext - r_int), r_ext*(angle_step/2-theta_2))),
				)
		pattern_ref.mergeclose()
	pattern_profile = repeat(web(pattern_ref), patterns, rotatearound(angle_step, axis))

	# Profiles and surfaces
	webs_ref = (pattern_profile.flip(), ext_circle, int_circle)
	top_webs = [web.transform(vec3(0, 0, half_depth - int_height)) for web in webs_ref]
	bottom_webs = [web.transform(vec3(0, 0, -half_depth + int_height)).flip() for web in webs_ref]

	top_profile = reduce(add, top_webs)
	bottom_profile = reduce(add, bottom_webs)
	surfaces = extrusion(vec3(0, 0, depth - 2 * int_height), bottom_webs[0].flip())

	mesh = triangulation(top_profile) + triangulation(bottom_profile) + surfaces
	mesh.mergeclose()
	return mesh


def pattern_rect(
	ext_radius,
	int_radius,
	depth,
	int_height = 0,
	ratio = 1,
	patterns = 5,
	ext_thickness = None,
	int_thickness = None,
	**kwargs,
) -> Mesh:
	"""
	Generate two parts of the structure (the top and the bottom) with `patterns` distributed on the whole structure.
	All corners are straight. Check the function `pattern_rounded` to get rounded corners.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters:

		ext_radius: float (radius of the external border of the structure)
		int_radius: float (radius of the internal border of the structure)
		depth: float (face width)
		int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
							the total height will be `total_height = depth - 2 * int_height`)
		ratio: float (it is a number which is the proportional factor for an homothety of patterns)
		patterns: int (number of patterns inside the structure)
		ext_thickness (float): internal radius of the pattern
		int_thickness (float): external radius of the pattern

	Note:

		- for instance, if `ratio` is 1.5, the area of the pattern will be divided by 1.5.
		- if `r_int` and `r_ext` are chosen, `ratio` won't impact these parameters
	"""
	return create_pattern_rect(ext_radius, int_radius, depth, int_height, ratio, patterns, ext_thickness, int_thickness, False)


def pattern_rounded(
	ext_radius,
	int_radius,
	depth,
	int_height = 0,
	ratio = 1,
	patterns: int = 5,
	ext_thickness = None,
	int_thickness = None,
	**kwargs,
) -> Mesh:
	"""
	Generate two parts of the structure (the top and the bottom) with `patterns` distributed on the whole structure.
	All corners are rounded. Check the function `pattern_rect` to get straight corners.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters:

		ext_radius: float (radius of the external border of the structure)
		int_radius: float (radius of the internal border of the structure)
		depth: float (face width)
		int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
							the total height will be `total_height = depth - 2 * int_height`)
		ratio: float (it is a number which is the proportional factor for an homothety of patterns)
		patterns: int (number of patterns inside the structure)
		ext_thickness (float): internal radius of the pattern
		int_thickness (float): external radius of the pattern

	Note:

		- for instance, if `ratio` is 1.5, the area of the pattern will be divided by 1.5.
		- if `r_int` and `r_ext` are chosen, `ratio` won't impact these parameters
	"""
	return create_pattern_rect(ext_radius, int_radius, depth, int_height, ratio, patterns, ext_thickness, int_thickness, True)


def minmax_radius(points):
	min, max = inf, 0
	for p in points:
		r = length2(vec2(p))
		if r > max:		max = r
		elif r < min:	min = r
	return sqrt(min), sqrt(max)



def gearexterior(
	profile: Wire,
	depth,
	step = None,
	helix_angle = 0,
	chamfer = 0,
	**kwargs,
) -> Mesh:
	"""
	Generate the external part of the pinion

	Parameters:

		profile (Web):         profile of the pinion generated by `gearprofile`
		depth (float):         extrusion eight - width of the gear along its axis
		step (float):          step of chordal pitch, must be specified for non-null helix_angle, unused otherwise
		helix_angle (float):   helix angle for helical gears - only without bevel; `bevel` = False - it must be a radian angle
		chamfer (float):       chamfer angle - only for straight pinion
	"""
	# TODO: helical + chamfer

	half_depth = depth / 2
	footer, header = minmax_radius(profile.points)
	
	# Internal circles
	circle_ref = wire(Circle((O,Z), footer - 0.3*(header-footer)))
	top_circle = circle_ref.transform(vec3(0, 0, half_depth))
	bottom_circle = circle_ref.transform(vec3(0, 0, -half_depth))

	assert not (chamfer and helix_angle),  "chamfer is not (yet) supported with a non-null helix_angle"

	if chamfer:  # chamfer case
		# create a truncated profile (the top profile of the chamfer)
		start = mix(footer, header, 0.6)  # primitive radius
		k = tan(chamfer)
		truncated = deepcopy(profile) # profile which will be truncated
		for i, point in enumerate(profile.points):
			# truncate points that overlap the primitive
			r = length2(point)
			if r > start**2:  
				r = sqrt(r)
				profile.points[i] = point - vec3(0, 0, (r - start) * k)
				truncated.points[i] = point * start / r

		# Profiles
		top_profile = profile.transform(vec3(0, 0, half_depth))
		bottom_profile = top_profile.transform(scaledir(Z, -1))
		top_truncated = truncated.transform(vec3(0, 0, half_depth))
		bottom_truncated = truncated.transform(vec3(0, 0, -half_depth))

		# Surfaces
		bp = partial(junction, tangents="straight", resolution = ("div", 0))
		top_surface = triangulation(web([top_truncated, top_circle.flip()]))
		bottom_surface = triangulation(web([bottom_truncated.flip(), bottom_circle]))
		top_bevel = bp(top_truncated, top_profile.flip())
		bottom_bevel = bp(bottom_truncated.flip(), bottom_profile)
		gear_surface = bp(top_profile, bottom_profile.flip())

		surfaces = (top_surface, bottom_surface, top_bevel, bottom_bevel, gear_surface)
		mesh = reduce(add, surfaces)
		mesh.mergeclose()
	else:
		if helix_angle:  # helical teeth case
			# Edges of teeth
			R = header # the largest radius of profile
			# step to generate a helical transformation
			step = settings.curve_resolution(depth / cos(helix_angle), depth * tan(helix_angle) / R)
			angle = depth * tan(helix_angle) / R / (step + 1)
			h = depth / (step + 1)
			bottom_gear_edge = profile.transform(vec3(0, 0, -half_depth))
			transformations = (
				transform((vec3(0, 0, i * h), angleAxis(angle * i, Z)))
				for i in range(step + 2)
			)
			links = ((i, i + 1, 0) for i in range(step + 1))
			gear_surface = extrans(bottom_gear_edge, transformations, links)
			t = transform((vec3(0, 0, depth), angleAxis(depth * tan(helix_angle) / R, Z)))
			top_gear_edge = bottom_gear_edge.transform(t)
		else:  # straight teeth case
			# Edges of teeth
			bottom_gear_edge = profile.transform(vec3(0, 0, -half_depth))
			top_gear_edge = profile.transform(vec3(0, 0, half_depth))
			gear_surface = extrusion(vec3(0, 0, depth), bottom_gear_edge)

		# Surfaces
		top_surface = triangulation(web([top_gear_edge, top_circle.flip()]))
		bottom_surface = triangulation(web([bottom_gear_edge.flip(), bottom_circle]))
		mesh = gear_surface + top_surface + bottom_surface
		mesh.mergeclose()
	return mesh


def gearstructure(
	pattern,
	ext_radius,
	int_radius,
	depth,
	int_height = 0,
	ratio = 1,
	**kwargs,
) -> Mesh:
	"""
	Generate the internal part of the pinion

	Parameters:

		ext_radius (float):

				given by the attribut `_radius` of the result of `repeat_circular` -
				to avoid interference, it must be smaller than `_radius` (for instance 0.95 * `_radius`))

		int_radius (float):    it is the same radius of the largest radius of the hub part
		depth (float):         face width
		pattern:               any of 'full', 'circle', 'rect', 'rounded'
		int_height (float):

				if you want a pinion with a structure thinner than the value of `depth`,
				the total height will be `total_height = depth - 2 * int_height`
	"""
	# int_radius must not be 0
	int_radius = int_radius or 0.1 * ext_radius
	pattern_function = globals()['pattern_' + pattern]
	return pattern_function(ext_radius, int_radius, depth, int_height, ratio=ratio, **kwargs)


def gearhub(
	bore_radius,
	depth,
	int_height = 0,
	hub_height = None,
	hub_radius = None,
	**kwargs,
) -> Web:
	"""
	Generate a hub for a pinion part

	Parameters:

		bore_radius (float):     radius of the central bore
		depth (float):           face width; same parameter for `gearexterior` and `gearstructure`
		int_height (float):      only useful for no hub case, checkout the function `gearstructure` for more information
		hub_height (float):      height of the hub
		hub_radius (float):      external radius of the hub

	Note:

		- if `bore_radius` is null, the function will return a top circle and a bottom circle used for `geargather` function
		- if `hub_height` is null, the function will return a structure with a bore and without a hub
	"""
	if not (bore_radius):  # No hub case
		half_depth = depth / 2
		circle_ref = web(Circle((O,Z), depth * 0.1))
		circle_ref = triangulation(circle_ref)
		top_surface = circle_ref.transform(vec3(0, 0, half_depth - int_height))
		bottom_surface = circle_ref.transform(vec3(0, 0, -half_depth + int_height)).flip()
		return top_surface + bottom_surface

	if hub_height is None:
		hub_height = bore_radius
	if not (hub_radius):
		hub_radius = 2 * bore_radius
	height = hub_height + depth / 2 if hub_height else depth / 2 - int_height
	axis = (vec3(0, 0, height), vec3(0, 0, 1))
	bore_circle = Circle(axis, bore_radius)
	hub_circle = Circle(axis, hub_radius)

	# Top part
	top_surface = triangulation(web(bore_circle).flip() + web(hub_circle))

	# Lateral surfaces
	lateral_surfaces = extrusion(vec3(0, 0, -hub_height - depth), web(bore_circle))

	# Bottom part
	axis = (vec3(0, 0, -depth / 2), vec3(0, 0, 1))
	bottom_bore_circle = Circle(axis, bore_radius)
	bottom_hub_circle = Circle(axis, hub_radius)
	bottom_web = web(bottom_bore_circle) + web(bottom_hub_circle).flip()
	bottom_surface = triangulation(bottom_web)

	return top_surface + bottom_surface + lateral_surfaces


def geargather(exterior, structure, hub) -> Mesh:
	"""
	Gather all parts: exterior, structure, and hub

	You can obtain them via the provided functions, or generate them yourself.
	"""
	exterior.mergeclose()
	structure.mergeclose()
	hub.mergeclose()

	# Parameters
	box = structure.box()
	height_top = box.max.z
	height_bot = box.min.z
	ext_radius = box.max.x
	int_radius = 0

	box = hub.box()
	height_h_top = box.max.z
	height_h_bot = box.min.z
	ext_h_radius = box.max.x

	# Select borderlines
	# int = internal / ext = external
	# e = exterior / s = structure / h = hub
	# top = top / bot = bottom
	circle_int_e_top = select(exterior, vec3(0, 0, height_top))
	circle_int_e_bot = select(exterior, vec3(0, 0, height_bot))
	circle_ext_s_top = select(structure, vec3(ext_radius, 0, height_top))
	circle_ext_s_bot = select(structure, vec3(ext_radius, 0, height_bot))
	circle_int_s_top = select(structure, vec3(int_radius, 0, height_top))
	circle_int_s_bot = select(structure, vec3(int_radius, 0, height_bot))
	circle_ext_h_top = select(hub, vec3(ext_h_radius, 0, height_h_top))
	circle_ext_h_bot = select(hub, vec3(ext_h_radius, 0, height_h_bot))

	# Join all borderlines
	# j1 is the junction between `exterior` and `structure`
	# j2 is the junction between `structure` and `hub`
	j1_top = junction(circle_ext_s_top, circle_int_e_top, tangents="straight", resolution = ("div", 0))
	j1_bot = junction(circle_ext_s_bot, circle_int_e_bot, tangents="straight", resolution = ("div", 0))
	j2_top = junction(circle_ext_h_top, circle_int_s_top, tangents="straight", resolution = ("div", 0))
	j2_bot = junction(circle_ext_h_bot, circle_int_s_bot, tangents="straight", resolution = ("div", 0))
	j1 = j1_top + j1_bot
	j2 = j2_top + j2_bot

	mesh = exterior + j1 + structure + j2 + hub
	mesh.mergeclose()
	return mesh


def gear(
	step,
	z: int,
	depth,
	bore_radius = 0,
	int_height = 0,
	pattern = 'full',
	**kwargs,
) -> Mesh:
	"""
	Generate a pinion.

	Any extra argument will go to functions `gearprofile`, `gearstructure`, or `gearhub`

	Parameters:

		step (float):

			tooth step over the primitive curve, same as the matching rack step
			the primitive perimeter is `step * z` and radius is `step * z / 2*pi`

		z (int):         number of teeth
		depth (float):       extrusion eight - width of the gear along its axis
		bore_radius (float):   radius of the main bore
		int_height (float):    if you want a pinion with a structure thinner than the value of `depth`,
						       the total height will be `total_height = depth - 2 * int_height`
		pattern:
			determine the structure between exterior (tooth) and hub
			This argument specifies the use a a function named `'pattern_'+pattern` in this module.

	* Extra parameters for `gearprofile`

		offset (float):   offset of tooth (as a distance)
		alpha (float):    pressure angle in radian

	* Extra parameters for `gearexterior`

		helix_angle (float):   helix angle to get a helical pinion in radian
		chamfer (float):       chamfer angle - only for straight pinion

	* Extra parameters for `gearstructure`

		ratio (float):       influence the proportion of dimensions of the structure

		Note: `int_height` impacts the thickness of the structure unless specified

	* Extra parameters for `gearhub`

		hub_height (float):   height of the hub shoulder

	Note:

		- `int_height` impacts the height of the hub unless specified.
		- if `hub_height` is null, there will be no hub.
	"""
	profile = repeat_circular(gearprofile(step, z, **kwargs), z)

	# Parts
	exterior = gearexterior(profile, depth, step, **kwargs)
	hub = gearhub(bore_radius, depth, int_height, **kwargs)
	ext_int = minmax_radius(exterior.points)[0]
	structure = gearstructure(
					pattern,
					ext_int * 0.95,
					max(minmax_radius(hub.points)[1], 0.2*ext_int),
					depth,
					int_height,
					**kwargs)
	return geargather(exterior, structure, hub)


def frange(start:float, end:float, div:int=10):
	"""Generate `div` numbers from `start` to `end`"""
	k = (end - start) / (div - 1)
	return (start + i * k for i in range(div))


def get_pitch_cone_angle(z_pinion:int, z_wheel:int, shaft_angle:float=0.5 * pi) -> float:
	"""
	Return the pitch cone angle of the pinion called `gamma_p`.
	The pitch cone angle of the wheel is equal to `shaft_angle - gamma_p`

	Parameters:

		z_pinion (int): 		the number of teeth on the bevel pinion
		z_wheel (int): 			the number of teeth on the bevel wheel
		shaft_angle (float): 	the shaft angle
	"""
	return atan2(sin(shaft_angle), ((z_wheel / z_pinion) + cos(shaft_angle)))


def spherical_involute(cone_angle:float, t0:float, t:float) -> vec3:
	"""
	Return spherical involute function

	Parameters:

		t (float): 				the angular position
		t0 (float): 			the difference phase
		cone_angle (float): 	the cone angle

	Return:	
	
		a normalized `vec3`
	"""
	cos_g, sin_g = cos(cone_angle), sin(cone_angle)
	return vec3(
		sin_g * cos(t * sin_g) * cos(t + t0) + sin(t * sin_g) * sin(t + t0),
		sin_g * cos(t * sin_g) * sin(t + t0) - sin(t * sin_g) * cos(t + t0),
		cos_g * cos(t * sin_g),
	)


def spherical_involuteof(pitch_cone_angle:float, t0:float, alpha:float, t:float) -> vec3:
	"""
	Return the spherical interference function

	Parameters:

		t (float): 					the angular position
		t0 (float): 				the difference phase
		pitch_cone_angle (float):	the pitch cone angle
		alpha(float): 				the height angle offset of the rack

	Return:	
	
		a normalized `vec3`
	"""
	cos_p, sin_p = cos(pitch_cone_angle), sin(pitch_cone_angle)
	involute = lambda t, t0: spherical_involute(pitch_cone_angle, t0, t)
	vec = lambda t: vec3(-cos_p * cos(t), -cos_p * sin(t), sin_p)
	return cos(alpha) * involute(t, t0) + sin(alpha) * vec(t + t0)


def derived_spherical_involute(cone_angle:float, t0:float):
	"""
	Return the function of the derived spherical involute function.

	Parameters:

		cone_angle (float): 	the cone angle
		t0 (float): 			the phase difference
	"""
	cos_g, sin_g = cos(cone_angle), sin(cone_angle)
	return lambda t: vec3(
		cos_g ** 2 * sin(t * sin_g) * cos(t + t0),
		cos_g ** 2 * sin(t * sin_g) * sin(t + t0),
		-cos_g * sin_g * sin(t * sin_g),
	)


def jacobian_spherical_involute(base_cona_angle:float, pitch_cone_angle:float, t01:float, t02:float, alpha:float) -> callable:
	"""
	Return the function of the jacobian used for the newton method in `spherical_gearprofile`

	Parameters:

		base_cona_angle (float): 	the base cone angle
		pitch_cone_angle (float): 	the pitch cone angle
		t01 (float): 				the phase of the spherical involute function
		t02 (float): 				the phase of the spherical interference function
		alpha (float): 				the height angle offset of the rack
	"""
	dsi = derived_spherical_involute # for convenience
	derived_involute = dsi(base_cona_angle, t01)
	cos_p = cos(pitch_cone_angle)
	vec = lambda t: vec3(cos_p * sin(t), -cos_p * cos(t), 0)
	derived_interference = lambda t: dsi(pitch_cone_angle, t02)(t) * cos(alpha) + sin(alpha) * vec(t + t02)
	return lambda t1, t2: mat3(derived_involute(t1), -derived_interference(t2), vec3(0, 0, 1))


def spherical_rack_tools(z:float, pressure_angle:float=pi / 9, ka:float=1, kd:float=1.25):
	"""
	Return a list of all information useful to generate a spherical rack.
	Five elements :
	
		1) the minimum abscissa for the function (fifth element)
		2) the maximum abscissa for the function (fifth element)
		3) the phase of a tooth
		4) the phase of space
		5) the function to generate a tooth

	Parameters :

		z (float):
			
			number of tooth of the rack equal to `z_pinion / sin(pitch_cone_angle)`
			or `z_wheel / sin(shaft_angle - pitch_cone_angle)`
		
		pressure_angle (float): the pressure angle of the gear
		ka (float): 			the addendum coefficient
		kd (float): 			the dedendum coefficient
	"""
	k = 1 / z
	gamma_p = 0.5 * pi
	gamma_b = asin(cos(pressure_angle))
	cos_b, sin_b = cos(gamma_b), sin(gamma_b)
	gamma_f = gamma_p + 2 * ka * k
	gamma_r = gamma_p - 2 * kd * k

	phi_p = acos(tan(gamma_b) / tan(gamma_p))
	theta_p = atan2(sin_b * tan(phi_p), 1) / sin_b - phi_p
	phase_diff = k * pi + 2 * theta_p

	t_min = acos(cos(gamma_r) / cos_b) / sin_b
	t_max = acos(cos(gamma_f) / cos_b) / sin_b

	involute = lambda t, t0: spherical_involute(gamma_b, t0, t)
	v = vec3(1, 1, 0)
	phase_empty = 2 * pi * k - anglebt(involute(t_min, 0) * v, involute(-t_min, phase_diff) * v)

	return [t_min, t_max, phase_diff, phase_empty, involute]


def spherical_rackprofile(z:float, pressure_angle:float=pi / 9, ka:float=1, kd:float=1.25):
	"""
	Return a `Wire` which is a tooth of the rack.

	Parameters:

		z (float):
		
			number of tooth of the rack equal to `z_pinion / sin(pitch_cone_angle)`
			or `z_wheel / sin(shaft_angle - pitch_cone_angle)`
		
		pressure_angle (float): 	the pressure angle of the gear
		ka (float): 				the addendum coefficient
		kd (float): 				the dedendum coefficient
	"""
	t_min, t_max, phase1, phase2, involute = spherical_rack_tools(z, pressure_angle, ka, kd)
	side1 = [involute(t, 0) for t in frange(t_min, t_max)]
	side2 = [involute(-t, phase1) for t in frange(t_min, t_max)]
	segment = Segment(involute(t_min, -phase2), side1[0]).mesh()
	segment2 = Segment(side1[-1], side2[-1]).mesh()
	wire = segment + Wire(side1) + segment2 + Wire(side2).flip()
	return wire


def spherical_gearprofile(z:int, pitch_cone_angle:float, pressure_angle:float=pi / 9, ka:float=1, kd:float=1.25) -> Wire:
	"""
	Generate and return a `Wire` of a 1-period tooth spherical profile for a bevel gear

	Parameters:

		z (int):						number of tooth on the gear this profile is meant for
		pitch_cone_angle (float): 		the pitch cone angle
		pressure_angle (float):			pressure angle of the tooth
		ka (float):						addendum coefficient
		kd (float): 					dedendum coefficient
	"""
	# Initialization of parameters
	gamma_p = pitch_cone_angle # for convenience
	gamma_b = asin(cos(pressure_angle) * sin(gamma_p))
	cos_b, sin_b = cos(gamma_b), sin(gamma_b)
	tooth_size = pi / z
	involute = lambda t, t0 : spherical_involute(gamma_b, t0, t)
	epsilon_p = acos(cos(gamma_p) / cos_b) / sin_b
	theta_p = anglebt(involute(0, 0) * vec3(1, 1, 0), involute(epsilon_p, 0) * vec3(1, 1, 0))
	phase_diff = tooth_size + 2 * theta_p
	phase_empty = phase_interference = 2 * pi / z - phase_diff
	# The following number `k` is useful to simplify some calculations
	# It's broadly speaking `1/z_rack` and `z_rack` is not an integer !
	k = sin(gamma_p) / z

	# Spherical involute part
	gamma_f = gamma_p + 2 * ka * k # addendum cone angle
	gamma_r = gamma_p - 2 * kd * k # dedendum cone angle
	t_min = 0
	t_max = acos(cos(gamma_f) / cos_b) / sin_b
	if gamma_r > gamma_b:
		v = vec3(1, 1, 0)
		t_min = acos(cos(gamma_r) / cos_b) / sin_b
		phase_empty = 2 * pi / z - anglebt(
			involute(t_min, 0) * v, involute(-t_min, phase_diff) * v
		)

	# Calculation of offsets due to geometry of spherical rack
	_, t_rack_max, phase1, _, rinvolute = spherical_rack_tools(1 / k, pressure_angle, ka, kd)
	interference = lambda t, t0 : spherical_involuteof(gamma_p, t0, alpha, t)
	alpha = 2 * ka * k
	n1, n2 = rinvolute(t_rack_max, 0) * vec3(1, 1, 0), rinvolute(-t_rack_max, phase1) * vec3(1, 1, 0)
	beta = 0.5 * anglebt(n1, n2) * length(n1) / sin_b

	# Newton method to calculate the intersection between
	# the spherical involute and the spherical interference.
	# Objective function
	involuteat = lambda t2, t0 : spherical_involuteof(gamma_p, t0, alpha, t2)
	f = lambda t1, t2: involute(t1, 0) - involuteat(t2, -0.5 * phase_interference + beta)
	# Jacobian matrix
	J = jacobian_spherical_involute(gamma_b, gamma_p, 0, -0.5 * phase_interference + beta, alpha)

	# Compute the intersection values
	t1, t2, t3 = 0.5 * t_max, -0.5 * t_max, 0
	for i in range(8):
		t1, t2, t3 = vec3(t1, t2, t3) - inverse(J(t1, t2)) * f(t1, t2)

	# Build sides of a tooth
	interference1 = [interference(t, -0.5 * phase_interference + beta) for t in frange(0, t2)]
	interference2 = [interference(-t, phase_diff + 0.5 * phase_interference - beta) for t in frange(0, t2)]
	side1 = interference1[:-1] + [involute(t, 0) for t in frange(t1, t_max)]
	side2 = interference2[:-1] + [involute(-t, phase_diff) for t in frange(t1, t_max)]

	# Extreme points of sides to compute angle between them
	a = interference(0, -0.5 * phase_interference + beta)
	b = interference(0, phase_diff + 0.5 * phase_interference - beta)
	final_phase_empty = 2 * pi / z - anglebt(a * vec3(1, 1, 0), b * vec3(1, 1, 0))
	top = Segment(involute(t_max, 0), involute(-t_max, phase_diff)).mesh()
	bottom = Segment(angleAxis(-final_phase_empty, vec3(0, 0, 1)) * a, a).mesh()

	return bottom + Wire(side1) + top + Wire(side2).flip()


def cone_projection(profile: Wire, pitch_cone_angle:float) -> Wire:
	"""
	Return a Wire of the spherical profile projected on a cone

	Parameters:
		profile (Wire) : 			the spherical profile
		pitch_cone_angle (float) : 	the pitch cone angle
	"""
	ref = lambda t: vec3(sin(pitch_cone_angle) * cos(t), sin(pitch_cone_angle) * sin(t), cos(pitch_cone_angle))
	new_points = [1 / dot(ref(atan2(point.y, point.x)), point) * point for point in profile.points]
	return Wire(new_points, indices=profile.indices)


def bevelgear(step:float, z:int, pitch_cone_angle:float, pressure_angle:float=pi/9, ka:float=1, kd:float=1.25, bore_radius:float=None, bore_height:float=None):
	"""
	Generate a bevel gear.

	Parameters:

		step (float):

			tooth step over the primitive curve, same as the matching rack step
			the primitive perimeter is `step * z` and radius is `step * z / (2 * pi)`

		z (int):		 			number of teeth
		pitch_cone_angle (float):	   		pitch cone angle
		pressure_angle (flaot):		the pressure angle of the tooth
		bore_radius (float):   		radius of the main bore
		bore_height (float):   		height of the main bore
	"""

	# Initialization of parameters
	gamma_p = pitch_cone_angle # for convenience
	gamma_b = asin(cos(pressure_angle) * sin(gamma_p))
	sin_b = cos(pressure_angle) * sin(gamma_p)
	rp = z*step / (2*pi)
	rho1 = rp / sin(gamma_p)
	rho0 = 2 * rho1 / 3
	k = sin(gamma_p) / z
	gamma_r = gamma_p - 2 * kd * k
	phi_p = acos(tan(gamma_b) / tan(gamma_p))
	theta_p = atan2(sin_b * tan(phi_p), 1) / sin_b - phi_p
	phase_diff = pi / z + 2 * theta_p

	# Generate spherical profiles
	spherical_profile = spherical_gearprofile(z, gamma_p, pressure_angle, ka, kd) # one tooth
	outside_profile = spherical_profile.transform(rho1 * 1.1)
	inside_profile = spherical_profile.transform(rho0 * 0.9)

	# Generate teeth border
	teeth_border = blendpair(outside_profile, inside_profile.flip(), tangents="straight")

	# Common values
	v = vec3(1, 1, 0)
	angle1tooth = anglebt(spherical_profile[0] * v, spherical_profile[-1] * v)
	gamma_l = gamma_p + 2 * (ka + 0.25) * k # offset : 0.25 for boolean operation
	A = vec3(rho1 * sin(gamma_r), 0, rho1 * cos(gamma_r))
	B = vec3(rho1 * sin(gamma_l), 0, rho1 * cos(gamma_l))
	C = vec3(rho0 * sin(gamma_r), 0, rho0 * cos(gamma_r))
	D = vec3(rho0 * sin(gamma_l), 0, rho0 * cos(gamma_l))
	v1 = A * v
	v2 = spherical_profile[0] * v
	phase_tooth_body = anglebt(v1, v2)
	phase_tooth_body = -phase_tooth_body if cross(v1, v2).z > 0 else phase_tooth_body


	# Generate points for a section
	if bore_radius is None:
		bore_radius = 0.5 * rho0 * sin(gamma_r)
	if bore_height is None:
		bore_height = 0.4 * rho1 * cos(gamma_r)
	if bore_radius:
		if bore_height:
			E = vec3(1.5 * bore_radius, 0, rho1 * cos(gamma_r))
			F = vec3(bore_radius, 0, rho0 * cos(gamma_r))
			G = vec3(1.5 * bore_radius, 0, rho1 * cos(gamma_r) + bore_height) # top of the bore
			H = vec3(bore_radius, 0, rho1 * cos(gamma_r) + bore_height) # top of the bore
			wire = Wire([A, B, D, C, F, H, G, E, A]).segmented()
			chamfer(wire, [4, 5, 6], ("distance", bore_radius * 0.05))
			bevel(wire, [7], ("distance", bore_height * 0.1))
		else:
			E = vec3(bore_radius, 0, rho1 * cos(gamma_r))
			F = vec3(bore_radius, 0, rho0 * cos(gamma_r))
			wire = Wire([A, B, D, C, F, E, A]).segmented()
			chamfer(wire, [4, 5], ("distance", bore_radius * 0.05))
	else:
		E = vec3(0, 0, rho1 * cos(gamma_r))
		F = vec3(0, 0, rho0 * cos(gamma_r))
		wire = Wire([A, B, D, C, F, E, A]).segmented()

	axis = (O,Z)
	body = revolution(angle1tooth, axis, wire)
	one_tooth = intersection(body, teeth_border.transform(angleAxis(phase_tooth_body, axis[1])))
	all_teeth = repeat(one_tooth, z, rotatearound(angle1tooth, axis))
	all_teeth.finish()
	return all_teeth
