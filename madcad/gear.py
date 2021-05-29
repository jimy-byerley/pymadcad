# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .mesh import Web, Wire, Mesh, web
from . import settings
from .blending import junction, blendpair
from .generation import extrusion, revolution, repeat, extrans
from .primitives import Circle, ArcCentered, Segment
from .triangulation import triangulation
from .selection import *
from .rendering import show
from .settings import curve_resolution
from .cut import bevel
from math import cos, sin, asin, tan
from functools import reduce, partial
from operator import add
from copy import copy


def rackprofile(step, h=None, e=-0.05, x=0.5, alpha=radians(30), resolution=None) -> Wire:
	if h is None:
		h = default_h(step, alpha)
	e = h*e

	return Wire([
		vec3(step*x/2 - tan(alpha) * ( h-e),   h-e,  0),
		vec3(step*x/2 - tan(alpha) * (-h-e),  -h-e,  0),
		vec3(step*(2-x)/2 + tan(alpha) * (-h-e),  -h-e,  0),
		vec3(step*(2-x)/2 + tan(alpha) * ( h-e),   h-e,  0),
		vec3(step*(2+x)/2 - tan(alpha) * ( h-e),   h-e,  0),
		], groups=['rack'])


def gearprofile(step, z, h=None, e=-0.05, x=0.5, alpha=radians(30), resolution=None, **kwargs) -> Wire:
	if h is None:
		h = default_h(step, alpha)
	e = h*e
	p = step*z / (2*pi)	# primitive circle
	c = p * cos(alpha)	# tangent circle to gliding axis

	o0 = angle(involute(c, 0, tan(alpha)))	# offset of contact curve
	oi = atan((h+e)/p * tan(alpha))		# offset of interference curve

	l0 = involuteat(c, p+h-e)	# interval size of contact curve

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
	interference = c > p-h-e
	if interference:
		# Newton solver method
		# to compute the parameters (t1, t2) of the intersection between contact line and interference line
		t0, ti = o0, oi
		# initial state
		t1 = t0 - tan(alpha)	# put contact line on primitive
		t2 = ti + sqrt(c**2 - (p-h-e)**2) /p	# put interference point on base circle
		for i in range(8):
			ct1, ct2 = cos(t1), cos(t2)
			st1, st2 = sin(t1), sin(t2)
			# function value
			f = (	c*vec2(ct1,st1) + c*(t0-t1)*vec2(-st1,ct1)
				+	(h+e-p)*vec2(ct2,st2) -p*(ti-t2)*vec2(-st2,ct2)
				)
			# jacobian matrix (f partial derivatives)
			J = mat2(
				-c*(t0-t1)*vec2(ct1,st1),
				p*(ti-t2)*vec2(ct2,st2) + (h+e)*vec2(-st2,ct2),
				)
			# iteration
			t1, t2 = vec2(t1,t2) - inverse(J)*f
		li = t2 - ti	# interval size of interference curve
		s0 = t0 - t1	# generation start of contact curve
	else:
		s0 = involuteat(c, p-h-e)

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
			v = involuteof(p, ti, -h-e, t)
			pts.append(vec3(v,0))

	# parameters for second side
	place = step/(2*p)*(2-x)
	t0 = place - o0
	ti = place - oi
	# interference line
	if interference:
		for i in range(n+1):
			t = interpol1(ti, ti-li, i/n)
			v = involuteof(p, ti, -h-e, t)
			pts.append(vec3(v,0))
	# contact line
	for i in range(n+1):
		t = interpol1(t0+s0, t0+l0, i/n)
		v = involute(c, t0, t)
		pts.append(vec3(v,0))

	pts.append(angleAxis(step/p, vec3(0,0,1)) * pts[0])

	return Wire(pts, groups=['gear'])

def gearcircles(step, z, h=None, e=-0.05, x=0.5, alpha=radians(30)):
	''' return the convenient circles radius for a gear with the given parameters
		return is `(primitive, base, bottom, top)`
	'''
	if h is None:
		h = default_h(step, alpha)
	e = h*e
	p = step*z / (2*pi)	# primitive circle
	c = p * cos(alpha)	# tangent circle to gliding axis
	return p, c, p-h-e, p+h-e


def default_h(step, alpha):
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


def z_multiple(gear_profile: Wire, z: int) -> Web:
	"""
	Repeat z times `gear_profile` by revolution

	Parameters
	----------
	:gear_profile: Wire (profile of the pinion generated by the function `gearprofile`)
	:z: int (number of teeth)
	"""
	# Parameters
	x, y = gear_profile[0], gear_profile[-1]
	angle = anglebt(x, y)
	axis = (vec3(0, 0, 0), vec3(0, 0, 1))

	profile = repeat(gear_profile, z, rotatearound(angle, axis))
	profile.mergeclose()
	return profile


def full_pattern(
	ext_radius: float, int_radius: float, depth: float, int_height: float = 0, **kwargs
) -> Mesh:
	"""
	Generate two full parts of the structure (the top and the bottom).

	Return a tuple (Web, Web, None) where the first `Web` is the top of the structure and
	the second `Web` is the bottom of the structure.

	Parameters
	----------
	:ext_radius: float (radius of the external border of the pattern)
	:int_radius: float (radius of the internal border of the pattern)
	:depth: float (face width)
	:int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	"""
	half_depth = depth / 2
	assert half_depth > int_height, "`int_height` must be smaller than `depth / 2`"
	axis = (vec3(0, 0, 0), vec3(0, 0, 1))
	circles_ref = web(Circle(axis, ext_radius)) + web(Circle(axis, int_radius)).flip()
	top_profile = circles_ref.transform(vec3(0, 0, half_depth - int_height))
	bottom_profile = circles_ref.transform(vec3(0, 0, -half_depth + int_height)).flip()
	mesh = triangulation(top_profile) + triangulation(bottom_profile)
	mesh.mergeclose()
	return mesh


def circle_pattern(
	ext_radius: float,
	int_radius: float,
	depth: float,
	int_height: float = 0,
	ratio: float = 1,
	n_circles: int = 5,
	r_int: float = None,
	r_ext: float = None,
	**kwargs,
) -> Mesh:
	"""
	Generate two parts of the structure (the top and the bottom) with `n_circles` distributed on the whole structure.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters
	----------
	:ext_radius: float (radius of the external border of the structure)
	:int_radius: float (radius of the internal border of the structure)
	:depth: float (face width)
	:int_height: flaot (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	:ratio: float (it is a number that allows to change proportionally the radius of circles)
	:n_circles: int (number of circles of the structure)
	:r_int: float (radius of circles)
	:r_ext: float (radius where the origins of circles are placed)

	Notes:
	- for instance, with a ratio of 1.5, the radius of circles `r_int` will be divided by 1.5
	- if `r_int` is chosen, `ratio` won't impact the radius of circles `r_int`
	"""
	# Parameters
	half_depth = depth / 2
	if not (r_int) and not (r_ext):
		assert 0.5 < ratio, "`ratio` must be in the interval ]0.5; +inf["
	r_ext = (ext_radius + int_radius) / 2 if not (r_ext) else r_ext # radius where the origins of circles are placed
	r_int = (ext_radius - int_radius) / (4 * ratio) if not (r_int) else r_int # radius of circles
	error_msg1 = "There are some circles who put themselves on top of each other. `n_circles` is too large or the value of `ratio` is too small if the value of `r` was defined by default."
	error_msg2 = "`r_int` must be smaller than `r_ext` if they were chosen. Else you should reduce the value of `ratio`"
	assert r_int / r_ext <= sin(pi / n_circles), error_msg1
	assert (r_int < r_ext), error_msg2
	Z = vec3(0, 0, 1)
	axis = (vec3(0, 0, 0), Z)

	# Borders
	ext_circle = web(Circle(axis, ext_radius))
	int_circle = web(Circle(axis, int_radius)).flip()

	# Pattern
	circle_ref = web(Circle((vec3(r_ext, 0, 0), Z), r_int))
	angle = 2 * pi / n_circles
	pattern_profile = repeat(circle_ref.flip(), n_circles, rotatearound(angle, axis))

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


def create_rect_pattern(
	ext_radius: float,
	int_radius: float,
	depth: float,
	int_height: float = 0,
	ratio: float = 1,
	n_patterns: int = 5,
	r_int: float = None,
	r_ext: float = None,
	rounded: bool = False,
	**kwargs,
) -> Mesh:
	"""
	Function used for `rect_pattern` when `pattern` = "rect"
	and `rounded_pattern` when `pattern` = "rounded"

	Parameters
	----------
	:ext_radius: float (radius of the external border of the structure)
	:int_radius: float (radius of the internal border of the structure)
	:depth: float (face width)
	:int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	:ratio: float (it is a number which is the proportional factor for an homothety of patterns)
	:n_patterns: int (number of patterns inside the structure)
	:r_int: float (internal radius of the pattern)
	:r_ext: float (external radius of the pattern)
	:rounded: bool (if it is True, the pattern will be rounded)

	Notes:
	- for instance, if `ratio` is 1.5, the area of the pattern will be divided by 1.5.
	- if `r_int` and `r_ext` are chosen, `ratio` won't impact these parameters
	"""
	# Parameters
	half_depth = depth / 2
	offset = ratio * 0.15 * (ext_radius - int_radius)
	r_ext = ext_radius - offset if not (r_ext) else r_ext # external radius of pattern
	r_int = int_radius + offset if not (r_int) else r_int # internal radius of pattern
	thickness = ratio * r_ext / 10  # thickness of "arms"
	# angle between the bottom edge of an arm and the middle of it
	theta_1 = asin(thickness / (2 * r_int))
	# angle between the top edge of an arm and the middle of it
	theta_2 = asin(thickness / (2 * r_ext))
	angle_step = 2 * pi / n_patterns
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
	arc_A1_B1 = ArcCentered((O, -Z), B1, A1)
	arc_A2_B2 = ArcCentered((O, Z), A2, B2)
	segmentA1_A2 = Segment(A1, A2)
	segmentB1_B2 = Segment(B2, B1)
	pattern_ref = web([segmentA1_A2, arc_A1_B1, segmentB1_B2, arc_A2_B2])

	# Multiply the pattern of reference
	if rounded:  # rounded case
		groups = pattern_ref.groupextremities()
		bevel(pattern_ref, groups, ("radius", 0.1 * (r_ext - r_int)))
		pattern_ref.mergeclose()
	pattern_profile = repeat(pattern_ref, n_patterns, rotatearound(angle_step, axis))

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


def rect_pattern(
	ext_radius: float,
	int_radius: float,
	depth: float,
	int_height: float = 0,
	ratio: float = 1,
	n_patterns: int = 5,
	r_int: float = None,
	r_ext: float = None,
	**kwargs,
) -> Mesh:
	"""
	Generate two parts of the structure (the top and the bottom) with `n_patterns` distributed on the whole structure.
	All corners are straight. Check the function `rounded_pattern` to get rounded corners.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters
	----------
	:ext_radius: float (radius of the external border of the structure)
	:int_radius: float (radius of the internal border of the structure)
	:depth: float (face width)
	:int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	:ratio: float (it is a number which is the proportional factor for an homothety of patterns)
	:n_patterns: int (number of patterns inside the structure)
	:r_int: float (internal radius of the pattern)
	:r_ext: float (external radius of the pattern)

	Notes:
	- for instance, if `ratio` is 1.5, the area of the pattern will be divided by 1.5.
	- if `r_int` and `r_ext` are chosen, `ratio` won't impact these parameters
	"""
	return create_rect_pattern(ext_radius, int_radius, depth, int_height, ratio, n_patterns, r_int, r_ext, False)


def rounded_pattern(
	ext_radius: float,
	int_radius: float,
	depth: float,
	int_height: float = 0,
	ratio: float = 1,
	n_patterns: int = 5,
	r_int: float = None,
	r_ext: float = None,
	**kwargs,
) -> Mesh:
	"""
	Generate two parts of the structure (the top and the bottom) with `n_patterns` distributed on the whole structure.
	All corners are rounded. Check the function `rect_pattern` to get straight corners.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters
	----------
	:ext_radius: float (radius of the external border of the structure)
	:int_radius: float (radius of the internal border of the structure)
	:depth: float (face width)
	:int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	:ratio: float (it is a number which is the proportional factor for an homothety of patterns)
	:n_patterns: int (number of patterns inside the structure)
	:r_int: float (internal radius of the pattern)
	:r_ext: float (external radius of the pattern)

	Notes:
	- for instance, if `ratio` is 1.5, the area of the pattern will be divided by 1.5.
	- if `r_int` and `r_ext` are chosen, `ratio` won't impact these parameters
	"""
	return create_rect_pattern(ext_radius, int_radius, depth, int_height, ratio, n_patterns, r_int, r_ext, True)


def gearexterior(
	profile: Web,
	min_radius: float,
	step: float,
	z: int,
	depth: float,
	helix_angle: float = 0,
	chamfer: float = 0,
	**kwargs,
) -> Mesh:
	"""
	Generate the external part of the pinion

	Parameters
	----------
	:profile: Web (profile of the pinion generated by `gearprofile`)
	:min_radius: float (internal radius of the external part)
	:step: float (step of chordal pitch)
	:z: int (number of teeth)
	:depth: float (face width)
	:helix_angle: float (helix angle for helical gears - only without bevel; `bevel` = False - it must be a radian angle)
	:chamfer: float (chamfer angle - only for straight pinion)
	"""
	# TODO: helical + chamfer

	half_depth = depth / 2
	# Internal circles
	axis = (vec3(0, 0, 0), vec3(0, 0, 1))
	circle_ref = Circle(axis, min_radius, resolution=("div", z)).mesh()
	top_circle = circle_ref.transform(vec3(0, 0, half_depth))
	bottom_circle = circle_ref.transform(vec3(0, 0, -half_depth))

	if chamfer:  # chamfer case
		pr = step * z / (2 * pi)  # primitive radius
		square_pr = pr ** 2
		Z = vec3(0, 0, 1)
		k = tan(chamfer)
		truncated = deepcopy(profile) # profile which will be truncated
		for i, point in enumerate(profile.points):
			r = length2(point)
			if r > square_pr:
				r = sqrt(r)
				profile.points[i] = point - vec3(0, 0, (r - pr) * k)
				truncated.points[i] = point * pr / r

		# Profiles
		top_profile = profile.transform(vec3(0, 0, half_depth))
		bottom_profile = top_profile.transform(scaledir(vec3(0, 0, 1), -1))
		top_truncated = truncated.transform(vec3(0, 0, half_depth))
		bottom_truncated = truncated.transform(vec3(0, 0, -half_depth))

		# Surfaces
		bp = partial(junction, tangents="straight")
		top_surface = bp(top_truncated.flip(), top_circle)
		bottom_surface = bp(bottom_truncated, bottom_circle.flip())
		top_bevel = bp(top_truncated, top_profile.flip())
		bottom_bevel = bp(bottom_truncated.flip(), bottom_profile)
		gear_surface = bp(top_profile, bottom_profile.flip())

		surfaces = (top_surface, bottom_surface, top_bevel, bottom_bevel, gear_surface)
		mesh = reduce(add, surfaces)
		mesh.mergeclose()
	else:
		if helix_angle:  # helical teeth case
			# Edges of teeth
			square_radius = max(length2(v) for v in profile.points)
			R = sqrt(square_radius) # the largest radius of profile
			# step to generate a helical transformation
			step = curve_resolution(depth / cos(helix_angle), depth * tan(helix_angle) / R)
			Z = vec3(0, 0, 1)
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
		top_surface = junction(top_gear_edge.flip(), top_circle, tangents="straight")
		bottom_surface = junction(bottom_gear_edge, bottom_circle.flip(), tangents="straight")
		mesh = gear_surface + top_surface + bottom_surface
		mesh.mergeclose()
	return mesh


def gearstructure(
	pattern: str,
	ext_radius: float,
	int_radius: float,
	depth: float,
	int_height: float = 0,
	ratio: float = 1,
	**kwargs,
) -> Mesh:
	"""
	Generate the internal part of the pinion
	:ext_radius: float (it is given by the attribut `_radius` of the result of `z_multiple` -
	| to avoid interference, it must be smaller than `_radius` (for instance 0.95 * `_radius`))
	:int_radius: float (it is the same radius of the largest radius of the hub part)
	:depth: float (face width)
	:pattern: str (values: 'full', 'circle', 'rect', 'rounded')
	:int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	"""
	# int_radius must not be 0
	int_radius = int_radius if int_radius else 0.1 * ext_radius
	pattern_function = globals()[pattern + "_pattern"]
	return pattern_function(ext_radius, int_radius, depth, int_height, ratio=ratio, **kwargs)


def gearhub(
	bore_radius: float,
	depth: float,
	int_height: float = 0,
	hub_height: float = None,
	hub_radius: float = None,
	**kwargs,
) -> Web:
	"""
	Generate a hub for a pinion part

	Parameters
	----------
	:bore_radius: float (radius of the central bore)
	:depth: float (face width; same parameter for `gearexterior` and `gearstructure`)
	:int_height: float (only useful for no hub case, checkout the function `gearstructure` for more information)
	:hub_height: float (height of the hub)
	:hub_radius: float (external radius of the hub)

	Notes:
	- if `bore_radius` is null, the function will return a top circle and a bottom circle used for `geargather` function
	- if `hub_height` is null, the function will return a structure with a bore and without a hub
	"""
	if not (bore_radius):  # No hub case
		half_depth = depth / 2
		circle_ref = web(Circle((vec3(0, 0, 0), vec3(0, 0, 1)), depth * 0.1))
		circle_ref = triangulation(circle_ref)
		top_surface = circle_ref.transform(vec3(0, 0, half_depth - int_height))
		bottom_surface = circle_ref.transform(vec3(0, 0, -half_depth + int_height)).flip()
		return top_surface + bottom_surface

	if hub_height is None:
		hub_height = bore_radius
	if not (hub_radius):
		hub_radius = 2.5 * bore_radius
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


def geargather(exterior: Mesh, structure: Mesh, hub: Mesh) -> Mesh:
	"""
	Gather all parts (`exterior`, `structure` and `hub` parts)
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
	circle_ext_h_top = select(hub, vec3(ext_radius, 0, height_top))
	circle_ext_h_bot = select(hub, vec3(ext_radius, 0, height_bot))

	# Join all borderlines
	# j1 is the junction between `exterior` and `structure`
	# j2 is the junction between `structure` and `hub`
	j1_top = junction(circle_ext_s_top, circle_int_e_top, tangents="straight")
	j1_bot = junction(circle_ext_s_bot, circle_int_e_bot, tangents="straight")
	j2_top = junction(circle_ext_h_top, circle_int_s_top, tangents="straight")
	j2_bot = junction(circle_ext_h_bot, circle_int_s_bot, tangents="straight")
	j1 = j1_top + j1_bot
	j2 = j2_top + j2_bot

	mesh = exterior + j1 + structure + j2 + hub
	mesh.mergeclose()
	return mesh


def gear(
	step: float,
	z: int,
	b: float,
	bore_radius: float = 0,
	int_height: float = 0,
	pattern: str = "full",
	**kwargs,
) -> Mesh:
	"""
	Generate a pinion

	Parameters
	----------
	:step: float (step of chordal pitch)
	:z: int (number of teeth)
	:b: float (face width)
	:bore_radius: float (radius of the main bore)
	:int_height: float (if you want a pinion with a structure thinner than the value of `depth`,
						the total height will be `total_height = depth - 2 * int_height`)
	:pattern: str (determine the structure of pinion)

	| Extra parameters for `gearprofile`
	| ----------------------------------
	| :x: float (offset of tooth)
	| :alpha: float (pressure angle in radian)
	|
	| Extra parameters for `gearexterior`
	| -----------------------------------
	| :helix_angle: float (helix angle to get a helical pinion in radian)
	| :chamfer: float (chamfer angle - only for straight pinion)
	|
	| Extra parameters for `gearstructure`
	| ------------------------------------
	| :ratio: float (influence the proportion of dimensions of the structure)
	| Note: `int_height` impacts the thickness of the structure
	|
	| Extra parameters for `gearhub`
	| ------------------------------
	| :hub_height: float (height of the hub shoulder)
	| Notes : - `int_height` impacts the height of the hub.
	|		  - if `hub_height` is null, there will be no hub.
	"""
	profile = z_multiple(gearprofile(step, z, **kwargs), z)

	square_radius = min(length2(v) for v in profile.points)
	min_radius = sqrt(square_radius) * 0.95
	coeff = 1 - int_height / min_radius if int_height else 0.95

	# Parts
	exterior = gearexterior(profile, min_radius, step, z, b, **kwargs)
	structure = gearstructure(pattern, coeff * min_radius, 2.5 * bore_radius, b, int_height, **kwargs)
	hub = gearhub(bore_radius, b, int_height, **kwargs)
	return geargather(exterior, structure, hub)


def gear_(radius, torque, step=None) -> Mesh:
	pass
