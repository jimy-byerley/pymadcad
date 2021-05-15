# This file is part of pymadcad,  distributed under license LGPL v3

from .mathutils import *
from .mesh import Web, Wire, Mesh, web
from . import settings
from .blending import junction, blendpair
from .generation import extrusion, revolution, repeat, extrans
from .primitives import Circle, ArcCentered, Segment
from .triangulation import triangulation, triangulation_closest
from .selection import *
from .rendering import show
from .settings import curve_resolution
from .cut import bevel
from math import cos, sin, asin, tan
from functools import reduce, partial
from operator import add
from copy import copy

# TODO: augmenter la surface disponible des dents en permettant un profil skysomorphique (utiliser des operations presques booleenes


def geartooth(profile, sector, m=1, resolution=None):
	''' correct a profile to allow it to be used on a gear where the tooth holds the given angular sector
		the module m can be provided to adjust curve resolution if it depends on the scale
	'''
	# perimeter of the profile
	total = 0
	for i in range(1,len(profile)):
		total += distance(profile[i-1],profile[i])
	# circular discrete profile
	res = settings.curve_resolution(m, pi, resolution)
	tooth = [vec2(profile[0])]
	for i in range(1,len(profile)):
		a = profile[i-1]
		b = profile[i]
		div = int(res * distance(a,b)/total)
		tooth += [interpol1(a,b, j/(div+1))		for j in range(1,div+2)]

	# correct the circular profile by removing intersections with the linear profile
	r = 1/sector
	rmin = min((y for x,y in profile))
	rmax = max((y for x,y in profile))
	# x is advancement of the gear (and rotation therefore)
	xlim = sqrt(1 - ((r+rmin)/(r+rmax))**2) * (r+rmax)
	x = 0
	i = 0
	while x >= -xlim:
		dx =  tooth[(i+1) % len(tooth)][0] - tooth[i % len(tooth)][0]
		if dx < 0:	dx += 1
		x -= dx
	while x <= xlim:
		dx =  tooth[(i+1) % len(tooth)][0] - tooth[i % len(tooth)][0]
		if dx < 0:	dx += 1

		# search intersections
		for k in range(len(tooth)):
			t = (tooth[k][0] - x)/r
			radial = vec2(sin(t), -cos(t))
			center = vec2(x, r)
			p = center + radial * tooth[k][1]
			for j in range(len(profile)-1):
				a, b = profile[j] * vec2(1,-1), profile[j+1] * vec2(1,-1)
				# compute intersection of radial vector with segment
				u,v = inverse(mat2(radial, b-a)) * (b-center)
				if 0 <= v and v <= 1:
					d = u - r
					if tooth[k][1] > d:
						tooth[k][1] = d
				#dir = a-b
				#dir = -normalize(vec2(dir[1], -dir[0]))
				##dir = normalize(radial*10 + vec2(0,-1))
				##dir = radial
				#u,v = inverse(mat2(dir, b-a)) * (b-p)
				#if 0 <= v and v <= 1 and u > 0:
					#drad = p+u*dir - center
					##drad = v*a + (1-v)*b - center
					#d = length(drad) - r
					#print(tooth[k], (r*atan2(drad[0], -drad[1]) + x, d))
					#tooth[k] = vec2(r*atan2(drad[0], -drad[1]) + x, d)
		x += dx
		i += 1
		#if i > 0:	break
	return tooth

from .mathutils import atan2, length, normalize

def racktooth(inclin, h1, h2=None):
	''' profile for iso normed gear
		inclin	  the contact angle
		h1		  the exterior height/module of the tooth
		h2		  the interior height/module of the tooth
	'''
	if h2 is None:	h2 = h1
	inclin = tan(inclin)
	profile = [vec2(0, h1), vec2(1/4-h1*inclin, h1), vec2(1/4+h2*inclin, -h2)]
	return profile + [vec2(1-x, y)  for x,y in reversed(profile)]

def gearprofile(profile, m, z, axis=(vec3(0,0,0), vec3(0,0,1)), align=vec3(1,0,0)):
	''' generate a circular gear profile using the tooth profile given
		m		the module
		z		the number of tooth on the profile (int)
	'''
	web = Web(groups=['gear'])
	sector = 2*pi/z
	r = z*m / (2*pi)
	o = axis[0]
	x,y,_ = dirbase(axis[1], align)
	last = None
	for i in range(z):
		for a,p in profile:
			t = (i+a) * sector
			if t == last:	continue
			last = t
			d = r+p*m
			web.points.append(d*cos(t)*x + d*sin(t)*y + o)
	for i in range(len(web.points)-1):
		web.edges.append((i,i+1))
		web.tracks.append(0)
	web.edges[-1] = (i,0)
	return web


def surfgear(profile, m, z, b, spin=0, axis=(vec3(0,0,0), vec3(0,0,1)), align=vec3(1,0,0), resolution=None):
	line = gearprofile(profile, m, z, axis, align)
	spin = b * tan(spin) * 2*pi/ (z*m)
	div = settings.curve_resolution(b, spin, resolution) + 2
	def trans():
		for i in range(div):
			x = i/(div-1)
			yield transform((x-0.5)*b*axis[1]) * transform(angleAxis(x*spin, axis[1]))

	return generation.extrans(line, trans(), ((i,i+1) for i in range(div-1)))

def surfscrewgear(profile, m, z, b, radius, n=1, axis=(vec3(0,0,0), vec3(0,0,1)), align=vec3(1,0,0), resolution=None):
	line = gearprofile(profile, m, z, axis, align)
	#r = z*m / (2*pi)		# gear primitive radius
	spin = n*b/(z*radius)	# spin angle for the extrusion
	angle = 2*asin(b/(2*radius))	# angular sector of the screw contact
	div = max(
			settings.curve_resolution(b, spin, resolution),
			settings.curve_resolution(angle*radius, angle, resolution),
			)+ 2
	def trans():
		for i in range(div):
			x = i/(div-1)
			t = (x-0.5)*angle
			s = 1 + radius*(1-cos(t)) * 2*pi/(z*m)
			h = radius*sin(t)
			yield transform(h*axis[1]) * transform(angleAxis(x*spin, axis[1])) * transform(s*scaledir(axis[1], 1/s))

	return generation.extrans(line, trans(), ((i,i+1) for i in range(div-1)))


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


def gearprofile(step, z, h=None, e=-0.05, x=0.5, alpha=radians(30), resolution=None) -> Wire:
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
	ext_radius: float, int_radius: float, depth: float, ratio: float = None
) -> (Web, Web, None):
	"""
	Generate two full parts of the structure (the top and the bottom).

	Return a tuple (Web, Web, None) where the first `Web` is the top of the structure and
	the second `Web` is the bottom of the structure.

	Parameters
	----------
	:ext_radius: float (radius of the external border of the pattern)
	:int_radius: float (radius of the internal border of the pattern)
	:depth: float (face width)
	:ratio: float (Not used)
	"""
	half_depth = depth / 2
	axis = (vec3(0, 0, 0), vec3(0, 0, 1))
	circles_ref = web(Circle(axis, ext_radius)) + web(Circle(axis, int_radius)).flip()
	top_profile = circles_ref.transform(vec3(0, 0, half_depth))
	bottom_profile = circles_ref.transform(vec3(0, 0, -half_depth))
	mesh = triangulation(top_profile) + triangulation(bottom_profile)
	mesh.mergeclose()
	return mesh

def circle_pattern(
	ext_radius: float,
	int_radius: float,
	depth: float,
	ratio: float = 1,
	n_circles: int = 5,
	r_int: float = None,
	r_ext: float = None,
) -> (Web, Web, Mesh):
	"""
	Generate two parts of the structure (the top and the bottom) with `n_circles` distributed on the whole structure.

	Return a tuple (Web, Web, Mesh) where the first `Web` is the top of the structure,
	the second `Web` is the bottom of the structure and the last element `Mesh` is all side surfaces.

	Parameters
	----------
	:ext_radius: float (radius of the external border of the structure)
	:int_radius: float (radius of the internal border of the structure)
	:depth: float (face width)
	:ratio: float (it is a number that allows to change proportionally the radius of circles)
	:n_circles: int (number of circles of the structure)
	:r_int: float (radius of circles)
	:r_ext: float (radius where the origins of circles are placed)

	Notes:
	- for instance, with a ratio of 1.5, the radius of circles `r_int` will be multiply by 1.5
	- if `r_int` is chosen, `ratio` won't impact the radius of circles `r_int`
	"""
	# Parameters
	half_depth = depth / 2
	if not (r_int) and not (r_ext):
		border_number = 4 * (ext_radius - int_radius) / (ext_radius + int_radius)
		error_msg = f"ratio must be in the intervalle ]0;{border_number}["
		assert 0 < ratio < border_number, error_msg
	r_ext = (ext_radius + int_radius) / 2 if not (r_ext) else r_ext
	r_int = r_ext * ratio / 4 if not (r_int) else r_int
	error_msg = "There are some circles who put themselves on top of each other. `n_circles` is too large or the value of `ratio` is too large if the value of `r` was defined by default."
	assert r_int / r_ext <= sin(pi / n_circles), error_msg
	assert r_int < r_ext, "`r_int` must be smaller than `r_ext` if they were chosen. Else you should reduce the value of `ratio`"
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
	top_webs = [wire.transform(vec3(0, 0, half_depth)) for wire in webs_ref]
	bottom_webs = [wire.transform(vec3(0, 0, -half_depth)) for wire in webs_ref]

	top_profile = reduce(add, top_webs)
	bottom_profile = reduce(add, bottom_webs)
	surfaces = extrusion(vec3(0, 0, depth), bottom_webs[0])
	mesh = triangulation(top_profile) + triangulation(bottom_profile) + surfaces
	mesh.mergeclose()
	return mesh


def create_rect_pattern(
	ext_radius: float,
	int_radius: int,
	depth: float,
	ratio: float = 1,
	n_patterns: int = 5,
	r_int: float = None,
	r_ext: float = None,
	rounded: bool = False,
):
	"""
	Function used for `rect_pattern` when `pattern` = "rect"
	and `rounded_pattern` when `pattern` = "rounded"

	Parameters
	----------
	:ext_radius: float (radius of the external border of the structure)
	:int_radius: float (radius of the internal border of the structure)
	:depth: float (face width)
	:ratio: float (it is a number which is the proportional factor for an homothety of patterns)
	:n_patterns: int (number of patterns inside the structure)
	:r_int: float (internal radius of the pattern)
	:r_ext: float (external radius of the pattern)
	:rounded: bool (if it is True, the pattern will be rounded)

	Notes:
	- for instance, with a ratio of 1.5, the pattern will be 1.5 bigger.
	- Choose a ratio equal to 1 if `r_int` and `r_ext` are chosen to not impact the pattern
	"""
	# Parameters
	half_depth = depth / 2
	r_ext = 0.9 * ext_radius if not (r_ext) else r_ext  # external radius
	r_int = int_radius + 0.1 * ext_radius if not (r_int) else r_int  # internal radius
	thickness = r_ext / 10  # thickness of "arms"
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

	# Pattern of reference
	R_m = (r_int + r_ext) / 2
	t_m = 0.5 * angle_step

	rt = ratio
	cyl2cart = lambda r, theta: vec3(r * cos(theta), r * sin(theta), 0)
	get_coords = lambda r, theta: (rt * (r - R_m) + R_m, rt * (theta - t_m) + t_m)
	A1 = cyl2cart(*get_coords(r_int, theta_1))
	A2 = cyl2cart(*get_coords(r_ext, theta_2))
	B1 = cyl2cart(*get_coords(r_int, angle_step - theta_1))
	B2 = cyl2cart(*get_coords(r_ext, angle_step - theta_2))
	arc_A1_B1 = ArcCentered((O, -Z), B1, A1)
	arc_A2_B2 = ArcCentered((O, Z), A2, B2)
	segmentA1_A2 = Segment(A1, A2)
	segmentB1_B2 = Segment(B2, B1)
	pattern_ref = web([segmentA1_A2, arc_A1_B1, segmentB1_B2, arc_A2_B2])

	# Multiply the pattern of reference
	if rounded: # rounded case
		groups = pattern_ref.groupextremities()
		bevel(pattern_ref, groups, ("radius", 0.1 * (r_ext - r_int)))
		pattern_ref.mergeclose()
	pattern_profile = repeat(pattern_ref, n_patterns, rotatearound(angle_step, axis))

	# Profiles and surfaces
	webs_ref = (pattern_profile.flip(), ext_circle, int_circle)
	top_webs = [web.transform(vec3(0, 0, half_depth)) for web in webs_ref]
	bottom_webs = [web.transform(vec3(0, 0, -half_depth)) for web in webs_ref]

	top_profile = reduce(add, top_webs)
	bottom_profile = reduce(add, bottom_webs)
	surfaces = extrusion(vec3(0, 0, depth), bottom_webs[0])

	mesh = triangulation(top_profile) + triangulation(bottom_profile) + surfaces
	mesh.mergeclose()
	return mesh

def rect_pattern(
	ext_radius: float,
	int_radius: int,
	depth: float,
	ratio: float = 1,
	n_patterns: int = 5,
	r_int: float = None,
	r_ext: float = None,
) -> (Web, Web, Mesh):
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
	:ratio: float (it is a number which is the proportional factor for an homothety of patterns)
	:n_patterns: int (number of patterns inside the structure)
	:r_int: float (internal radius of the pattern)
	:r_ext: float (external radius of the pattern)

	Notes:
	- for instance, with a ratio of 1.5, the pattern will be 1.5 bigger.
	- Choose a ratio equal to 1 if `r_int` and `r_ext` are chosen to not impact the pattern
	"""
	return create_rect_pattern(ext_radius, int_radius, depth, ratio, n_patterns, r_int, r_ext, False)

def rounded_pattern(
	ext_radius: float,
	int_radius: int,
	depth: float,
	ratio: float = 1,
	n_patterns: int = 5,
	r_int: float = None,
	r_ext: float = None,
) -> (Web, Web, Mesh):
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
	:ratio: float (it is a number which is the proportional factor for an homothety of patterns)
	:n_patterns: int (number of patterns inside the structure)
	:r_int: float (internal radius of the pattern)
	:r_ext: float (external radius of the pattern)

	Notes:
	- for instance, with a ratio of 1.5, the pattern will be 1.5 bigger.
	- Choose a ratio equal to 1 if `r_int` and `r_ext` are chosen to not impact the pattern
	"""
	return create_rect_pattern(ext_radius, int_radius, depth, ratio, n_patterns, r_int, r_ext, True)


def gearexterior(
	profile: Web,
	min_radius: float,
	step: float,
	z: int,
	depth: float,
	helix_angle: float = radians(20),
	bevel_angle: bool = False,
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
	:bevel_angle: bool (boolean to get a bevel angle - only for straight pinion)
	"""
	half_depth = depth / 2

	# Internal circles
	axis = (vec3(0, 0, 0), vec3(0, 0, 1))
	circle_ref = Circle(axis, min_radius, resolution=("div", z)).mesh()
	top_circle = circle_ref.transform(vec3(0, 0, half_depth))
	bottom_circle = circle_ref.transform(vec3(0, 0, -half_depth))

	if bevel_angle: #bevel case
		pr = step * z / (2 * pi)  # primitive radius
		square_pr = pr ** 2
		Z = vec3(0, 0, 1)
		k = tan(pi / 4)
		cp = deepcopy(profile)
		truncated = deepcopy(profile)
		for i, point in enumerate(profile.points):
			r = length2(point)
			if r > square_pr:
				r = sqrt(r)
				profile.points[i] = point - vec3(0, 0, (r - pr) * k)
				truncated.points[i] = point * pr / r

		top_profile = profile.transform(vec3(0, 0, half_depth))
		bottom_profile = top_profile.transform(scaledir(vec3(0, 0, 1), -1))
		top_truncated = truncated.transform(vec3(0, 0, half_depth))
		bottom_truncated = truncated.transform(vec3(0, 0, -half_depth))

		bp = partial(blendpair, tangents = "straight")
		top_surface = bp(top_truncated.flip(), top_circle)
		bottom_surface = bp(bottom_truncated.flip(), bottom_circle)
		top_bevel = bp(top_truncated, top_profile.flip())
		bottom_bevel = bp(bottom_truncated.flip(), bottom_profile)
		gear_surface = bp(top_profile, bottom_profile.flip())

		surfaces = (top_surface, bottom_surface, top_bevel, bottom_bevel, gear_surface)
		mesh = reduce(add, surfaces)
		mesh.mergeclose()
	else:
		if helix_angle: # helical teeth case
			# Edges of teeth
			# step = 50
			square_radius = max(length2(v) for v in profile.points)
			R = sqrt(square_radius)
			step = curve_resolution(depth / cos(helix_angle), depth * tan(helix_angle) / R)
			step = step if step else 1
			Z = vec3(0, 0, 1)
			angle = helix_angle / step
			h = depth / step
			bottom_gear_edge = profile.transform(vec3(0, 0, -half_depth))
			transformations = (transform((vec3(0, 0, i * h), angleAxis(angle * i, Z))) for i in range(step + 1))
			links = ((i, i + 1, 0) if i - step else (i, 0, 0) for i in range(step + 1))
			gear_surface = extrans(bottom_gear_edge, transformations, links)
			t = transform((vec3(0, 0, depth), angleAxis(helix_angle, Z)))
			top_gear_edge = bottom_gear_edge.transform(t)
		else: # straight teeth case
			# Edges of teeth
			bottom_gear_edge = profile.transform(vec3(0, 0, -half_depth))
			top_gear_edge = profile.transform(vec3(0, 0, half_depth))
			gear_surface = extrusion(vec3(0, 0, depth), bottom_gear_edge)

		# Surfaces
		top_surface = junction(top_gear_edge.flip(), top_circle, tangents="straight")
		bottom_surface = junction(bottom_gear_edge.flip(), bottom_circle, tangents="straight")
		mesh = gear_surface + top_surface + bottom_surface
		mesh.mergeclose()
	return mesh


def gearstructure(
	pattern: str, ext_radius: float, int_radius: float, depth: float, ratio: float = 1
) -> Mesh:
	"""
	Generate the internal part of the pinion
	:ext_radius: float (it is given by the attribut `_radius` of the result of `z_multiple` -
	| to avoid interference, it must be smaller than `_radius` (for instance 0.95 * `_radius`))
	:int_radius: float (it is the same radius of the largest radius of the hub part)
	:depth: float (face width)
	:pattern: str (values: 'full', 'circle', 'rect', 'rounded')
	"""
	# int_radius must not be 0
	int_radius = int_radius if int_radius else 0.1 * ext_radius
	return eval(pattern + "_pattern")(ext_radius, int_radius, depth, ratio)


def gearhub(
	bore_radius: float, depth: float, hub_height: float = None, hub_radius: float = None
) -> Web:
	"""
	Generate a hub for a pinion part

	Parameters
	----------
	:bore_radius: float (radius of the central bore)
	:depth: float (face width; same parameter for `gearexterior` and `gearstructure`)
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
		top_surface = circle_ref.transform(vec3(0, 0, half_depth))
		bottom_surface = circle_ref.transform(vec3(0, 0, -half_depth))
		return top_surface + bottom_surface

	if hub_height is None:
		hub_height = bore_radius
	if not (hub_radius):
		hub_radius = 2.5 * bore_radius
	axis = (vec3(0, 0, hub_height + depth / 2), vec3(0, 0, 1))
	bore_circle = Circle(axis, bore_radius)
	hub_circle = Circle(axis, hub_radius)

	# Top part
	top_surface = triangulation(web(bore_circle).flip() + web(hub_circle))

	# Lateral surfaces
	lateral_surfaces = extrusion(vec3(0, 0, -hub_height - depth), web(bore_circle))
	if hub_height:  # Hub case
		lateral_surfaces += extrusion(vec3(0, 0, -hub_height), web(hub_circle).flip())

	# Bottom part
	axis = (vec3(0, 0, -depth / 2), vec3(0, 0, 1))
	bottom_bore_circle = Circle(axis, bore_radius)
	bottom_hub_circle = Circle(axis, hub_radius)
	bottom_web = web(bottom_bore_circle).flip() + web(bottom_hub_circle)
	bottom_surface = triangulation(bottom_web)

	return top_surface + bottom_surface + lateral_surfaces


def geargather(exterior: Mesh, structure: Mesh, hub: Mesh) -> Mesh:
	"""
	Gather all parts (`exterior`, `structure` and `hub` parts)
	"""
	exterior.mergeclose()
	structure.mergeclose()
	hub.mergeclose()

	direction = vec3(1, 1, 0)
	Z = vec3(0, 0, 1)
	height = 0
	ext_radius = int_radius = length2(structure.points[0] * direction)
	for p in structure.points:
		r = length2(p * direction)
		h = dot(p, Z)
		if r > ext_radius:
			ext_radius = r
		if r < int_radius:
			int_radius = r
		if h > height:
			height = h
	ext_radius = sqrt(ext_radius)
	int_radius = sqrt(int_radius)

	circle_int_e = select(exterior, vec3(0, 0, height))
	circle_ext_s = select(structure, vec3(ext_radius, 0, height))
	edge = circle_ext_s.edges[0]
	if edge[0] > edge[1]:
		circle_ext_s = circle_ext_s.flip()
	j1 = junction(circle_ext_s, circle_int_e, tangents='straight')
	j1 = j1 + j1.transform(vec3(0, 0, -2 * height))

	direction = vec3(1, 1, 0)
	hub_ext_radius = sqrt(max(length2(p * direction) for p in hub.points))
	circle_ext_h = select(hub, vec3(hub_ext_radius, 0, height))
	circle_int_s = select(structure, vec3(int_radius, 0, height))
	edge = circle_int_s.edges[0]
	if edge[0] < edge[1]:
		circle_int_s = circle_int_s.flip()
	j2 = junction(circle_ext_h, circle_int_s, tangents='straight')
	j2 = j2 + j2.transform(vec3(0, 0, -2 * height))

	mesh = exterior + j1 + structure + j2 + hub
	mesh.mergeclose()
	return mesh

def gear(
	step: float,
	z: int,
	b: float,
	helix_angle: float = radians(20),
	x: float = 0,
	bore_radius: float = 0,
	hub_height: float = None,
	pattern: str = "full",
	ratio: int = 1,
) -> Mesh:
	"""
	Generate a pinion

	Parameters
	----------
	:step: float (step of chordal pitch)
	:z: int (number of teeth)
	:b: float (face width)
	:helix_angle: float (helix angle to get a helical pinion)
	:x: float (offset of tooth)
	:main_bore_radius: float (radius of the main bore)
	:hub_height: float (height of the hub shoulder)
	:pattern: str (determine the structure of pinion)
	:ratio: float (influence the proportion of dimensions of the structure)
	"""
	profile = z_multiple(gearprofile(step, z), z)

	square_radius = min(length2(v) for v in profile.points)
	min_radius = sqrt(square_radius) * 0.95

	# Parts
	exterior = gearexterior(profile, min_radius, step, z, b, helix_angle)
	structure = gearstructure(pattern, 0.95 * min_radius, 2.5 * bore_radius, b, ratio)
	hub = gearhub(bore_radius, b, hub_height)
	return geargather(exterior, structure, hub)


def gear_(radius, torque, step=None) -> Mesh:
	pass
