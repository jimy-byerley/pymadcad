# This file is part of pymadcad,  distributed under license LGPL v3
'''
	This module provide functions to generate racks and gears with many different shapes.
	
	The mainstream gears are involute gears (using involute of circles as contact curves). This is due to the standard shape of racks. And involute gears are the best choice for most designs, which is why the current implementation focusses on them.
	
	However the tools here are modular, which means you can combine them with your own functions to create customized gears.
'''

from .mathutils import *
from .mesh import Web, Wire
from . import settings
from . import generation


def rackprofile(step, h=None, offset=0, alpha=radians(30), resolution=None) -> Wire:
	''' Generate a 1-period tooth profile for a rack
	
		Parameters:
			:step:		period length over the primitive line
			:h:			tooth half height 
			:offset:	rack reference line offset with the primitive line
			
						- the primitive line is the adherence line with gears
						- the reference line is the line half the tooth is above and half below
			:alpha:		angle of the tooth sides
	'''
	if h is None:
		h = default_height(step, alpha)
	e = offset  # change name for convenience
	x = 0.5 + 2*offset/p*tan(alpha)  # fraction of the tooth above the primitive circle
	
	return Wire([
		vec3(step*x/2 - tan(alpha) * ( h-e),   h-e,  0),
		vec3(step*x/2 - tan(alpha) * (-h-e),  -h-e,  0),
		vec3(step*(2-x)/2 + tan(alpha) * (-h-e),  -h-e,  0),
		vec3(step*(2-x)/2 + tan(alpha) * ( h-e),   h-e,  0),
		vec3(step*(2+x)/2 - tan(alpha) * ( h-e),   h-e,  0),
		], groups=['rack'])


def gearprofile(step, z, h=None, offset=0, alpha=radians(30), resolution=None) -> Wire:
	''' Generate a 1-period tooth profile for a straight gear
	
		Parameters:
			:step:		period length over the primitive circle
			:z:			number of tooth on the gear this profile is meant for
			:h:			tooth half height
			:offset:	offset of the matching rack profile (see above)
			:alpha:		pressure angle in the contact
	'''
	if h is None:
		h = default_height(step, alpha)
	p = step*z / (2*pi)	# primitive circle
	c = p * cos(alpha)	# tangent circle to gliding axis
	e = offset  # change name for convenience
	#e = offset + 0.05*h  # the real offset is 5% (the 1*m, 1.25*m) when the standard says it is 0
	x = 0.5 + 2*offset/p*tan(alpha)  # fraction of the tooth above the primitive circle
	
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

