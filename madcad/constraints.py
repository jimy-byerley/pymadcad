# This file is part of pymadcad,  distributed under license LGPL v3

''' This modules defines the constraints definitions and the solver tools

	Constraints can be any object referencing `variables` and implementing the following signature to guide the solver in the resolution:
		
		class SomeConstraint:
			
			# name the attributes referencing the solver variables to change
			slvvars = 'some_primitive', 'some_point'
			# function returning the squared error in the constraint
			
			#  for a coincident constraint for instance it's the squared distance
			#  the error must be a contiguous function of the parameters, and it's squared for numeric stability reasons.
			#  the solver internally use an iterative approach to optimize the sum of all fit functions.
			def fit((self):
				...
'''

from collections import Counter
import numpy as np
from scipy.optimize import least_squares
from .nprint import nprint
from .mathutils import *
from . import primitives
from . import displays, text, settings
from . import scheme
import array


class SolveError(Exception):	pass

class Constraint(object):
	def __init__(self, *args, **kwargs):
		for i,name in enumerate(self.__slots__):
			setattr(self, name, args[i] if i < len(args) else None)
		for name, arg in kwargs.items():
			setattr(self, name, arg)
	#def fitgrad(self):
		#varset = Varset()
		#for name in self.primitives:
			#varset.register(getattr(self, name))
		#return Derived(varset.state(), varset.grad(), varset.vars)

def isconstraint(obj):
	''' return True if obj match the constraint signature '''
	return hasattr(obj, 'fit') and hasattr(obj, 'slvvars')

	
	


class Tangent(Constraint):
	''' Makes to curves tangent in the given point 
		The point moves as well as curves.
		
		The curves primitives must have a member ``slv_tangent(p)`` returning the tangent vector at the nearest position to `p`
	'''
	__slots__ = 'c1', 'c2', 'p', 'size'
	slvvars = 'c1', 'c2', 'p'
	def fit(self):
		return length2(cross(self.c1.slv_tangent(self.p), self.c2.slv_tangent(self.p))),
	
	#def display(self, scene):
		#return displays.TangentDisplay(scene, (self.p, self.c2.slv_tangent(self.p)), self.size)

class Distance(Constraint):
	''' Makes two points distant of the fixed given distance '''
	__slots__ = 'p1', 'p2', 'd', 'along', 'location'
	slvvars = 'p1', 'p2'
	def fit(self):
		if isinstance(self.p1, vec3) and isinstance(self.p2, vec3):
			if self.along:
				if isinstance(along, vec3):	a = along
				else:						a = along.direction
				return (dot(self.p1-self.p2, a) - d) ** 2,
			else:
				return (distance(self.p1, self.p2) - self.d) **2,
		elif isinstance(self.p1, vec3):
			return (length(noproject(self.p2.origin-self.p1, self.p2.direction)) - self.d) **2,
		elif isinstance(self.p2, vec3):
			return (length(noproject(self.p1.origin-self.p2, self.p1.direction)) - self.d) **2,
		else:
			d1 = self.p1.direction
			d2 = self.p2.direction
			if dot(d1,d2) < 0:	d2 = -d2
			return length2(cross(d1,d2)), (length(noproject(self.p1.origin-self.p2.origin, d1+d2)) - self.d) **2
			
	#def fitgrad(self):
		#return derived.compose(dot, Derived(self.p1), Derived(self.p2))
	def display(self, scene):
		if isinstance(self.p1, vec3) and isinstance(self.p2, vec3):
			return scene.display(scheme.note_distance(
					self.p1, self.p2, 
					project=self.along,
					text='{:.5g}\n{:+.1g}'.format(self.d, sqrt(self.fit()[0])),
					))
		elif isinstance(self.p1, vec3):
			return scene.display(scheme.note_distance(
					self.p1, self.p1 + noproject(self.p2.origin-self.p1, self.p2.direction),
					text='{:.5g}\n{:+.1g}'.format(self.d, sqrt(self.fit()[0])),
					))
		elif isinstance(self.p2, vec3):
			return scene.display(scheme.note_distance(
					self.p2, self.p2 + noproject(self.p1.origin-self.p2, self.p1.direction), 
					text='{:.5g}\n{:+.1g}'.format(self.d, sqrt(self.fit()[0])),
					))
		else:
			p = mix(self.p1.origin, self.p2.origin, 0.5)
			d1 = self.p1.direction
			d2 = self.p2.direction
			if dot(d1,d2) < 0:	d2 = -d2
			d = d1 + d2
			p1 = self.p1.origin
			p2 = self.p2.origin
			return scene.display(scheme.note_distance(
					p + noproject(p1-p, d),
					p + noproject(p2-p, d),
					text='{:.5g}\n{:+.1g}'.format(self.d, sqrt(self.fit()[0])),
					))
			

class Angle(Constraint):
	''' Gets two segments with the given fixed angle between them '''
	__slots__ = 's1', 's2', 'angle'
	slvvars = 's1', 's2'
	def fit(self):
		d1 = self.s1.direction
		d2 = self.s2.direction
		a = atan2(length(cross(d1,d2)), dot(d1,d2))
		return (a - self.angle)**2,
	
	def display(self, scene):
		return scene.display(scheme.note_angle(
					(self.s1.origin, -self.s1.direction),
					(self.s2.origin, -self.s2.direction),
					text='{:.5g}°\n{:+.1g}'.format(degrees(self.angle), degrees(sqrt(self.fit()[0]))),
					))

class Parallel(Constraint):
	''' Strict equivalent of Angle(s1,s2,0) '''
	__slots__ = 's1', 's2'
	slvvars = __slots__
	def fit(self):
		d1 = self.s1.direction
		d2 = self.s2.direction
		return length2(cross(d1,d2)),
		

class Radius(Constraint):
	''' Gets the given Arc with the given fixed radius 
		
		Note: Only ArcCentered are supported yet.
	'''
	__slots__ = 'arc', 'radius', 'location'
	slvvars = 'arc',	
	def fit(self):
		return (self.arc.radius - self.radius) **2,
	
	def display(self, scene):
		r = self.arc.radius
		center, z = self.arc.axis
		x = dirbase(z)[0]
		return scene.display(scheme.note_leading(
					center+x*r, 
					r*x, 
					text='R{:.5g}\n{:+.1g}'.format(r, self.radius - self.arc.radius),
					))

class OnPlane(Constraint):
	''' Puts the given points on the fixed plane given by its normal axis '''
	__slots__ = 'axis', 'pts'
	def slvvars(self):
		return self.pts
	def fit(self):
		s = 0
		for p in self.pts:
			yield dot(p-self.axis[0], self.axis[1]) **2

class PointOn(Constraint):
	''' Puts the given point on the curve.
	
		The curve primitive must have a member  ``slv_nearest(p) -> vec3`` returning the closest point to p on the curve.
	'''
	__slots__ = 'point', 'curve'	
	slvvars = 'point', 'curve'
	def fit(self):
		return distance2(self.curve.slv_nearest(self.point), self.point),

		
		

def solve(constraints, fixed=(), *args, **kwargs):
	''' short hand to use the class Problem '''
	return Problem(constraints, fixed).solve(*args, **kwargs)

class Problem:
	''' class to holds data for a problem solving process.
		it is intended to be instantiated for each different probleme solving, an instance is used multiple times only when we want to solve on top of the previous results, using exactly the same probleme definition (constraints and variables)
		
		therefore the solver protocol is the follownig:
			- constraints define the probleme
			- each constraint refers to variables it applies on
				constraints have the method fit() and a member 'slvvars' that can be  
				
				1. an iterable of names of variable members in the constraint object
				2. a function returning an iterable of the actual variables objects (that therefore must be referenced refs and not primitive types)
			
			- each variable object can redirect to other variable objects if they implements such a member 'slvvars'
			- primitives can also be constraints on their variables, thus they must have a method fit()   (but no member 'primitives' here)
			- primitives can implement the optional solver methods for some constraints, such as 'slv_tangent'
		
		solving method
			Internally, this class uses scipy.optimize.minimize. Therefore the scipy minimization methods using only the gradient are all available. The mose usefull may be:
			
			BFGS	fast even for complex problems (few variables)
						https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
			CG		for problems with tons of variables	or for nearby solutions
						https://en.wikipedia.org/wiki/Conjugate_gradient_method
			Powell	for simple non-continuous problems with tons of variables
						https://en.wikipedia.org/wiki/Powell%27s_method
			
			default is `method='BFGS'`
	'''
	def __init__(self, constraints, fixed=()):
		self.constraints = set()
		self.slvvars = {}
		self.dim = 0
		for cst in constraints:
			# cst can contains objects that are not constraints
			if hasattr(cst, 'fit'):
				self.register(cst)
		for prim in fixed:
			self.unregister(prim)
		for v in self.slvvars.values():
			if isinstance(v, tuple):	self.dim += 1
			else:						self.dim += len(v)
	
	def register(self, obj):
		''' register a constraint or a variable object '''
		if hasattr(obj, 'fit'):
			self.constraints.add(obj.fit)
		# register object's variables
		if hasattr(obj, 'slvvars'):
			if callable(obj.slvvars):
				for var in obj.slvvars():
					if isinstance(var, (float, int)):		raise TypeError("primitive types (float,int) are not allowed when 'slvvars' is a callable")
					self.register(var)
			else:
				for varname in obj.slvvars:
					var = getattr(obj, varname)
					if isinstance(var, (float,int)):
						self.slvvars[(id(obj), varname)] = (obj, varname)
					else:
						self.register(var)
		elif isinstance(obj, tuple):
			for p in obj:
				self.register(p)
		else:
			if id(obj) not in self.slvvars:
				self.slvvars[id(obj)] = obj
	
	def unregister(self, obj):
		''' unregister all variables from a constraint or a variable object '''
		if hasattr(obj, 'slvvars'):
			if callable(obj.slvvars):
				for var in obj.slvvars():
					self.unregister(var)
			for varname in obj.slvvars:
				var = getattr(obj, varname)
				if isinstance(var, (float,int)):
					del self.slvvars[(id(obj), varname)]
				else:
					self.unregister(var)
		elif isinstance(obj, tuple):
			for p in obj:
				self.unregister(p)
		else:
			if id(obj) in self.slvvars:
				del self.slvvars[id(obj)]
	
	def state(self):
		x = np.empty(self.dim, dtype='f8')
		i = 0
		for v in self.slvvars.values():
			l = len(v)
			x[i:i+l] = v
			i += l
		return x
	
	def place(self, x):
		i = 0
		for v in self.slvvars.values():
			l = len(v)
			for j in range(l):	v[j] = x[i+j]
			i += l
	
	def fit(self):
		residuals = array.array('d')
		for fit in self.constraints:
			residuals.extend(fit())
		return residuals
	
	def evaluate(self, x):
		self.place(x)
		return self.fit()
	
	def solve(self, precision=1e-6, method='trf', maxiter=None, afterset=None):
		#nprint(self.slvvars)
		if afterset:	evaluate = lambda x: afterset(x) or self.evaluate(x)
		else:			evaluate = self.evaluate
		res = least_squares(evaluate, self.state(), 
				xtol=precision, 
				gtol=precision**3,
				ftol=precision**2,
				method=method, 
				max_nfev=maxiter,
				)
		
		self.place(res.x)
		#print(res)
		if res.cost < precision:
			return res
		elif not res.sucess:
			raise SolveError(res.message)
		else:
			raise SolveError('no solution found')




def solve2(constraints, precision=1e-4, afterset=None, fixed=(), maxiter=0):
	params = []
	corrections = []
	corrnorms = []
	
	indices = []
	knownparams = {}
	
	parts = []
	
	mincorr = precision * 1e-2
	
	# get parameters and prepare solver
	i = 0
	for const in constraints:
		c = 0
		for param in const.params():
			k = id(param)
			if k not in fixed:	c += 1
			if k not in knownparams:
				knownparams[k] = i
				params.append(param)
				corrections.append(None)
				corrnorms.append(0)
				i += 1
			indices.append(knownparams[k])
		parts.append(c)
	
	# iterative resolution
	oldcorrs = [None] * len(corrections)
	maxdelta = inf
	it = 0
	while maxdelta > precision:
		maxcorr = 0
		maxdelta = 0
		
		# initialize corrections
		for i in range(len(corrections)):
			corrections[i] = type(params[i])()
			corrnorms[i] = 0.
		
		# compute constraints contributions to corrections
		i = 0
		for const,part in zip(constraints, parts):
			corr = const.corrections()
			for correction in corr:
				contrib = correction / part
				corrections[indices[i]] += contrib
				l = length(contrib)
				if l > corrnorms[indices[i]]:	corrnorms[indices[i]] = l
				i += 1
		
		# apply changes
		for param,correction,corrnorm,oldcorr in zip(params, corrections, corrnorms, oldcorrs):
			if id(param) not in fixed:
				l = length(correction)
				if l > maxcorr:		maxcorr = l
				if corrnorm > maxdelta:		maxdelta = corrnorm
				# filtre a oscillation
				if oldcorr:
					#print('       ', correction*0.5 + oldcorr*0.5)
					param += (correction*0.5 + oldcorr*0.5)
				else:
					param += correction * 0.8
		oldcorrs = corrections[:]
		
		if afterset:	afterset()
		it += 1
		# check that the solver is solving
		if maxiter and it > maxiter:	
			raise SolveError('failed to converge with the allowed iteration count: '+str(maxiter))
		if maxdelta > precision and maxcorr < mincorr:	
			raise SolveError('resolution is blocked (try an other initial state)')

	#print('iterations:', it-1)
	return maxdelta
