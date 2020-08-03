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
from scipy.optimize import minimize
from .nprint import nprint
from .mathutils import project, normalize, length, vec3, distance, anglebt, cross, dot, noproject, atan2, pi, cos, sin
from . import primitives
from . import displays, text, settings

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
		return length(cross(self.c1.slv_tangent(self.p), self.c2.slv_tangent(self.p))) **2
	
	def display(self, scene):
		return displays.TangentDisplay(scene, (self.p, self.c2.slv_tangent(self.p)), self.size),

class Distance(Constraint):
	''' Makes two points distant of the fixed given distance '''
	__slots__ = 'p1', 'p2', 'd', 'location'
	slvvars = 'p1', 'p2'
	def fit(self):
		return (distance(self.p1, self.p2) - self.d) **2
	#def fitgrad(self):
		#return derived.compose(dot, Derived(self.p1), Derived(self.p2))
	def display(self, scene):
		arrows = displays.LengthMeasure(scene, self.p1, self.p2, location=self.location or (self.p1+self.p2)*0.6)
		measure = text.TextDisplay(scene, arrows.textplace, 
					text='{}\n({:+0.3g})'.format(self.d, self.d-distance(self.p1, self.p2)),
					align=(-1,1))
		return arrows, measure

class Angle(Constraint):
	''' Gets two segments with the given fixed angle between them '''
	__slots__ = 's1', 's2', 'angle', 'location'
	slvvars = 's1', 's2'
	def fit(self):
		d1 = self.s1.direction
		d2 = self.s2.direction
		a = atan2(length(cross(d1,d2)), dot(d1,d2))
		return (a - self.angle)**2
	#def display(self, scene):
		#return displays.ArcMeasure(scene, arc, 

def Parallel(s1,s2):
	''' Strict equivalent of Angle(s1,s2,0) '''
	return Angle(s1,s2,0)

class Radius(Constraint):
	''' Gets the given Arc with the given fixed radius 
		
		Note: Only ArcCentered are supported yet.
	'''
	__slots__ = 'arc', 'radius', 'location'
	slvvars = 'arc',	
	def fit(self):
		#ra = length(noproject(self.arc.a - self.arc.axis[0], self.arc.axis[1]))
		#rb = length(noproject(self.arc.b - self.arc.axis[0], self.arc.axis[1]))
		#return (ra - self.radius) **2 + (rb - self.radius) **2
		return (self.arc.radius - self.radius) **2
	
	def display(self, scene):
		dra = abs(length(noproject(self.arc.a - self.arc.axis[0], self.arc.axis[1])) - self.radius)
		drb = abs(length(noproject(self.arc.b - self.arc.axis[0], self.arc.axis[1])) - self.radius)
		center, z = self.arc.axis
		v = noproject(self.arc.a-center, z)
		r = length(v)
		x = v/r
		y = cross(z, x)
		angle = atan2(dot(self.arc.b-center,y), dot(self.arc.b-center,x)) % (2*pi)
		location = self.location or center + 2*r * (cos(angle/2)*x + sin(angle/2)*y)
		arrows = displays.RadiusMeasure(scene, primitives.Circle(self.arc.axis, self.radius), location)
		measure = text.TextDisplay(scene, arrows.textplace,
					text='{}\n({:+0.3g})'.format(self.radius, max(dra, drb)),
					align=(-0.5, 0.5))
		return arrows, measure

class Projected(Constraint):
	__slots__ = 'a', 'b', 'proj'
	slvvars = 'a', 'b'
	def fit(self):
		return dot(self.a - self.b - self.proj, self.proj)

class OnPlane(Constraint):
	''' Puts the given points on the fixed plane given by its normal axis '''
	__slots__ = 'axis', 'pts'
	def slvvars(self):
		return self.pts
	def fit(self):
		s = 0
		for p in self.pts:
			s += dot(p-self.axis[0], self.axis[1]) **2
		return s

class PointOn(Constraint):
	''' Puts the given point on the curve.
	
		The curve primitive must have a member  ``slv_nearest(p) -> vec3`` returning the closest point to p on the curve.
	'''
	__slots__ = 'point', 'curve'	
	slvvars = 'point', 'curve'
	def fit(self):
		return distance(self.curve.slv_nearest(self.point), self.point) ** 2

		
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
				if isinstance(v, (float,int)):
					del self.slvvars[(id(obj), varname)]
				else:
					self.unregister(v)
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
		return sum((fit()  for fit in self.constraints))
	def evaluate(self, x):
		self.place(x)
		return self.fit()
	
	def solve(self, precision=1e-4, method='BFGS', afterset=None, maxiter=None):
		#nprint(self.slvvars)
		res = minimize(self.evaluate, self.state(), 
				tol=precision, method=method, callback=afterset, 
				options={'eps':precision/2, 'maxiter':maxiter})
		if res.fun <= precision:
			self.place(res.x)
			return res
		elif not res.success:
			print(res)
			raise SolveError(res.message)
		else:
			raise SolveError('no solution found')

'''
def solve_old(constraints, precision=1e-4, afterset=None, fixed=()):
	corrections = {}
	params = {}
	rate = 1
	max_delta = inf
	
	# recuperation des parametres a modifier
	for const in constraints:
		for param in const.params():
			corrections[id(param)] = None
			params[id(param)] = param
			
	# resolution iterative
	while max_delta > precision:
		max_delta = 0
		
		for k in corrections:
			corrections[k] = vec3(0)
			
		for const in constraints:
			corr = list(zip(const.params(), const.corrections()))
			for i in reversed(range(len(corr))):
				if id(corr[i][0]) in fixed:	corr.pop(i)
			if not len(corr):	continue
			f = rate / len(corr)
			for p,correction in corr:
				corrections[id(p)] += correction * f
		
		print('corrections', corrections)
		
		for k,correction in corrections.items():
			corr_norm = length(correction)
			if corr_norm > max_delta:	max_delta = corr_norm
			params[k] += correction
		
		#print(max_delta)
		if afterset:	afterset()

	return max_delta
'''

class SolveError(Exception):	pass

'''
def geoparams(constraints):
	knownparams = set()
	# get parameters and prepare solver
	for const in constraints:
		for param in const.params():
			k = id(param)
			if k not in knownparams:
				knownparams.add(k)
				yield param
'''

from math import inf

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
