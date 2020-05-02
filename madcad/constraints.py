from collections import Counter
import numpy as np
from scipy.optimize import minimize
from nprint import nprint
from .mathutils import project, normalize, length, vec3, distance, anglebt, cross, dot, noproject, atan2
from . import primitives

class Constraint(object):
	def __init__(self, *args, **kwargs):
		for name, arg in zip(self.__slots__, args):
			setattr(self, name, arg)
		for name, arg in kwargs.items():
			setattr(self, name, arg)
	def fitgrad(self):
		varset = Varset()
		for name in self.primitives:
			varset.register(getattr(self, name))
		return Derived(varset.state(), varset.grad(), varset.vars)

class Tangent(Constraint):
	__slots__ = 'c1', 'c2', 'p'
	primitives = 'c1', 'c2', 'p'
	def fit(self):
		return length(cross(self.c1.slv_tangent(self.p), self.c2.slv_tangent(self.p))) **2

class Distance(Constraint):
	__slots__ = 'p1', 'p2', 'd'
	primitives = 'p1', 'p2'
	def fit(self):
		return (distance(self.p1, self.p2) - self.d) **2
	def fitgrad(self):
		return derived.compose(dot, Derived(self.p1), Derived(self.p2))

class Angle(Constraint):
	__slots__ = 's1', 's2', 'angle'
	primitives = 's1', 's2'
	def fit(self):
		d1 = self.s1.direction()
		d2 = self.s2.direction()
		a = atan2(length(cross(d1,d2)), dot(d1,d2))
		return (a - self.angle)**2

def Parallel(s1,s2):
	return Angle(s1,s2,0)

class Radius(Constraint):
	__slots__ = 'arc', 'radius' #'consta', 'constb'
	primitives = 'arc',
	def __init__(self, arc, radius):
		self.arc = arc
		self.radius = radius
	
	def fit(self):
		ra = length(noproject(self.arc.a - self.arc.axis[0], self.arc.axis[1]))
		rb = length(noproject(self.arc.b - self.arc.axis[0], self.arc.axis[1]))
		return (ra - self.radius) **2 + (rb - self.radius) **2

class Projected(Constraint):
	__slots__ = 'a', 'b', 'proj'
	primitives = 'a', 'b'
	def fit(self):
		return dot(self.a - self.b - self.proj, self.proj)

class OnPlane(Constraint):
	__slots__ = 'axis', 'pts'
	primitives = 'pts',
	def fit(self):
		s = 0
		for p in self.pts:
			s += dot(p-self.axis[0], self.axis[1]) **2
		return s

class PointOn(Constraint):
	__slots__ = 'point', 'curve'	
	primitives = 'point', 'curve'
	def fit(self):
		return distance(self.curve.slv_nearest(self.point), self.point) ** 2

		
def solve(constraints, fixed=(), *args, **kwargs):
	return Problem(constraints, fixed).solve(*args, **kwargs)

class Problem:
	def __init__(self, constraints, fixed=()):
		self.constraints = constraints
		self.slvvars = []
		self.identified = Counter()
		self.dim = 0
		for cst in constraints:
			for pname in cst.primitives:
				primitive = getattr(cst, pname)
				self.register(primitive)
		for prim in fixed:
			self.unregister(prim)
		for v in self.slvvars:
			if isinstance(v, tuple):	self.dim += 1
			else:						self.dim += len(v)
	
	def register(self, primitive):
		if hasattr(primitive, 'slv_vars'):
			for varname in primitive.slv_vars:
				v = getattr(primitive, varname)
				if isinstance(v, (float,int)):
					self.slvvars.append((primitive, varname))
				else:
					self.register(v)
		elif isinstance(primitive, tuple):
			for p in primitive:
				self.register(p)
		else:
			if id(primitive) not in self.identified:
				self.slvvars.append(primitive)
				self.identified[id(primitive)] += 1
	def unregister(self, primitive):
		if hasattr(primitive, 'slv_vars'):
			for varname in primitive.slv_vars:
				v = getattr(primitive, varname)
				if isinstance(v, (float,int)):
					for i,r in enumerate(self.slvvars):
						if r[0] is primitive and r[1] == varname:
							self.identified.pop(i)
							break
				else:
					self.unregister(v)
		elif isinstance(primitive, tuple):
			for p in primitive:
				self.unregister(p)
		else:
			if id(primitive) in self.identified:
				del self.identified[id(primitive)]
				for i,p in enumerate(self.slvvars):
					if p is primitive:
						self.slvvars.pop(i)
						break
	
	def state(self):
		x = np.empty(self.dim, dtype='f8')
		i = 0
		for v in self.slvvars:
			l = len(v)
			x[i:i+l] = v
			i += l
		return x
	def place(self, x):
		i = 0
		for v in self.slvvars:
			l = len(v)
			for j in range(l):	v[j] = x[i+j]
			i += l
	def fit(self):
		return sum((c.fit() for c in self.constraints))
	def evaluate(self, x):
		self.place(x)
		return self.fit()
	
	def solve(self, precision=1e-4, method='BFGS', afterset=None):
		nprint(self.slvvars)
		res = minimize(self.evaluate, self.state(), 
				tol=precision, method=method, callback=afterset, 
				options={'eps':precision/2})
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
		
		#print('corrnorms', corrnorms)
		#print('corrections', corrections)
		#for i in range(len(corrections)):
			#if id(params[i]) in fixed:	corrections[i] = vec3(0)
		#for c in corrections:	print('    ',c)
		
		# apply changes
		for param,correction,corrnorm,oldcorr in zip(params, corrections, corrnorms, oldcorrs):
			if id(param) not in fixed:
				l = length(correction)
				if l > maxcorr:		maxcorr = l
				if corrnorm > maxdelta:		maxdelta = corrnorm
				#if l > 0:
					#param += correction * (corrnorm / l)
				# filtre a oscillation
				if oldcorr:
					#print('       ', correction*0.5 + oldcorr*0.5)
					param += (correction*0.5 + oldcorr*0.5)
				else:
					param += correction * 0.8
		oldcorrs = corrections[:]
		
		#print('solve:', it, maxdelta)
		if afterset:	afterset()
		it += 1
		# check that the solver is solving
		if maxiter and it > maxiter:	
			raise SolveError('failed to converge with the allowed iteration count: '+str(maxiter))
		if maxdelta > precision and maxcorr < mincorr:	
			raise SolveError('resolution is blocked (try an other initial state)')

	print('iterations:', it-1)
	return maxdelta
