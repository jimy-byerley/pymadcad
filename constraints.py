import primitives
from mathutils import project, normalize, length, vec3
from math import asin, cos, sin, degrees, inf

class Tangent:
	def __init__(self, c1, c2, p):
		arc = None
		if isinstance(c1, (primitives.Segment, primitives.Axis)) and isinstance(c2, primitives.Arc):
			arc,seg = c2,c1
		elif isinstance(c2, (primitives.Segment, primitives.Axis)) and isinstance(c1, primitives.Arc):
			arc,seg = c1,c2
		if arc:
			self.arc, self.seg = arc, seg
			self.p = p
		else:
			raise TypeError('currently only a segment and an arc can be tangent')
		
	def params(self):
		return self.p, self.arc.center
	
	def corrections(self):
		arc, seg = self.arc, self.seg
		AB = seg.b - seg.a
		AC = arc.center - seg.a
		AC_AB = project(AC, normalize(AB))
		v = AC_AB - AC
		vn = length(v)
		corr = AC_AB - AB + (arc.radius()-vn)/vn * v
		return corr, -corr

class Distance:
	def __init__(self, p1, p2, d):
		self.distance = d
		self.p1, self.p2 = p1,p2
	
	def params(self):
		return self.p1,self.p2
		
	def corrections(self):
		v = self.p1 - self.p2
		d = length(v)
		corr = (self.distance - d)/d
		return corr*v, -corr*v

class Angle:
	def __init__(self, s1,s2, angle):	# WARNING: ici on considere que ce sont des segments orientés (trouver une autre representation)
		self.s1,self.s2 = s1,s2
		self.angle = angle
		
	def params(self):
		return self.s1.a, self.s1.b, self.s2.a, self.s2.b
	
	def corrections(self):
		z = cross(self.s1.direction(), self.s2.direction())
		
		# s1 donne le s2 idéal appellé v2
		x = x1 = self.s1.b - self.s1.a
		y = cross(z, x)
		v2 = cos(self.angle)*x + sin(self.angle)*y
		# s2 donne le s1 idéal appellé v1
		x = x2 = self.s2.b - self.s2.a
		y = cross(z, x)
		v1 = cos(self.angle)*x - sin(self.angle)*y
		
		d1 = v1 - x1
		d2 = v2 - x2
		return -d1,d1,-d2,d2

class Parallel(Angle):	# pas tres optimisé pour le moment, mais infaillible
	def __init__(self, s1,s2):
		super().__init__(s1,s2, 0)

class Radius:
	def __init__(self, arc, radius):
		self.arc = arc
		self.consta = Distance(arc.center, arc.a, radius)
		self.constb = Distance(arc.center, arc.b, radius)
	
	def params(self):
		return self.arc.center, self.arc.a, self.arc.b
	
	def corrections(self):
		corr1 = self.consta.corrections()
		corr2 = self.constb.corrections()
		return corr1[0]+corr2[0], corr1[1], corr2[1]

class Projected:
	def __init__(self, a,b, projection):
		self.projection = projection
		self.a,self.b = a,b
	
	def params(self):
		return self.a, self.b
	
	def corrections(self):
		corr = self.projection - project((self.b - self.a), self.projection.direction())
		return -corr, corr

class PointOn:
	def __init__(self, point, curve):
		if not isinstance(curve, Segment):
			raise TypeError('currently, curve can only be a Segment')
		self.curve = curve
		self.point = point
	
	def params(self):
		return self.point, self.curve.a, self.curve.b
	
	def corrections(self):
		indev()
		

def solve(constraints, precision=1e-5, afterset=None, fixed=()):
	corrections = {}
	params = {}
	rate = 0.8
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
			f = rate / len(corr)
			for p,correction in corr:
				corrections[id(p)] += correction * f
		
		for k,correction in corrections.items():
			corr_norm = length(correction)
			if corr_norm > max_delta:	max_delta = corr_norm
			params[k] += correction
		
		print(max_delta)
		if afterset:	afterset()

	return max_delta


if __name__ == '__main__':
	from primitives import Point, Segment, Arc
	from math import radians
	
	print('equilateral triangle')
	A = Point(0, 0, 0)
	B = Point(1, 0, 0)
	C = Point(0, 1, 0)
	AB = Segment(A,B)
	AC = Segment(A,C)
	BC = Segment(B,C)
	
	csts = [
		Distance(A,B, 1),
		Distance(A,C, 1),
		Distance(B,C, 1),
		]
	solve(csts, afterset=lambda: print(A,B,C), fixed={id(A)})

	print('tangent arc')
	A = Point(0, 0, 0)
	B = Point(1, 0, 0)
	C = Point(0, 1, 0)
	AB = Segment(A,B)
	AC = Segment(A,C)
	O = Point(0.8,0.8,0)
	BC = Arc(O, B,C)
	
	csts = [
		Distance(A,B, 1),
		Distance(A,C, 1),
		#Distance(B,C, 1),
		Tangent(AB,BC,B),
		Tangent(AC,BC,C),
		]
	solve(csts, afterset=lambda: print(A,B,C,O), fixed={id(A)})
	
	print('angle')
	A = Point(0, 0, 0)
	B = Point(1, 0, 0)
	C = Point(0, 1, 0)
	D = Point(0.9, 0.9, 0)
	AB = Segment(A,B)
	AC = Segment(A,C)
	CD = Segment(C,D)
	BD = Segment(C,D)
	
	csts = [
		Distance(A,B, 1),
		Distance(A,C, 1),
		Angle(AB,AC, radians(120)),
		Angle(CD,Segment(D,B), radians(90)),
		Parallel(AB,CD),
		]
	solve(csts, afterset=lambda: print(A,B,C,D), fixed={id(A)})
	
