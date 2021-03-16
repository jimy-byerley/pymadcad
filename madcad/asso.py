from bisect import bisect_left, bisect_right

class Asso(object):
	''' Assoassociative container.
		This is a sort dict that stores as many values as we want for each key. The return value for a key is then always an iterable, even when the value set to the key is unique and is not an iterator.
		
		For now, keys must be comparable, a bisection is used to access keys, so the complexity is O(ln(n))
		
		
		Asso(iterable)				the iterable must yields (key,value) pairs
		Asso(keys=[], values=[])	build form existing lists, assuming they are already sorted
	'''
	__slots__ = '_keys', '_values'
	
	def __init__(self, iter=None, keys=None, values=None):
		self._keys = keys or []
		self._values = values or []
		assert len(self._keys) == len(self._values)
		
		if iter:
			# collect and sort
			for key, value in iter:
				self._keys.append(key)
				self._values.append(value)
			sort = sorted(range(len(self._keys)), key=lambda i: self._keys[i])
			self._keys = [self._keys[i] 	for i in sort]
			self._values = [self._values[i] 	for i in sort]
		
	def __getitem__(self, key):
		''' return an iterable of the objects associated to the given key '''
		i = bisect_left(self._keys, key)
		if i == len(self._keys):	
			return
		start = self._keys[i]
		while i < len(self._keys) and self._keys[i] == start:
			yield self._values[i]
			i += 1
			
	def add(self, key, value):
		''' associate a new value to this key '''
		i = bisect_right(self._keys, key)
		self._keys.insert(i, key)
		self._values.insert(i, value)
		
	def update(self, other):
		''' append all key,value associations to this Asso '''
		pool = self + other
		self._keys = pool._keys
		self._values = pool._values
		
	def __add__(self, other):
		if isinstance(other, dict):	
			other = dict.items()
		if not isinstance(other, Asso):
			other = Asso(other)
		# interclass
		knew = []
		vnew = []
		si, oi = 0, 0
		while si < len(self._keys) and oi < len(other._keys):
			if self._keys[si] < other._keys[oi]:
				knew.append(self._keys[si])
				vnew.append(self._values[si])
				si += 1
			else:
				knew.append(other._keys[si])
				vnew.append(other._values[si])
				oi += 1
		for i in range(si, len(self._keys)):
			knew.append(self._keys[i])
			vnew.append(self._values[i])
		for i in range(oi, len(other._keys)):
			knew.append(other._keys[i])
			vnew.append(other._values[i])
		return Asso(keys=knew, values=vnew)
		
		
	def __delitem__(self, key):
		''' delete all associations with this key '''
		istart = bisect_left(self._keys, key)
		start = self._values[istart]
		while i < len(self._keys) and self._keys[i] == start:
			i += 1
		if i > istart:
			del self._keys[istart:i]
			del self._values[istart:i]
	
	def clear(self):
		''' empty the container '''
		self._keys.clear()
		self._values.clear()

	def items(self):
		''' iterator of (key, value) pairs '''
		return zip(self._keys, self._values)

	def keys(self):
		''' iterator of the keys '''
		return iter(self._keys)
		
	def values(self):
		''' iterator of the values '''
		return iter(self._values)

	def __contains__(self, key):
		''' return True if key is associated to something '''
		return self._keys[bisect_left(self._keys, key)] == key
		
	def __repr__(self):
		return 'Asso([{}])'.format(', '.join(
					'({}, {})'.format(repr(k), repr(v))	
						for k,v in zip(self._keys, self._values)
					))


if __name__ == '__main__':
	from nprint import nprint

	m = Asso([
			(1,'truc'), 
			(2,'machin'),
			(12, 'chose'),
			(1, 'bidule'),
			])
	nprint('created', m)
	
	m.update([
		(1,'machin'),
		(-1,'gna'),
		])
	nprint('updated', m)
	
	m.add(1, 2)
	nprint('inserted', m)
	
	assert list(m[1]) == ['truc', 'bidule', 2]
	assert list(m[12]) == ['chose']


