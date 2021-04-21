

class Asso(object):
	''' Assoassociative container.
		This is a sort dict that stores as many values as we want for each key. The return value for a key is then always an iterable of the associated values, even when the value set to the key is unique and is not an iterator.
		
		Asso(iterable)				the iterable must yields (key,value) pairs
		Asso(keys=[], values=[])	build form existing lists, assuming they are already sorted
	'''
	__slots__ = '_table'
	
	def __init__(self, iter=None):
		if isinstance(iter, Asso):
			self._table = dict(iter.table)
		else:
			self._table = {}
			if iter:
				self.update(iter)
		
	def __getitem__(self, key):
		''' return an iterable of the objects associated to the given key '''
		i = 0
		while True:
			k = (i,key)
			if k not in self._table:
				break
			yield self._table[k]
			i += 1
			
	def add(self, key, value):
		''' associate a new value to this key '''
		i = 0
		while True:
			k = (i,key)
			if k not in self._table:
				self._table[k] = value
				return
			i += 1
	
	def remove(self, key, value):
		l = self.connexity(key)-1
		i = l
		while i>=0:
			k = (i,key)
			if self._table[k] == value:
				last = (l,key)
				self._table[k] = self._table[last]
				del self._table[last]
				return
			i -= 1
		raise KeyError('key not in table')
		
	def discard(self, key, value):
		''' remove all (key,value) pair of that table '''
		l = self.connexity(key)-1
		i = l
		while i>=0:
			k = (i,key)
			if self._table[k] == value:
				last = (l,key)
				self._table[k] = self._table[last]
				del self._table[last]
				l -= 1
			i -= 1
	
	def update(self, other):
		''' append all key,value associations to this Asso '''
		if isinstance(other, dict):
			other = other.items()
		for k,v in other:
			self.add(k,v)
		
	def __add__(self, other):
		new = Asso()
		new._table = dict(self._table)
		new.update(other)
		return new
		
		
	def __delitem__(self, key):
		''' delete all associations with this key '''
		i = 0
		while True:
			k = (i,k)
			if k not in self._table:
				break
			del self._table[k]
			i += 1
	
	def clear(self):
		''' empty the container '''
		self._table.clear()

	def items(self):
		''' iterator of (key, value) pairs '''
		for k,v in self._table.items():
			yield k[1],v

	def keys(self):
		''' iterator of the keys '''
		for k,v in self._table:
			yield k[1]
		
	def values(self):
		''' iterator of the values '''
		return self._table.values()
		
	def connexity(self, key):
		''' return the number of values associated to the given key '''
		i = 0
		while True:
			k = (i,key)
			if k not in self._table:
				break
			i += 1
		return i
		

	def __contains__(self, key):
		''' return True if key is associated to something '''
		return (0,key) in self._table
		
	def __repr__(self):
		return 'Asso([{}])'.format(', '.join(
					'({}, {})'.format(repr(k), repr(v))	
						for k,v in self.items()
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
	
	assert set(m[1]) == {'truc', 'bidule', 2, 'machin'}
	assert set(m[12]) == {'chose'}


