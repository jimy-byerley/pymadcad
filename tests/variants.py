

class variants(object):
	__slots__ = 'iters', 'state', 'stack', 'item'
	
	def __init__(self, **iters):
		self.iters = iters # defintions of the iterables used
		for name,it in iters.items():
			if not hasattr(it, '__getattr__'):
				self.iters[name] = list(it)
		self.state = {}	# item index per iterable (indexed by name)
		self.stack = [] # stack of iterable names
		self.item = None
		
	def __iter__(self):
		return self
		
	def __next__(self):
		# unstack if indexes overflows
		while self.stack:
			name = self.stack[-1]
			inc = self.state[name] + 1
			# unstack if index overflow
			if inc == len(self.iters[name]):
				del self.state[self.stack.pop()]
			# increment if still iterating over the current row
			else:
				self.state[name] = inc
				return self.item
		# in case of an empty stack:
		# starting iterator: item not initialized yet
		if not self.item:
			self.item = self.Item(self)
			return self.item
		# ending iterator
		else:
			raise StopIteration
			
	def __repr__(self):
		return '<variants {}>'.format([self.state[name] for name in self.stack])
		
	class Item(object):
		__slots__ = 'variants'
		
		def __init__(self, variants):
			self.variants = variants
			
		def __getitem__(self, name):
			if not isinstance(name, str):
				raise TypeError('the given key must be str')
			v = self.variants
			# new asked parameter: stack it
			if name not in v.state:
				v.state[name] = 0
				v.stack.append(name)
				# new undefined parameter:  defaults to boolean in the rest of the variants (even unstacking)
				if name not in v.iters:
					v.iters[name] = v._booliter
			# get current value
			return v.iters[name][v.state[name]]
			
		def __iter__(self):
			v = self.variants
			for name in v.stack:
				yield v.iters[v.state[name]]
		
		def items(self):
			v = self.variants
			for name in v.stack:
				yield name, v.iters[name][v.state[name]]
				
		def keys(self):
			return iter(self.variants.stack)
			
		def __repr__(self):
			return '<variant {}>'.format(', '.join('{}={}'.format(name, repr(value))  for name,value in self.items()))
			
					
	_booliter = (False, True)

	
if __name__ == '__main__':
	print(variants(some=range(3), other=(False,True)))
	
	for variant in variants(a=range(4), b=[3,5,1], d=range(12)):
		if variant['c']:  # if not specified in variants, a variable is boolean
			if variant['a'] % 2 == 0:  # only pairs values of a do not use b
				print(variant, 'pair', variant['b']+variant['a'])
			else:
				print(variant, 'odd')
		else:
			print(variant, 'nothing')
		
		# do something that is common to all cases
		# some test on the current combination for instance

