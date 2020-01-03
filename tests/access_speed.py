from time import time
import array
import numpy.core as np

num = 1000
size = 100

table = [0]*size
start = time()
for _ in range(num):
	for i in range(len(table)):
		table[i] += 1.2
print('python list: ', time()-start)



table = array.array('d', [0]*size)
start = time()
for _ in range(num):
	for i in range(len(table)):
		table[i] += 1.2
print('array.array: ', time()-start)


table = np.array([0]*size, dtype='f8')
start = time()
for _ in range(num):
	for i in range(len(table)):
		table[i] += 1.2
print('numpy.array: ', time()-start)
