# This file is part of pymadcad,  distributed under license LGPL v3

# cython: language_level=3, cdivision=True

from libc.math cimport fabs, ceil, floor, sqrt, fmod, INFINITY
cimport cython
import glm

cdef:
	DEF NUMPREC = 1e-13
	
	double pmod(double l, double r):
		''' proceed to a python-like modulo on floats 
			(C module doesn't give a good reminder for negative values) 
		'''
		m = fmod(l,r)
		if m < 0:	m += r
		return m
	
	struct cvec3:
		double x
		double y
		double z
	
	double * varr(cvec3 *v):
		return <double*> v

	double dot(cvec3 a, cvec3 b):
		return a.x*b.x + a.y*b.y + a.z*b.z
	cvec3 cross(cvec3 a, cvec3 b):
		return cvec3(a.y*b.z - a.z*b.y,   a.z*b.x - a.x*b.z,   a.x*b.y - a.y*b.x)
	cvec3 vabs(cvec3 v):
		return cvec3(fabs(v.x), fabs(v.y), fabs(v.z))
	double amax(double *v, size_t l):
		cdef double m
		m = v[0]
		for i in range(1,l):
			if v[i] > m:	m = v[i]
		return m
	double amin(double *v, size_t l):
		cdef double m
		m = v[0]
		for i in range(1,l):
			if v[i] < m:	m = v[i]
		return m
	double aimax(double *v, size_t l):
		j = 0
		for i in range(1,l):
			if v[i] > v[j]:	j = i
		return j
	double aimin(double *v, size_t l):
		j = 0
		for i in range(1,l):
			if v[i] < v[j]:	j = i
		return j
	double vmax(cvec3 v):
		return amax(<double*>&v,3)
	double vmin(cvec3 v):
		return amin(<double*>&v,3)
	cvec3 vadd(cvec3 a, cvec3 b):
		return cvec3(a.x+b.x, a.y+b.y, a.z+b.z)
	cvec3 vsub(cvec3 a, cvec3 b):
		return cvec3(a.x-b.x, a.y-b.y, a.z-b.z)
	cvec3 vmul(cvec3 v, double r):
		return cvec3(r*v.x, r*v.y, r*v.z)
	double length(cvec3 v):
		return sqrt(dot(v,v))
	double norminf(cvec3 v):
		cdef cvec3 a = cvec3(abs(v.x), abs(v.y), abs(v.z))
		return amax(<double*>&a,3)

	cvec3 normalize(cvec3 v):
		return vmul(v, 1/length(v))
		
	cvec3 vaffine(cvec3 b, cvec3 a, double x):
		return cvec3(b.x + a.x*x,
					b.y + a.y*x,
					b.z + a.z*x)

cdef cvec3 glm2c(v):
	cdef cvec3 r
	cdef size_t i
	assert isinstance(v, (glm.dvec3, glm.fvec3, glm.ivec3))
	cdef double[:] a = v
	for i in range(3):	varr(&r)[i] = a[i]
	return r

cdef object c2glm(cvec3 v):
	return glm.dvec3(v.x, v.y, v.z)
	
cdef int dsign(double v):
	if v > 0:	return 1
	elif v < 0:	return -1
	else:		return 0
	

	
cdef long key(double f, double cell):
	''' hashing key for a float '''
	return <long> floor(f/cell)
	
def rasterize_segment(spaceo, double cell):
	''' return a list of hashing keys for an edge '''
	cdef cvec3 n, v, o
	cdef size_t order[3]
	cdef size_t reorder[3]
	cdef size_t i,j,k
	cdef double x,y,z, xmin,ymin, zmin,zmax
	cdef long pk[3]
	cdef cvec3 space[2]
	cdef double prec
	
	if cell <= 0:	raise ValueError('cell must be strictly positive')
	
	space = [glm2c(spaceo[0]), glm2c(spaceo[1])]
	rasterization = []
	prec = NUMPREC * max(norminf(space[0]), norminf(space[1]))
	
	# permutation of coordinates to get the direction the closest to Z
	n = vabs(vsub(space[1], space[0]))
	if vmax(n) < prec:	return rasterization
	if   n.y >= n.x and n.y >= n.z:		order,reorder = [2,0,1],[1,2,0]
	elif n.x >= n.y and n.x >= n.z:		order,reorder = [1,2,0],[2,0,1]
	else:								order,reorder = [0,1,2],[0,1,2]
	for i in range(2):
		temp = space[i]
		for j in range(3):
			varr(&space[i])[j] = varr(&temp)[order[j]]
	
	# prepare variables
	v = vsub(space[1], space[0])
	cell2 = cell/2
	assert v.z, order
	dy = v.y/v.z
	dx = v.x/v.z
	o = space[0]
	
	# z selection
	zmin,zmax = space[0].z, space[1].z
	if v.z < 0:		zmin,zmax = zmax,zmin
	zmin -= prec
	zmax += prec
	zmin -= pmod(zmin,cell)
	for i in range(max(1,<size_t>ceil((zmax-zmin)/cell))):
		z = zmin + cell*i + cell2
		
		# y selection
		ymin,ymax = o.y + dx*(z-cell2-o.z),  o.y + dx*(z+cell2-o.z)
		if dy < 0:	ymin,ymax = ymax,ymin
		ymin -= prec
		ymax += prec
		ymin -= pmod(ymin,cell)
		for j in range(max(1,<size_t>ceil((ymax-ymin)/cell))):
			y = ymin + j*cell + cell2
			
			# x selection
			xmin,xmax = o.x + dx*(z-cell2-o.z),  o.x + dx*(z+cell2-o.z)
			if dx < 0:	xmin,xmax = xmax,xmin	
			xmin -= prec
			xmax += prec
			xmin -= pmod(xmin,cell)
			for k in range(max(1,<size_t>ceil((xmax-xmin)/cell))):
				x = xmin + k*cell + cell2
				
				pk = [key(x,cell), key(y,cell), key(z,cell)]
				rasterization.append(( pk[reorder[0]], pk[reorder[1]], pk[reorder[2]] ))
	return rasterization


def rasterize_triangle(spaceo, double cell):
	''' return a list of hashing keys for a triangle '''
	cdef size_t i,j,k
	cdef size_t order[3]
	cdef size_t reorder[3]
	cdef cvec3 v[3]
	cdef double candz[4]
	cdef double candy[6]
	cdef size_t candylen
	cdef long pk[3]
	cdef cvec3 pmin, pmax
	cdef double xmin,xmax, ymin,ymax, zmin,zmax
	cdef cvec3 space[3]
	cdef double prec
	
	if cell <= 0:	raise ValueError('cell must be strictly positive')
	
	space = [glm2c(spaceo[0]), glm2c(spaceo[1]), glm2c(spaceo[2])]
	rasterization = []
	prec = NUMPREC*max(norminf(space[0]), norminf(space[1]), norminf(space[2]))
	
	# permutation of coordinates to get the normal the closer to Z
	n = vabs(cross(vsub(space[1],space[0]), vsub(space[2],space[0])))
	if vmax(n) < prec:	return rasterization
	if   n.y >= n.x and n.y >= n.z:		order,reorder = [2,0,1],[1,2,0]
	elif n.x >= n.y and n.x >= n.z:		order,reorder = [1,2,0],[2,0,1]
	else:								order,reorder = [0,1,2],[0,1,2]
	for i in range(3):
		temp = space[i]
		for j in range(3):
			varr(&space[i])[j] = varr(&temp)[order[j]]
	
	# prepare variables
	# WARNING: due to C differences with modulo (%) we can't use the negative indices for arrays
	v = [vsub(space[0],space[1]), vsub(space[1],space[2]), vsub(space[2],space[0])]
	n = cross(v[0],v[1])
	assert n.z
	dx = -n.x/n.z
	dy = -n.y/n.z
	o = space[0]
	cell2 = cell/2
	pmin = cvec3(
			min(space[0].x, space[1].x, space[2].x),
			min(space[0].y, space[1].y, space[2].y),
			min(space[0].z, space[1].z, space[2].z),
			)
	pmax = cvec3(
			max(space[0].x, space[1].x, space[2].x),
			max(space[0].y, space[1].y, space[2].y),
			max(space[0].z, space[1].z, space[2].z),
			)
	xmin,xmax = pmin.x,pmax.x
	for i in range(3):	varr(&pmin)[i] -= pmod(varr(&pmin)[i], cell)
	for i in range(3):	varr(&pmax)[i] += cell - pmod(varr(&pmax)[i], cell)
	
	# x selection
	xmin -= prec
	xmax += prec
	xmin -= pmod(xmin,cell)
	for i in range(max(1,<size_t>ceil((xmax-xmin)/cell))):
		x = xmin + cell*i + cell2
	
		# y selection
		candylen = 0
		for i in range(3):
			# NOTE: cet interval ajoute parfois des cases inutiles apres les sommets
			if (space[(i+1)%3].x-x+cell2)*(space[i].x-x-cell2) <= 0 or (space[(i+1)%3].x-x-cell2)*(space[i].x-x+cell2) <= 0:
				d = v[i].y / (v[i].x if v[i].x else INFINITY)
				candy[candylen]   = ( space[i].y + d * (x-cell2-space[i].x) )
				candy[candylen+1] = ( space[i].y + d * (x+cell2-space[i].x) )
				candylen += 2
		ymin,ymax = max(pmin.y,amin(candy,candylen)), min(pmax.y,amax(candy,candylen))
		ymin -= prec
		ymax += prec
		ymin -= pmod(ymin,cell)
		if ymax < ymin:	continue
		for j in range(max(1,<size_t>ceil((ymax-ymin)/cell))):
			y = ymin + cell*j + cell2
		
			# z selection
			candz = [
				o.z + dx*(x-cell2-o.x) + dy*(y-cell2-o.y),
				o.z + dx*(x+cell2-o.x) + dy*(y-cell2-o.y),
				o.z + dx*(x-cell2-o.x) + dy*(y+cell2-o.y),
				o.z + dx*(x+cell2-o.x) + dy*(y+cell2-o.y),
				]
			zmin,zmax = max(pmin.z,amin(candz,4)), min(pmax.z,amax(candz,4))
			zmin -= prec
			zmax += prec
			zmin -= pmod(zmin,cell)
			if zmax < zmin:	continue
			for k in range(max(1,<size_t>ceil((zmax-zmin)/cell))):
				z = zmin + cell*k + cell2
				
				# remove box from corners that goes out of the area
				if pmin.x<x and pmin.y<y and pmin.z<z and x<pmax.x and y<pmax.y and z<pmax.z:
					pk = [key(x,cell), key(y,cell), key(z,cell)]
					rasterization.append(( pk[reorder[0]], pk[reorder[1]], pk[reorder[2]] ))
	return rasterization


def intersect_triangles(f0, f1, precision):
	''' Intersects 2 triangles and outputs intersections vertices

		f0 = first face (tuple of 3 vertices given in clock wise orientation. vertices are glm.vec3 or glm.dvec3)
		f1 = second face

		output = None if no intersection
					2 intersection vertices given as : 
					((fi, ej, xj), (fk, em, xm)) where
						fi, fj = face id
						ej, em = edge id on face
						xj, xm = intersection point (same precision as input vertices) 
						
		restrictions : vertices on faces must be spatially different. identical vertices on triangles are not managed
	'''
	cdef cvec3 A1A2, A1A3, B1B2, B1B3, nA, nB, d, xA, xB, yA, yB
	cdef int i

	cdef cvec3[3] fA = [glm2c(f0[0]), glm2c(f0[1]), glm2c(f0[2])]
	cdef cvec3[3] fB = [glm2c(f1[0]), glm2c(f1[1]), glm2c(f1[2])]
	cdef double prec = precision
	
	# get the normal to the first face
	A1A2 = vsub(fA[1],fA[0])
	A1A3 = vsub(fA[2],fA[0])
	nA = normalize(cross(A1A2, A1A3))
	
	# get the normal to the second face
	B1B2 = vsub(fB[1],fB[0])
	B1B3 = vsub(fB[2],fB[0])
	nB = normalize(cross(B1B2, B1B3))
	
	# gets the direction of the intersection between the plan containing fA and the one containing fB 
	d1 = cross(nA, nB)
	ld1 = length(d1)
	if ld1 <= prec :
		#print("coplanar or parallel faces")
		return None
	d = vmul(d1, 1/ld1)
	
	# projection direction on to d from fA and fB
	tA = cross(nA, d)
	tB = cross(nB, d)
	
	# project fA summits onto d (in pfA)
	# xA being the coordinates of fA onto d
	pA1 = vsub(fA[0],  vmul(tA, dot(vsub(fA[0],fB[0]), nB) / dot(tA,nB)) )
	xA = cvec3(0, dot(A1A2,d), dot(A1A3,d))
	cdef cvec3[3] pfA = [pA1, vaffine(pA1, d, xA.y), vaffine(pA1, d, xA.z)]
	
	# project fB summits onto d
	xB = cvec3(dot(vsub(fB[0],fA[0]), d), dot(vsub(fB[1],fA[0]), d), dot(vsub(fB[2],fA[0]), d))
	cdef cvec3[3] pfB = [vaffine(pA1, d, xB.x), vaffine(pA1, d, xB.y), vaffine(pA1, d, xB.z)]
	
	# project fA and fB summits on transversal direction tA and tB
	for i in range(3):
		varr(&yA)[i] = dot(vsub(fA[i], pfA[i]), tA)
		varr(&yB)[i] = dot(vsub(fB[i], pfB[i]), tB)
	
	# identify signs of yA and yB
	cdef int[3] sYA
	cdef int[3] sYB
	for i in range(3):
		if abs(varr(&yA)[i]) <= prec:
			sYA[i] = 0
			varr(&yA)[i] = 0
		else:
			sYA[i] = dsign(varr(&yA)[i])
		if abs(varr(&yB)[i]) <= prec:
			sYB[i] = 0
			varr(&yB)[i] = 0
		else:
			sYB[i] = dsign(varr(&yB)[i])
	
	# check if triangles have no intersections with line D
	if abs(sYA[0]+sYA[1]+sYA[2]) == 3 or abs(sYB[0]+sYB[1]+sYB[2]) == 3:
		#print("plans intersects but no edges intersection (1)")
		return None

	# we know that triangles do intersect the line D
	# edges of intersection on A and B with the convention : edge i of face X connects fX[i] and fX[(i+1)%3] 
	cdef int eIA[3]
	cdef int eIB[3]
	cdef size_t neIA=0
	cdef size_t neIB=0
	
	cdef int j, k
	# prioritize on edges really getting through the face (not stopping on)
	for j in range(3):
		if sYA[j]*sYA[(j+1)%3] < 0:
			break		
	# look for edges intersecting starting from the eventual through one
	for i in range(j,j+3):
		if sYA[i%3]*sYA[(i+1)%3] <= 0 and abs(sYA[i%3])+abs(sYA[(i+1)%3]) > 0 : 
			eIA[neIA] = i%3
			neIA += 1
		if sYB[i%3]*sYB[(i+1)%3] <= 0 and abs(sYB[i%3])+abs(sYB[(i+1)%3]) > 0 : 
			eIB[neIB] = i%3
			neIB += 1
	if neIA==1:		eIA[1] = eIA[0]
	if neIB==1:		eIB[1] = eIB[0]

	# intersections coordinates onto line D
	cdef double xIA[2]
	cdef double xIB[2]
	for i in range(2):
		xIA[i] = (varr(&yA)[(eIA[i]+1)%3] * varr(&xA)[eIA[i]] - varr(&yA)[eIA[i]] * varr(&xA)[(eIA[i]+1)%3]) / (varr(&yA)[(eIA[i]+1)%3] - varr(&yA)[eIA[i]])
		xIB[i] = (varr(&yB)[(eIB[i]+1)%3] * varr(&xB)[eIB[i]] - varr(&yB)[eIB[i]] * varr(&xB)[(eIB[i]+1)%3]) / (varr(&yB)[(eIB[i]+1)%3] - varr(&yB)[eIB[i]])
		
	# intervals of intersections
	piA, miA = (0, 1)	if xIA[0] > xIA[1] else   (1, 0)
	piB, miB = (0, 1)	if xIB[0] > xIB[1] else   (1, 0)
	
    # one intersection at the border of the intervals
	if abs(xIA[piA]-xIB[miB]) <= prec:
		# edge of max from A matches min of B
		return (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA]))),  (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB])))
		
	if abs(xIB[piB]-xIA[miA]) <= prec:
		# edge of max from B matches min of A
		return (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA]))),  (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
	
	# no intersection - intervals doesn't cross
	if xIB[piB]-prec < xIA[miA] or xIA[piA]-prec < xIB[miB]:
		#print("plans intersects but no edges intersection (2)")
		return None
		
	# one interval is included in the other one
	if xIB[miB]-prec <= xIA[miA] and xIA[piA]-prec <= xIB[piB]:
		# edges of A cross face B
		return (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA]))),  (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA])))
	if xIA[miA]-prec <= xIB[miB] and xIB[piB]-prec <= xIA[piA]:
		# edges of A cross face B
		#return (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB]))),  (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
		
		# give priority to face index 0 when equivalent regarding the precision
		if abs(xIA[miA]-xIB[miB]) <= prec:	mr = (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA])))
		else:								mr = (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB])))
		if abs(xIB[piB]-xIA[piA]) <= prec:	pr = (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA])))
		else:								pr = (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
		return mr, pr
	
	# intervals cross each other
	if xIB[miB] > xIA[miA]-prec and xIA[piA]-prec < xIB[piB]:
		# M edge of B crosses face A and P edge of A crosses face B
		return (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA]))), (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB])))
	if xIA[miA] > xIB[miB]-prec and xIB[piB]-prec < xIA[piA]:
		# M edge of A crosses face B and P edge of B crosses face A
		return (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA]))), (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
	
	raise Exception("unexpected case: {} {}".format(repr(fA), repr(fB)))


