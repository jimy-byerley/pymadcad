# This file is part of pymadcad,  distributed under license LGPL v3

# cython: language_level=3, cdivision=True

from libc.math cimport fabs, ceil, floor, sqrt, INFINITY
from libc.math cimport fmod
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
	
	if cell <= 0:	raise ValueError('cell must be strictly positive')
	
	space = [glm2c(spaceo[0]), glm2c(spaceo[1])]
	rasterization = []
	
	# permutation of coordinates to get the direction the closer to Z
	v = vsub(space[1], space[0])
	n = vabs(v)
	if vmax(n) < NUMPREC*max(norminf(space[0]), norminf(space[1])):	return rasterization
	if   n.y >= n.x and n.y >= n.z:		order,reorder = [2,0,1],[1,2,0]
	elif n.x >= n.y and n.x >= n.z:		order,reorder = [1,2,0],[2,0,1]
	else:								order,reorder = [0,1,2],[0,1,2]
	for i in range(2):
		temp = space[i]
		for j in range(3):
			varr(&space[i])[j] = varr(&temp)[order[j]]
	
	# prepare variables
	cell2 = cell/2
	assert v.z
	dy = v.y/v.z
	dx = v.x/v.z
	o = space[0]
	
	# z selection
	zmin,zmax = space[0].z, space[1].z
	if v.z < 0:		zmin,zmax = zmax,zmin
	zmin -= pmod(zmin,cell)
	for i in range(max(1,<size_t>ceil((zmax-zmin)/cell))):
		z = zmin + cell*i + cell2
		
		# y selection
		ymin,ymax = o.y + dx*(z-cell2-o.z),  o.y + dx*(z+cell2-o.z)
		if dy < 0:	ymin,ymax = ymax,ymin
		ymin -= pmod(ymin,cell)
		for j in range(max(1,<size_t>ceil((ymax-ymin)/cell))):
			y = ymin + j*cell + cell2
			
			# x selection
			xmin,xmax = o.x + dx*(z-cell2-o.z),  o.x + dx*(z+cell2-o.z)
			if dx < 0:	xmin,xmax = xmax,xmin	
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
	
	if cell <= 0:	raise ValueError('cell must be strictly positive')
	
	space = [glm2c(spaceo[0]), glm2c(spaceo[1]), glm2c(spaceo[2])]
	rasterization = []
	
	# permutation of coordinates to get the normal the closer to Z
	n = vabs(cross(vsub(space[1],space[0]), vsub(space[2],space[0])))
	if vmax(n) < NUMPREC*max(norminf(space[0]), norminf(space[1]), norminf(space[2])):	return rasterization
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
			zmin -= pmod(zmin,cell)
			if zmax < zmin:	continue
			for k in range(max(1,<size_t>ceil((zmax-zmin)/cell))):
				z = zmin + cell*k + cell2
				
				# remove box from corners that goes out of the area
				if pmin.x<x and pmin.y<y and pmin.z<z and x<pmax.x and y<pmax.y and z<pmax.z:
					pk = [key(x,cell), key(y,cell), key(z,cell)]
					rasterization.append(( pk[reorder[0]], pk[reorder[1]], pk[reorder[2]] ))
	return rasterization


cdef int dsign(double v):
	if v > 0:	return 1
	elif v < 0:	return -1
	else:		return 0
	
def intersect_triangles(f0, f1, precision):
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
	if ld1 < prec :
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
		if abs(varr(&yA)[i]) < prec:
			sYA[i] = 0
			varr(&yA)[i] = 0
		else:
			sYA[i] = dsign(varr(&yA)[i])
		if abs(varr(&yB)[i]) < prec:
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
	cdef int eIA[2]
	cdef int eIB[2]
	cdef size_t neIA=0
	cdef size_t neIB=0
	for i in range(3):
		if sYA[i]*sYA[(i+1)%3]<=0 and abs(sYA[i])+abs(sYA[(i+1)%3])>0 : 
			eIA[neIA] = i
			neIA += 1
		#elif sYA[i]==0:
			#if abs(sYA[(i+1)%3])>0 : 
				#eIA[neIA] = i
				#neIA += 1
			#else :
				#eIA[neIA] = (i-1)%3
				#neIA += 1
		if sYB[i]*sYB[(i+1)%3]<=0 and abs(sYB[i])+abs(sYB[(i+1)%3])>0 : 
			eIB[neIB] = i
			neIB += 1
		#elif sYB[i]==0:
			#if abs(sYB[(i+1)%3])>0 : 
				#eIB[neIB] = i
				#neIB += 1
			#else :
				#eIB[neIB] = (i-1)%3
				#neIB += 1
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
	if abs(xIA[piA]-xIB[miB]) < prec:
		# edge of max from A matches min of B
		return (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA]))),  (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB])))
		
	if abs(xIB[piB]-xIA[miA]) < prec:
		# edge of max from B matches min of A
		return (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA]))),  (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
	
	# no intersection - intervals doesn't cross
	if xIB[piB]-prec < xIA[miA] or xIA[piA]-prec < xIB[miB]:
		#print("plans intersects but no edges intersection (2)")
		return None
		
	# one interval is included in the other one
	if xIB[miB]-prec < xIA[miA] and xIA[piA]-prec < xIB[piB]:
		# edges of A cross face B
		return (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA]))),  (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA])))
	if xIA[miA]-prec < xIB[miB] and xIB[piB]-prec < xIA[piA]:
		# edges of A cross face B
		return (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB]))),  (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
	
	# intervals cross each other
	if xIB[miB] > xIA[miA]-prec and xIA[piA]-prec < xIB[piB]:
		# M edge of B crosses face A and P edge of A crosses face B
		return (0, eIA[piA], c2glm(vaffine(pA1, d, xIA[piA]))), (1, eIB[miB], c2glm(vaffine(pA1, d, xIB[miB])))
	if xIA[miA] > xIB[miB]-prec and xIB[piB]-prec < xIA[piA]:
		# M edge of A crosses face B and P edge of B crosses face A
		return (0, eIA[miA], c2glm(vaffine(pA1, d, xIA[miA]))), (1, eIB[piB], c2glm(vaffine(pA1, d, xIB[piB])))
	
	print("intersect_triangles: unexpected case : ", fA, fB)
	return None
"""
import glm 

ctypedef struct dv3 :
	double x[3]

# routines toF simplify intersections
cdef double max_dv(double *x, int n, int *im):
	cdef double m=x[0]
	cdef int i
	im[0]=0
	
	for i in range(1,n):
		if x[i]>m:
			m=x[i]
			im[0]=i
	return m

cdef double min_dv(double *x, int n, int*im):
	cdef double m=x[0]
	cdef int i
	im[0]=0
	for i in range(1,n):
		if x[i]<m:
			m=x[i]
			im[0]=i
	return m

cdef double max_d(double d1, double d2):
	if (d1<d2) :
		return d2
	return d1

cdef double min_d(double d1, double d2):
	if (d1>d2) :
		return d2
	return d1

cdef int sign_d(double d) :
	if (d > 0) :
		return 1
	if (d < 0) :
		return -1
	return 0


def conv_v(r,x):
	if isinstance(r, glm.fvec3): 
		return glm.fvec3(x)
	if isinstance(r, glm.dvec3): 
		return glm.dvec3(x)
	return x


cdef dv3 minus_dv3(dv3 v1, dv3 v2) :
	cdef dv3 r
	r.x[0]=v1.x[0]-v2.x[0]
	r.x[1]=v1.x[1]-v2.x[1]
	r.x[2]=v1.x[2]-v2.x[2]
	return r


cdef dv3 plus_dv3(dv3 v1, dv3 v2) :
	cdef dv3 r
	r.x[0]=v1.x[0]+v2.x[0]
	r.x[1]=v1.x[1]+v2.x[1]
	r.x[2]=v1.x[2]+v2.x[2]
	return r

cdef dv3 cross_dv3(dv3 v1, dv3 v2) :
	cdef dv3 r
	r.x[0]=v1.x[1]*v2.x[2]-v1.x[2]*v2.x[1]
	r.x[1]=v1.x[2]*v2.x[0]-v1.x[0]*v2.x[2]
	r.x[2]=v1.x[0]*v2.x[1]-v1.x[1]*v2.x[0]
	return r

cdef double dot_dv3(dv3 v1, dv3 v2) :
	return v1.x[0]*v2.x[0]+v1.x[1]*v2.x[1]+v1.x[2]*v2.x[2]

from libc.math cimport sqrt as csqrt
from libc.math cimport abs as cabs


cdef double length_dv3(dv3 v1) :
	return csqrt(v1.x[0]*v1.x[0]+v1.x[1]*v1.x[1]+v1.x[2]*v1.x[2])

cdef dv3 normalize_dv3(dv3 v1) :
	cdef dv3 r
	cdef double n = length_dv3(v1)
	r.x[0]=v1.x[0]/n
	r.x[1]=v1.x[1]/n
	r.x[2]=v1.x[2]/n
	return r
	
cdef dv3 mult_dv3(dv3 v1, double d) :
	cdef dv3 r
	r.x[0]=v1.x[0]*d
	r.x[1]=v1.x[1]*d
	r.x[2]=v1.x[2]*d
	return r
	
from libc.math cimport fabs as cfabs    
cdef dv3 abs_dv3(dv3 v) : 
	cdef dv3 r
	r.x[0]=cfabs(v.x[0])
	r.x[1]=cfabs(v.x[1])
	r.x[2]=cfabs(v.x[2])
	return r
	
cdef dv3 sum_mult_dv3(dv3 v1, dv3 v2, double d) : 
	cdef dv3 r
	r.x[0]=v1.x[0]+v2.x[0]*d
	r.x[1]=v1.x[1]+v2.x[1]*d
	r.x[2]=v1.x[2]+v2.x[2]*d
	return r

cdef int sum_iv(int *v, int n) :
	cdef int i
	cdef int r=0
	for i in range(n) : 
		r=r+v[i]
	return r

# main intersection routine
def intersect_triangles2(f0, f1, precision=1e-7):
	''' Intersects 2 triangles and outputs intersections vertices

		f0 = first face (tuple of 3 vertices given in clock wise orientation. vertices are glm.vec3 or glm.dvec3)
		f1 = second face

		output = None if no intersection
				2 intersection vertices given as : 
				((fi, ej, xj), (fk, em, xm)) where
					fi, fk = face id
					ej, em = edges id on faces
					xj, xm = intersection point (same precision as input vertices) 
					
		restrictions : vertices on faces must be spatially different. identical vertices on triangles are not managed
	'''
	# computation in double precision
	cdef dv3 fA[3]
	cdef dv3 fB[3]
	cdef int i
	for i in range(3) :
		fA[i].x[0] = f0[i][0]
		fA[i].x[1] = f0[i][1]
		fA[i].x[2] = f0[i][2]
		fB[i].x[0] = f1[i][0]
		fB[i].x[1] = f1[i][1]
		fB[i].x[2] = f1[i][2]
	cdef double precision_d = precision


	# gets the normal to the fisrt face
	#A1A2=fA[1]-fA[0]
	cdef dv3 A1A2=minus_dv3(fA[1], fA[0])
	#A1A3=fA[2]-fA[0]
	cdef dv3 A1A3=minus_dv3(fA[2], fA[0])
	#nA1=cross(A1A2, A1A3)
	cdef dv3 nA1=cross_dv3(A1A2, A1A3)
	#nA=normalize(nA1)
	cdef dv3 nA=normalize_dv3(nA1)
	
	# gets the normal to the second face
	#B1B2=fB[1]-fB[0]
	cdef dv3 B1B2=minus_dv3(fB[1], fB[0])
	#B1B3=fB[2]-fB[0]
	cdef dv3 B1B3=minus_dv3(fB[2], fB[0])
	#nB1=cross(B1B2, B1B3)
	cdef dv3 nB1=cross_dv3(B1B2, B1B3)
	#nB=normalize(nB1)
	cdef dv3 nB=normalize_dv3(nB1)
	
	# gets the direction of the intersection between the plan containing fA and the one containing fB 
	#d1=cross(nA, nB)
	cdef dv3 d1=cross_dv3(nA, nB)
	#ld1=length(d1)
	cdef double ld1=length_dv3(d1)
	if ld1 < precision_d :
		#print("coplanar or parallel faces")
		return None
		
	# normalize d1
	#d=d1/ld1
	cdef dv3 d=mult_dv3(d1, 1.0/ld1)

	# projection direction on to d from fA and fB
	#tA=cross(nA, d)
	cdef dv3 tA=cross_dv3(nA, d)
	#tB=cross(nB, d)
	cdef dv3 tB=cross_dv3(nB, d)

	# project fA summits onto d (in pfA)
	# xA being the coordinates of fA onto d
	# pA1=fA[0]-dot(fA[0]-fB[0], nB)/dot(tA,nB)*tA
	cdef dv3 pA1=minus_dv3(fA[0], mult_dv3(tA, dot_dv3(minus_dv3(fA[0],fB[0]), nB)/dot_dv3(tA,nB)))
	
	#xA=dvec3(0., dot(A1A2, d), dot(A1A3, d))
	cdef dv3 xA
	xA.x[0]=0.
	xA.x[1]=dot_dv3(A1A2, d)
	xA.x[2]=dot_dv3(A1A3, d)
	#pfA=(pA1, pA1+xA[1]*d, pA1+xA[2]*d)
	cdef dv3 pfA[3]
	pfA[0]=pA1
	for i in range(1,3):
		pfA[i]=sum_mult_dv3(pA1, d, xA.x[i])
	
	# project fB summits onto d
	#xB=dvec3(dot(fB[0]-fA[0], d), dot(fB[1]-fA[0], d), dot(fB[2]-fA[0], d))
	cdef dv3 fAfB[3]
	cdef dv3 xB
	for i in range(3):
		fAfB[i]=minus_dv3(fB[i], fA[0])
		xB.x[i]=dot_dv3(fAfB[i], d)
	#pfB=(pA1+xB[0]*d, pA1+xB[1]*d, pA1+xB[2]*d)
	cdef dv3 pfB[3]
	for i in range(3):
		pfB[i]=sum_mult_dv3(pA1, d, xB.x[i])
	
	# project fA and fB summits on transversal direction tA tB
	cdef dv3 yA
	cdef dv3 yB
	for i in range(3):
		#yA[i]=dot((fA[i]-pfA[i]), tA)
		yA.x[i]=dot_dv3(minus_dv3(fA[i], pfA[i]), tA)
		#yB[i]=dot((fB[i]-pfB[i]), tB)
		yB.x[i]=dot_dv3(minus_dv3(fB[i], pfB[i]), tB)

	# dimensioning for fA and fB
	#sA=max(max_v(abs(xA))[0], max_v(abs(yA))[0])
	cdef double sA
	cdef int iA1, iA2
	sA=max_d(max_dv(abs_dv3(xA).x, 3, &iA1), max_dv(abs_dv3(yA).x, 3, &iA2))
	
	#sB=max(max_v(abs(xB))[0], max_v(abs(yB))[0])
	cdef double sB
	cdef int iB1, iB2
	sB=max_d(max_dv(abs_dv3(xB).x, 3, &iB1), max_dv(abs_dv3(yB).x, 3, &iB2))
	
	# identify signs of yA and yB
	cdef int sYA[3]
	cdef int sYB[3]
	for i in range(3):
		if cfabs(yA.x[i])/sA<precision_d :
			sYA[i]=0
			yA.x[i]=0.
		else : 
			sYA[i]=sign_d(yA.x[i])
		if cfabs(yB.x[i])/sB<precision_d :
			sYB[i]=0
			yB.x[i]=0.
		else : 
			sYB[i]=sign_d(yB.x[i])

	# check if triangles have no intersections with line D
	if cabs(sum_iv(sYA, 3))==3 or cabs(sum_iv(sYB,3))==3 : 
		#print("plans intersects but no edges intersection")
		return None

	# we know that triangles do intersect the line D
	# edges of intersection on A and B with the convention : edge i of face X connects fX[i] and fX[(i+1)%3] 
	cdef int eIA[2]
	cdef int eIB[2]
	cdef int neIA=0
	cdef int neIB=0
	for i in range(0,3):
		if sYA[i]*sYA[(i+1)%3]<0 and cabs(sYA[i])+cabs(sYA[(i+1)%3])>0 : 
			eIA[neIA]=i
			neIA=neIA+1
		elif sYA[i]==0 :
			if cabs(sYA[(i+1)%3])>0 : 
				eIA[neIA]=i
				neIA=neIA+1
			else :
				eIA[neIA]=(i-1)%3
				neIA=neIA+1
		if sYB[i]*sYB[(i+1)%3]<0 and cabs(sYB[i])+cabs(sYB[(i+1)%3])>0 : 
			eIB[neIB]=i
			neIB=neIB+1
		elif sYB[i]==0 :
			if cabs(sYB[(i+1)%3])>0 : 
				eIB[neIB]=i
				neIB=neIB+1
			else :
				eIB[neIB]=(i-1)%3
				neIB=neIB+1
	if neIA==1 :
		eIA[1]=eIA[0]
	if neIB==1 :
		eIB[1]=eIB[0]

	# intersections coordinates onto line D
	cdef double xIA[2]
	cdef double xIB[2]
	for i in range(2):
		xIA[i]=(yA.x[(eIA[i]+1)%3]*xA.x[eIA[i]]-yA.x[eIA[i]]*xA.x[(eIA[i]+1)%3])/(yA.x[(eIA[i]+1)%3]-yA.x[eIA[i]])
		xIB[i]=(yB.x[(eIB[i]+1)%3]*xB.x[eIB[i]]-yB.x[eIB[i]]*xB.x[(eIB[i]+1)%3])/(yB.x[(eIB[i]+1)%3]-yB.x[eIB[i]])

	# intervals of intersections
	cdef double xPIA0, xMUA0, xPIB0, xMIB0
	cdef int xPIA1, xMIA1, xPIB1, xMIB1
	xPIA0=max_dv(xIA, 2, &xPIA1)
	xMIA0=min_dv(xIA, 2, &xMIA1)
	xPIB0=max_dv(xIB, 2, &xPIB1)
	xMIB0=min_dv(xIB, 2, &xMIB1)

	# one intersection at the border of the intervals
	if cfabs(xPIA0-xMIB0)/(0.5*(sB+sA))<precision_d : 
		# edge of max from A matches min of B
		#print("edges intersection PA-MB")
		return (0, eIA[xPIA1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIA[xPIA1]).x)), \
			(1, eIB[xMIB1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIB[xMIB1]).x))
	if cfabs(xPIB0-xMIA0)/(0.5*(sB+sA))<precision_d : 
		# edge of max from B matches min of A
		#print("edges intersection PB-MA")
		return (0, eIA[xMIA1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIA[xMIA1]).x)), \
			(1, eIB[xPIB1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIB[xPIB1]).x))
		
	# no intersection
	if xPIB0-precision_d<xMIA0 or xPIA0-precision_d<xMIB0:
		#print("plans intersects but no edges intersection")
		return None
	
	# one interval is included into the other one
	if xMIB0-precision_d<xMIA0 and xPIA0-precision_d<xPIB0:
		# edges of A cross face B
		#print("edges of A intersect B")
		return (0, eIA[xMIA1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIA[xMIA1]).x)), \
			(0, eIA[xPIA1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIA[xPIA1]).x))
	if xMIA0-precision_d<xMIB0 and xPIB0-precision_d<xPIA0:
		# edges of B cross face A
		#print("edges of B intersect A")
		return (1, eIB[xMIB1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIB[xMIB1]).x)), \
			(1, eIB[xPIB1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIB[xPIB1]).x))

	# intervals cross each other
	if xMIB0>xMIA0-precision_d and xPIA0-precision_d<xPIB0:
		# M edge of B crosses face A and P edge of A crosses face B
		#print("M edge of B intersects A and P edge of A intersects B")
		return (0, eIA[xPIA1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIA[xPIA1]).x)), \
			(1, eIB[xMIB1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIB[xMIB1]).x))
	if xMIA0>xMIB0-precision_d and xPIB0-precision_d<xPIA0:
		# M edge of A crosses face B and P edge of B crosses face A
		#print("M edge of A intersects B and P edge of B intersects A")
		return (0, eIA[xMIA1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIA[xMIA1]).x)), \
			(1, eIB[xPIB1], conv_v(f0[0], sum_mult_dv3(pA1, d, xIB[xPIB1]).x))

	print("intersect_triangles: unexpected case : ", fA, fB)
	return None
"""
