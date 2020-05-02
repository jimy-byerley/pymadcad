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
		assert m >= 0
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

cdef cvec3 glm2c(v):
	cdef cvec3 r
	cdef size_t i
	assert isinstance(v, glm.dvec3)
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


"""
cdef struct cvec3 :
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

cdef int sign_d(double d) :
	if (d > 0) :
		return 1
	if (d < 0) :
		return -1
	return 0

cdef cvec3 minus_dv3(cvec3 v1, cvec3 v2) :
	cdef cvec3 r
	r.x[0]=v1.x[0]-v2.x[0]
	r.x[1]=v1.x[1]-v2.x[1]
	r.x[2]=v1.x[2]-v2.x[2]
	return r


cdef cvec3 plus_dv3(cvec3 v1, cvec3 v2) :
	cdef cvec3 r
	r.x[0]=v1.x[0]+v2.x[0]
	r.x[1]=v1.x[1]+v2.x[1]
	r.x[2]=v1.x[2]+v2.x[2]
	return r

cdef cvec3 cross_dv3(cvec3 v1, cvec3 v2) :
	cdef cvec3 r
	r.x[0]=v1.x[1]*v2.x[2]-v1.x[2]*v2.x[1]
	r.x[1]=v1.x[2]*v2.x[0]-v1.x[0]*v2.x[2]
	r.x[2]=v1.x[0]*v2.x[1]-v1.x[1]*v2.x[0]
	return r

cdef double dot_dv3(cvec3 v1, cvec3 v2) :
	return v1.x[0]*v2.x[0]+v1.x[1]*v2.x[1]+v1.x[2]*v2.x[2]

from libc.math cimport sqrt as csqrt
from libc.math cimport abs as cabs


cdef double length_dv3(cvec3 v1) :
	return csqrt(v1.x[0]*v1.x[0]+v1.x[1]*v1.x[1]+v1.x[2]*v1.x[2])

cdef cvec3 normalize_dv3(cvec3 v1) :
	cdef cvec3 r
	cdef double n = length_dv3(v1)
	r.x[0]=v1.x[0]/n
	r.x[1]=v1.x[1]/n
	r.x[2]=v1.x[2]/n
	return r
	
cdef cvec3 mult_dv3(cvec3 v1, double d) :
	cdef cvec3 r
	r.x[0]=v1.x[0]*d
	r.x[1]=v1.x[1]*d
	r.x[2]=v1.x[2]*d
	return r
	
from libc.math cimport fabs as cfabs    
cdef cvec3 abs_dv3(cvec3 v) : 
	cdef cvec3 r
	r.x[0]=cfabs(v.x[0])
	r.x[1]=cfabs(v.x[1])
	r.x[2]=cfabs(v.x[2])
	return r
	
cdef cvec3 sum_mult_dv3(cvec3 v1, cvec3 v2, double d) : 
	cdef cvec3 r
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
def intersect_triangles(f0, f1, double precision):
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
	cdef cvec3 fA[3]
	cdef cvec3 fB[3]
	cdef size_t i,j
	
	for i in range(3):
		for j in range(3):
			varr(&fA[i])[j] = f0[i][j]
			varr(&fB[i])[j] = f1[i][j]

	# gets the normal to the fisrt face
	cdef cvec3 A1A2 = vsub(fA[1], fA[0])
	cdef cvec3 A1A3 = vsub(fA[2], fA[0])
	cdef cvec3 nA = normalize(cross(A1A2, A1A3))
	
	# gets the normal to the second face
	cdef cvec3 nB1 = normalize(cross(sub(fB[1], fB[0]), sub(fB[2], fB[0])))
	
	# gets the direction of the intersection between the plan containing fA and the one containing fB 
	cdef cvec3 d1 = cross(nA, nB)
	cdef double ld1 = length(d1)
	if ld1 < precision :
		#print("coplanar or parallel faces")
		return None
		
	# normalize d1
	cdef cvec3 d = vmul(d1, 1/ld1)

	# projection direction on to d from fA and fB
	cdef cvec3 tA = cross(nA, d)
	cdef cvec3 tB = cross(nB, d)

	# project fA summits onto d (in pfA)
	# xA being the coordinates of fA onto d
	# pA1=fA[0]-dot(fA[0]-fB[0], nB)/dot(tA,nB)*tA
	cdef cvec3 pA1 = vsub(fA[0], vmul(tA, dot(vsub(fA[0],fB[0]), nB)/dot(tA,nB)))
	
	#xA=dvec3(0., dot(A1A2, d), dot(A1A3, d))
	cdef cvec3 xA
	xA.x[0] = 0
	xA.x[1] = dot(A1A2, d)
	xA.x[2] = dot(A1A3, d)
	#pfA=(pA1, pA1+xA[1]*d, pA1+xA[2]*d)
	cdef cvec3 pfA[3]
	pfA[0] = pA1
	for i in range(1,3):
		pfA[i] = sum_mult_dv3(pA1, d, xA.x[i])
	
	# project fB summits onto d
	#xB=dvec3(dot(fB[0]-fA[0], d), dot(fB[1]-fA[0], d), dot(fB[2]-fA[0], d))
	cdef cvec3 fAfB[3]
	cdef cvec3 xB
	for i in range(3):
		fAfB[i] = sub(fB[i], fA[0])
		xB.x[i] = dot(fAfB[i], d)
	#pfB=(pA1+xB[0]*d, pA1+xB[1]*d, pA1+xB[2]*d)
	cdef cvec3 pfB[3]
	for i in range(3):
		pfB[i] = sum_mult_dv3(pA1, d, varr(&xB)[i])
	
	# project fA and fB summits on transversal direction tA tB
	cdef cvec3 yA
	cdef cvec3 yB
	for i in range(3):
		varr(&yA)[i] = dot(sub(fA[i], pfA[i]), tA)
		varr(&yB)[i] = dot(sub(fB[i], pfB[i]), tB)

	# --------- NOTE finir la conversion -----------
	# dimensioning for fA and fB
	#sA=max(max_v(abs(xA))[0], max_v(abs(yA))[0])
	cdef double sA
	cdef size_t iA1, iA2
	sA=max(max_dv(abs_dv3(xA).x, 3, &iA1), max_dv(abs_dv3(yA).x, 3, &iA2))
	
	#sB=max(max_v(abs(xB))[0], max_v(abs(yB))[0])
	cdef double sB
	cdef size_t iB1, iB2
	sB=max(max_dv(abs_dv3(xB).x, 3, &iB1), max_dv(abs_dv3(yB).x, 3, &iB2))
	
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
			sYB[i]=sign(yB.x[i])

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
		return (0, eIA[xPIA1], c2glm(sum_mult_dv3(pA1, d, xIA[xPIA1]).x)), \
			(1, eIB[xMIB1], c2glm(sum_mult_dv3(pA1, d, xIB[xMIB1]).x))
	if cfabs(xPIB0-xMIA0)/(0.5*(sB+sA))<precision_d : 
		# edge of max from B matches min of A
		#print("edges intersection PB-MA")
		return (0, eIA[xMIA1], c2glm(sum_mult_dv3(pA1, d, xIA[xMIA1]).x)), \
			(1, eIB[xPIB1], c2glm(sum_mult_dv3(pA1, d, xIB[xPIB1]).x))
		
	# no intersection
	if xPIB0-precision_d<xMIA0 or xPIA0-precision_d<xMIB0:
		#print("plans intersects but no edges intersection")
		return None
	
	# one interval is included into the other one
	if xMIB0-precision_d<xMIA0 and xPIA0-precision_d<xPIB0:
		# edges of A cross face B
		#print("edges of A intersect B")
		return (0, eIA[xMIA1], c2glm(sum_mult_dv3(pA1, d, xIA[xMIA1]).x)), \
			(0, eIA[xPIA1], c2glm(sum_mult_dv3(pA1, d, xIA[xPIA1]).x))
	if xMIA0-precision_d<xMIB0 and xPIB0-precision_d<xPIA0:
		# edges of B cross face A
		#print("edges of B intersect A")
		return (1, eIB[xMIB1], c2glm(sum_mult_dv3(pA1, d, xIB[xMIB1]).x)), \
			(1, eIB[xPIB1], c2glm(sum_mult_dv3(pA1, d, xIB[xPIB1]).x))

	# intervals cross each other
	if xMIB0>xMIA0-precision_d and xPIA0-precision_d<xPIB0:
		# M edge of B crosses face A and P edge of A crosses face B
		#print("M edge of B intersects A and P edge of A intersects B")
		return (0, eIA[xPIA1], c2glm(sum_mult_dv3(pA1, d, xIA[xPIA1]).x)), \
			(1, eIB[xMIB1], c2glm(sum_mult_dv3(pA1, d, xIB[xMIB1]).x))
	if xMIA0>xMIB0-precision_d and xPIB0-precision_d<xPIA0:
		# M edge of A crosses face B and P edge of B crosses face A
		#print("M edge of A intersects B and P edge of B intersects A")
		return (0, eIA[xMIA1], c2glm(sum_mult_dv3(pA1, d, xIA[xMIA1]).x)), \
			(1, eIB[xPIB1], c2glm(sum_mult_dv3(pA1, d, xIB[xPIB1]).x))

	print("intersect_triangles: unexpected case : ", fA, fB)
	return None

"""
