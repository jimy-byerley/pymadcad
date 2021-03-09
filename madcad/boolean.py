# This file is part of pymadcad,  distributed under license LGPL v3

'''	
	Defines boolean operations for triangular meshes
	
	This relies on a new intersection algorithm named syandana. It finds candidates for intersections using a spacial hashing of triangles over a voxel (see madcad.hashing). This is solving the problem of putting triangles in an octree.
	Also to avoid the increasing complexity of the operation with flat planes divided in multiple parallel triangles, the algorithm is implemented with a detection of ngons.
	
	The syandana algorithm achieves intersections of meshes in nearly `O(n)` where usual methods are `O(n**2)`
	
	After intersection, the selection of surface sides to keep or not is done through a propagation.
'''

from copy import copy,deepcopy
from time import time
from math import inf
from .mathutils import *
from . import core
from .mesh import Mesh, Web, edgekey, connef, line_simplification
from . import hashing
from . import triangulation

__all__ = ['intersect', 'boolean', 'intersection', 'union', 'difference']
		


def intersectwith(m1, m2, prec=None):
	''' Cut m1 faces at their intersections with m2. 
		Returning the intersection edges in m1 and associated m2 faces.
		
		m1 faces and tracks are replaced thus the underlying buffers stays untouched.
	
		The algorithm is using ngon intersections and retriangulation, in order to avoid infinite loops and intermediate triangles.
	'''
	if not prec:	prec = m1.precision()
	frontier = []	# cut points for each face from m2
	
	# topology informations for optimization
	points = hashing.PointSet(prec, manage=m1.points)
	prox2 = hashing.PositionMap(hashing.meshcellsize(m2))
	for f2 in range(len(m2.faces)):
		prox2.add(m2.facepoints(f2), f2)
	conn = connef(m1.faces)
	
	mn = Mesh(m1.points, groups=m1.groups)	# resulting mesh
	grp = [-1]*len(m1.faces)	# flat region id
	currentgrp = -1
	for i in range(len(m1.faces)):
		# process the flat surface starting here, if the m1's triangle hits m2
		if grp[i] == -1 and m1.facepoints(i) in prox2:
			
			# get the flat region - aka the n-gon to be processed
			currentgrp += 1
			surf = []
			front = [i]
			normal = m1.facenormal(i)
			if not isfinite(normal):	continue
			track = m1.tracks[i]
			while front:
				fi = front.pop()
				if grp[fi] == -1 and m1.tracks[fi] == track and distance2(m1.facenormal(fi), normal) <= NUMPREC:
					surf.append(fi)
					grp[fi] = currentgrp
					f = m1.faces[fi]
					for edge in ((f[1],f[0]), (f[2],f[1]), (f[0],f[2])):
						if edge in conn:	front.append(conn[edge])
			
			# get region ORIENTED outlines - aka the outline of the n-gon
			outline = set()
			for f1 in surf:
				f = m1.faces[f1]
				for edge in ((f[1],f[0]), (f[2],f[1]), (f[0],f[2])):
					if edge in outline:	outline.remove(edge)
					else:				outline.add((edge[1], edge[0]))
			original = set(outline)
			
			# process all ngon triangles
			# enrich outlines with intersections
			#segts = Web(m1.points, groups=m2.faces)
			segts = {}
			for f1 in surf:
				f = m1.faces[f1]
				for f2 in set( prox2.get(m1.facepoints(f1)) ):
					intersect = core.intersect_triangles(m1.facepoints(f1), m2.facepoints(f2), 8*prec)
					if intersect:
						ia, ib = intersect
						if distance2(ia[2],ib[2]) <= prec**2:	continue
						# insert intersection points
						p1 = points.add(ia[2])
						p2 = points.add(ib[2])
						# associate the intersection edge with the m2's face
						if (p1,p2) in segts:	continue
						segts[(p1,p2)] = f2
						# cut the outline if needed
						if ia[0] == 0:	
							o = f[ia[1]], f[(ia[1]+1)%3]
							if o in original:
								e = find(outline, lambda e: distance_pe(m1.points[p1], (m1.points[e[0]], m1.points[e[1]])) <= prec)
								if p1 not in e:	# NOTE maybe not necessary
									outline.remove(e)
									outline.add((e[0],p1))
									outline.add((p1,e[1]))
						if ib[0] == 0:	
							o = f[ib[1]], f[(ib[1]+1)%3]
							if o in original:
								e = find(outline, lambda e: distance_pe(m1.points[p2], (m1.points[e[0]], m1.points[e[1]])) <= prec)
								if p2 not in e:
									outline.remove(e)
									outline.add((e[0],p2))
									outline.add((p2,e[1]))
			
			# simplify the intersection lines
			segts = Web(m1.points, list(segts.keys()), list(segts.values()), m2.faces)
			simp = line_simplification(segts, prec)
			segts.mergepoints(simp)
			frontier.extend(zip(segts.edges, segts.tracks))
			
			# retriangulate the cutted surface
			segts.edges.extend(segts.edges)
			segts.edges.extend(outline)
			segts.tracks = [0] * len(segts.edges)
			flat = triangulation.triangulation_sweepline(segts, normal, prec)
			# append the triangulated face, in association with the original track
			flat.tracks = [track] * len(flat.faces)
			flat.groups = m1.groups
			mn += flat
	# append non-intersected faces
	for f,t,grp in zip(m1.faces, m1.tracks, grp):
		if grp == -1:
			mn.faces.append(f)
			mn.tracks.append(t)
	
	m1.faces = mn.faces
	m1.tracks = mn.tracks
	return frontier
from .mesh import suites
#def dumpvec(v):
	#return 'dvec3({:.15g},{:.15g},{:.15g})'.format(*v)
#def dumpface(f):
	#return ', '.join([dumpvec(p) for p in f])

# routines to simplify intersections
def max_v(x):
	m = x[0]
	im = 0
	for i in range(1,len(x)):
		if x[i]>m:
			m = x[i]
			im = i
	return m, im

def min_v(x):
	m = x[0]
	im = 0
	for i in range(1,len(x)):
		if x[i]<m:
			m = x[i]
			im = i
	return m, im

def conv_v(t,x):
	if not isinstance(x,t):	return t(x)
	else:					return x

# main intersection routine
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
    # computation in double precision
    dr=dvec3
    do=vec3
    fA = (conv_v(dr,f0[0]), conv_v(dr,f0[1]), conv_v(dr,f0[2]))
    fB = (conv_v(dr,f1[0]), conv_v(dr,f1[1]), conv_v(dr,f1[2]))

    # gets the normal to the fisrt face
    A1A2=fA[1]-fA[0]
    A1A3=fA[2]-fA[0]
    nA1=cross(A1A2, A1A3)
    nA=normalize(nA1)

    # gets the normal to the second face
    B1B2=fB[1]-fB[0]
    B1B3=fB[2]-fB[0]
    nB1=cross(B1B2, B1B3)
    nB=normalize(nB1)

    # gets the direction of the intersection between the plan containing fA and the one containing fB 
    d1=cross(nA, nB)
    ld1=length(d1)
    if ld1 < precision :
        #print("coplanar or parallel faces")
        return None
        
    # normalize d1
    d=d1/ld1

    # projection direction on to d from fA and fB
    tA=cross(nA, d)
    tB=cross(nB, d)

    # project fA summits onto d (in pfA)
    # xA being the coordinates of fA onto d
    pA1=fA[0]-dot(fA[0]-fB[0], nB)/dot(tA,nB)*tA
    xA=dvec3(0., dot(A1A2, d), dot(A1A3, d))
    pfA=(pA1, pA1+xA[1]*d, pA1+xA[2]*d)

    # project fB summits onto d
    xB=dvec3(dot(fB[0]-fA[0], d), dot(fB[1]-fA[0], d), dot(fB[2]-fA[0], d))
    pfB=(pA1+xB[0]*d, pA1+xB[1]*d, pA1+xB[2]*d)

    # project fA and fB summits on transversal direction tA tB
    yA=dvec3(0.)
    yB=dvec3(0.)
    for i in range(0,3):
        yA[i]=dot((fA[i]-pfA[i]), tA)
        yB[i]=dot((fB[i]-pfB[i]), tB)

    # dimensioning for fA and fB
    sA=max(max(abs(xA)), max(abs(yA)))
    sB=max(max(abs(xB)), max(abs(yB)))
    
    # identify signs of yA and yB
    sYA=ivec3(0)
    sYB=ivec3(0)
    for i in range(0,3):
        if abs(yA[i])/sA<precision :
            sYA[i]=0
            yA[i]=0.
        else : 
            sYA[i]=sign(yA[i])
        if abs(yB[i])/sB<precision :
            sYB[i]=0
            yB[i]=0.
        else : 
            sYB[i]=sign(yB[i])

    # check if triangles have no intersections with line D
    if abs(sum(sYA))==3 or abs(sum(sYB))==3 : 
        #print("plans intersects but no edges intersection")
        return None

    # we know that triangles do intersect the line D
    # edges of intersection on A and B with the convention : edge i of face X connects fX[i] and fX[(i+1)%3] 
    eIA=[]
    eIB=[]
    for i in range(0,3):
        if sYA[i]*sYA[(i+1)%3]<0 and abs(sYA[i])+abs(sYA[(i+1)%3])>0 : 
            eIA.append(i)
        elif sYA[i]==0 :
            if abs(sYA[(i+1)%3])>0 : 
                eIA.append(i)
            else :
                eIA.append((i-1)%3)
        if sYB[i]*sYB[(i+1)%3]<0 and abs(sYB[i])+abs(sYB[(i+1)%3])>0 : 
            eIB.append(i)
        elif sYB[i]==0:
            if abs(sYB[(i+1)%3])>0 : 
                eIB.append(i)
            else :
                eIB.append((i-1)%3)
    if len(eIA)==1 :
        eIA.append(eIA[0])
    if len(eIB)==1 :
        eIB.append(eIB[0])
                
    # intersections coordinates onto line D
    xIA=dvec2(0.)
    xIB=dvec2(0.)
    for i in range(0,2):
        xIA[i]=(yA[(eIA[i]+1)%3]*xA[eIA[i]]-yA[eIA[i]]*xA[(eIA[i]+1)%3])/(yA[(eIA[i]+1)%3]-yA[eIA[i]])
        xIB[i]=(yB[(eIB[i]+1)%3]*xB[eIB[i]]-yB[eIB[i]]*xB[(eIB[i]+1)%3])/(yB[(eIB[i]+1)%3]-yB[eIB[i]])

    # intervals of intersections
    xPIA=max_v(xIA)
    xMIA=min_v(xIA)
    xPIB=max_v(xIB)
    xMIB=min_v(xIB)

    # one intersection at the border of the intervals
    if abs(xPIA[0]-xMIB[0])/(0.5*(sB+sA))<precision : 
        # edge of max from A matches min of B
        #print("edges intersection PA-MB")
        return (0, eIA[xPIA[1]], conv_v(do, pA1+xIA[xPIA[1]]*d)), (1, eIB[xMIB[1]], conv_v(do,pA1+xIB[xMIB[1]]*d))
        
    if abs(xPIB[0]-xMIA[0])/(0.5*(sB+sA))<precision : 
        # edge of max from B matches min of A
        #print("edges intersection PB-MA")
        return (0, eIA[xMIA[1]], conv_v(do, pA1+xIA[xMIA[1]]*d)), (1, eIB[xPIB[1]], conv_v(do,pA1+xIB[xPIB[1]]*d))

    # no intersection
    if xPIB[0]-precision<xMIA[0] or xPIA[0]-precision<xMIB[0]:
        #print("plans intersects but no edges intersection")
        return None
        
    # one interval is included into the other one
    if xMIB[0]-precision<xMIA[0] and xPIA[0]-precision<xPIB[0]:
        # edges of A cross face B
        #print("edges of A intersect B")
        return (0, eIA[xMIA[1]], conv_v(do, pA1+xIA[xMIA[1]]*d)), (0, eIA[xPIA[1]], conv_v(do, pA1+xIA[xPIA[1]]*d))
    if xMIA[0]-precision<xMIB[0] and xPIB[0]-precision<xPIA[0]:
        # edges of B cross face A
        #print("edges of B intersect A")
        return (1, eIB[xMIB[1]], conv_v(do, pA1+xIB[xMIB[1]]*d)), (1, eIB[xPIB[1]], conv_v(do, pA1+xIB[xPIB[1]]*d))

    # intervals cross each other
    if xMIB[0]>xMIA[0]-precision and xPIA[0]-precision<xPIB[0]:
        # M edge of B crosses face A and P edge of A crosses face B
        #print("M edge of B intersects A and P edge of A intersects B")
        return (0, eIA[xPIA[1]], conv_v(do, pA1+xIA[xPIA[1]]*d)), (1, eIB[xMIB[1]], conv_v(do, pA1+xIB[xMIB[1]]*d))
    if xMIA[0]>xMIB[0]-precision and xPIB[0]-precision<xPIA[0]:
        # M edge of A crosses face B and P edge of B crosses face A
        #print("M edge of A intersects B and P edge of B intersects A")
        return (0, eIA[xMIA[1]], conv_v(do, pA1+xIA[xMIA[1]]*d)), (1, eIB[xPIB[1]], conv_v(do, pA1+xIB[xPIB[1]]*d))

    print("intersect_triangles: unexpected case : ", fA, fB)
    return None


def booleanwith(m1, m2, side, prec=None):
	''' execute the boolean operation only on m1 '''
	if not prec:	prec = m1.precision()
	start = time()
	frontier = intersectwith(m1, m2, prec)
	print('intersectwith', time()-start)
	start = time()
	
	conn1 = connef(m1.faces)
	used = [False] * len(m1.faces)
	notto = set((edgekey(*e[0]) for e in frontier))
	front = []
	# get front and mark frontier faces as used
	for e,f2 in frontier:
		for edge in (e, (e[1],e[0])):
			if edge in conn1:
				fi = conn1[edge]
				f = m1.faces[fi]
				for i in range(3):
					if f[i] not in edge:
						proj = dot(m1.points[f[i]] - m1.points[f[i-1]], m2.facenormal(f2))  * (-1 if side else 1)
						if proj > prec:
							used[fi] = True
							front.append((f[i], f[i-1]))
							front.append((f[i-2], f[i]))
						break
	if not front:
		if side:	
			m1.faces = []
			m1.tracks = []
		return notto
	
	# display frontier
	#from . import text
	#for edge in notto:
		#p = (m1.points[edge[0]] + m1.points[edge[1]]) /2
		#scn3D.add(text.Text(p, str(edge), 9, (1, 1, 0)))
	#if debug_propagation:
		#from .mesh import Web
		#w = Web([1.01*p for p in m1.points], [e for e,f2 in frontier])
		#w.options['color'] = (1,0.9,0.2)
		#scn3D.add(w)
	
	# propagation
	front = [e for e in front if edgekey(*e) not in notto]
	c = 1
	while front:
		newfront = []
		for edge in front:
			if edge in conn1:
				fi = conn1[edge]
				if not used[fi]:
					used[fi] = c
					f = m1.faces[fi]
					for i in range(3):
						if edgekey(f[i-1],f[i]) not in notto:	
							newfront.append((f[i],f[i-1]))
		c += 1
		front = newfront
	# selection of faces
	#if debug_propagation:
		#from . import text
		#for i,u in enumerate(used):
			#if u:
				#p = m1.facepoints(i)
				#scn3D.add(text.Text((p[0]+p[1]+p[2])/3, str(u), 9, (1,0,1), align=('center', 'center')))
	m1.faces =  [f for u,f in zip(used, m1.faces) if u]
	m1.tracks = [t for u,t in zip(used, m1.tracks) if u]
	''' # simplification of the intersection, only needed by intersectwith_naive
	# simplify the intersection (only at the intersection contains useless points)
	# remove edges from empty triangles	
	edges = m1.edges()
	lines = []
	for e in notto:
		if e in edges:
			lines.append(e)
	m1.mergepoints(line_simplification(Web(m1.points, lines), prec))
	'''
	print('booleanwith', time()-start)
	
	return notto

debug_propagation = False

def intersect(m1: Mesh, m2: Mesh):
	''' cut the faces of m1 and m2 at their intersections '''
	m3 = copy(m1)
	intersectwith(m1, m2)
	intersectwith(m2, m3)

def boolean(m1: Mesh, m2: Mesh, selector: '(bool,bool)', prec=None) -> Mesh:
	''' execute boolean operation on volumes 
	
		selector decides which part of each mesh to keep
	
			- False keep the exterior part (part exclusive to the other mesh)
			- True keep the common part
	'''
	if not prec:	prec = max(m1.precision(), m2.precision())
	
	if selector[0] is not None:
		if selector[1] is not None:
			mc1, mc2 = copy(m1), copy(m2)
			booleanwith(mc1, m2, selector[0], prec)
			booleanwith(mc2, m1, selector[1], prec)
			if selector[0] and not selector[1]:		mc1 = mc1.flip()
			if not selector[0] and selector[1]:		mc2 = mc2.flip()
			res = mc1 + mc2
			res.mergeclose()
			return res
		else:
			mc1 = copy(m1)
			booleanwith(mc1, m2, selector[0], prec)
			return mc1
	elif selector[1] is not None:
		return boolean(m2, m1, (selector[1], selector[0]))

def union(a: Mesh, b: Mesh) -> Mesh:			
	''' return a mesh for the union of the volumes. 
		It is a boolean with selector (False,False) 
	'''
	return boolean(a,b, (False,False))

def intersection(a: Mesh, b: Mesh) -> Mesh:	
	''' return a mesh for the common volume. 
		It is a boolean with selector (True, True) 
	'''
	return boolean(a,b, (True,True))

def difference(a: Mesh, b: Mesh) -> Mesh:	
	''' return a mesh for the volume of a less the common volume with b
		It is a boolean with selector (False, True)
	'''
	return boolean(a,b, (False,True))

