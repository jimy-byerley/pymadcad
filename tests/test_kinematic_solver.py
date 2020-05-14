from madcad.mathutils import vec3
from madcad.constraints import solve
from madcad.kinematic import *

s0 = Solid()

print('\ntest simple pivot')
s1 = Solid(pose=(vec3(0), 2*vec3(1,1,0)))
csts = [Pivot(s0,s1, (vec3(0,0,0), vec3(0,1,0)))]
solve(csts,
		fixed=[s0.position, s0.orientation], 
		afterset=lambda x: print(x))

print('\ntest simple plane')
s1 = Solid(pose=(vec3(0), 2*vec3(1,1,0)))
csts = [Plane(s0,s1, (vec3(0,0,0), vec3(0,1,0)))]
solve(csts,
		fixed=[s0.position, s0.orientation], 
		afterset=lambda x: print(x))

print('\ntest plane and pivot')
s1 = Solid()
s2 = Solid(pose=(vec3(2,0,1.5), 2*vec3(1,1,0)))
csts = [
	Plane(s0,s1, (vec3(0), vec3(0,0,1))),  
	Pivot(s1,s2, (vec3(0,0,1), vec3(0,1,0)), (vec3(0,0,-1), vec3(0,1,0))),
	]
solve(csts,
		fixed=[s0.position, s0.orientation, s2.position], 
		afterset=lambda x: print(x),
		precision=1e-4)

'''
print('\n test pivots and shared point')
s0 = Solid()
s1 = Solid()
s2 = Solid()
A = vec3(0.1, 0.2, 0.3)
csts = [
	Pivot(s0,s1, (vec3(1,0,0),vec3(0,1,0))),
	Pivot(s0,s2, (vec3(-1,0,0),vec3(0,1,0))),
	InSolid(s1, [A], [vec3(0,0,1)]),
	InSolid(s2, [A], [vec3(0,0,1)]),
	]
solve(csts, 
		fixed={id(s0.position), id(s0.orientation)}, 
		afterset=lambda: print(s1.orientation, s2.orientation))

l12 = Pivot((0,x))
l23 = Pivot((A,x))
csts = [
	Solid(
'''
