from random import random
from arrex import typedlist
from pyglm.glm import uvec2
from madcad import vec3, Axis, Circle, brick
from madcad.mesh import Web, web

def test_extremities():
	w = Web(
		[vec3(0)]*5,
		[(0,1), (1,2), (2,3), (4,0), (3,4)],
		)
	assert len(w.extremities_unoriented()) == 0
	assert len(w.extremities_oriented()) == 0
	assert w.isloop()
	
	w = Web(
		[vec3(0)]*5,
		[(0,1), (1,2), (2,3), (4,1), (3,4)],
		)
	assert len(w.extremities_unoriented()) == 2
	assert len(w.extremities_oriented()) == 2
	assert not w.isloop()
	
	w = Web(
		[vec3(0)]*5,
		[(0,1), (1,2), (2,3), (1,4), (3,4)],
		)
	assert len(w.extremities_unoriented()) == 2
	assert len(w.extremities_oriented()) == 3
	assert not w.isloop()
	

def test_islands():
    m = Web(
        [vec3(0), vec3(1,0,0), vec3(1,0,0), vec3(2,1,0), vec3(1,3,0), vec3(1,5,4), vec3(2,5,0)], 
        [(0,1), (1,2), (2,3), (4, 5), (5,6), (6,4)])
    m.check()
    assert len(m.islands()) == 2

def test_frontiers():
    bri = brick(center=vec3(2,0,0), width=vec3(0.5))
    m = bri.frontiers(0,2,3).frontiers({1,2,None})
    assert set(m.indices) == {5, 1, 2}

def test_orientation():
	c = web(Circle(Axis(vec3(0), vec3(0,0,1)), 1))
	c.edges = typedlist((e if random()>0.5 else (e[1],e[0])	for e in c.edges), dtype=uvec2)

	assert not c.isloop()
	
	o = c.own(edges=True).orient()
	o.check()
	assert o.isloop()

	o = c.own(edges=True).orient(vec3(1,0,0))
	o.check()
	assert o.isloop()
	
	o = c.own(edges=True).orient(vec3(1,0,0), vec3(0,1,1))
	o.check()
	assert o.isloop()
