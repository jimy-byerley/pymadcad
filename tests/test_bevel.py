# test intersections
from madcad import vec3, saddle, tube, ArcThrough, Web, web, filet, chamfer, show
from madcad.bevel import edgecut
from madcad.nprint import nprint
from copy import deepcopy

mesh = saddle(
		Web(
			[vec3(-2,1.5,0),vec3(-1,1,0),vec3(0,0,0),vec3(1,1,0),vec3(1.5,2,0)], 
			[(0,1), (1,2), (2,3), (3,4)],
			[0,1,2,3]),
		#web(vec3(0,1,-1),vec3(0,0,0),vec3(0,1,1)),
		#web(ArcThrough(vec3(0,1,-1),vec3(0,1.5,0),vec3(0,1,1))),
		web(
			ArcThrough(vec3(0,1,-1),vec3(0,1.3,-0.5),vec3(0,1,0)), 
			ArcThrough(vec3(0,1,0),vec3(0,0.7,0.5),vec3(0,1,1))),
		)
box = mesh.box()
edge = mesh.frontiers((1,2)) + mesh.frontiers((5,6))

results = {}
for i, operation in enumerate([edgecut, chamfer, filet]):
	for j, dim in enumerate(['width', 'depth', 'radius', 'distance']):
		print('testing  {} {}  ... '.format(operation.__name__, dim), end='')
		try:
			operated = deepcopy(mesh)
			operated.check()
			operation(operated, edge, **{dim: 0.6})
			operated.check()
			# assert operated.issurface()
		except Exception as err:
			print('failed', err)
			raise
		else:
			results[(operation, dim)] = operated.transform(box.width * 1.2 * vec3(i,0,j))
			print('ok')

show(results, display_wire=True)

