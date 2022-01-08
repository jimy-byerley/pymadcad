from .mathutils import *
from .mesh import Mesh, connef, edgekey
from .constraints import SolveError
from .nprint import nprint

import numpy as np
from scipy.optimize import minimize, least_squares

__all__ = [	'segmentation',
			'guessjoint', 'guesssurface',
			]


def segmentation(mesh, tolerance=5, sharp=0.2) -> Mesh:
	''' surface segmentation based on curvature.
		This functions splits faces into groups based on curvature proximity.
		Ideally each resulting group is a set of faces with the same curvature. In practice such is not possible due to the mesh resolution introducing bias in curvature estimation. Thus a `tolerance` is used to decide what is a sufficiently close curvature to belong to the same group.
		In order to avoid too sharp edges to be considered a very low resolution but smooth surface, `sharp` is the value of the limit edge angle that can be considered in smooth surface.
		
		This function is particularly usefull when importing geometries from a format that doesn't manage groups (such as STL).
		
		Parameters:
		
			tolerance (float):	maximum difference factor between curvatures of a same group  (1 means +100% curvature is allowed)
			sharp (float):	angle above which an angle is always sharp (radians)
			
		NOTE:
			
			This functions is not magic and despite the default parameters are usually fine for mechanical parts, you may get badly grouped faces in some cases, hence you will need to tune the parameters. 
			
			The success of this functions is highly dependent of the quality of the input mesh triangulation.
			
		Example:
		
			>>> part = read('myfile.stl')
			>>> part.mergeclose()
			>>> part = segmentation(part)
	'''
	# pick informations from the mesh
	pts = mesh.points
	conn = connef(mesh.faces)
	facenormals = mesh.facenormals()
	prec = 1 / mesh.maxnum()  # curvature precision, curvature is rad/m, so 1rad/max dist seems to be a fairly low curvature
	
	# this is what this functions is working to create: new groups and tracks to assign it to faces
	tracks = [-1] * len(mesh.faces)
	groups = []
	
	# evaluate curvature around every edge
	edgecurve = {}
	facecurve = [0.] * len(mesh.faces)
	weight = [0.] * len(mesh.faces)
	for e, fa in conn.items():
		if edgekey(*e) in edgecurve:	continue
		fb = conn.get((e[1],e[0]))
		if not fb:	continue
		
		curve = dot(pts[e[1]] - pts[e[0]], cross(facenormals[fa], facenormals[fb]))
		edgecurve[edgekey(*e)] = angle = anglebt(facenormals[fa], facenormals[fb])
		
		# report average curvature to neighboring faces
		if sharp > angle:
			w = max(0, 0.5-angle)  # empirical weighting
			facecurve[fa] += curve * w  # multiply angle at edge by   (triangle height) ~= surface/(edge length)
			facecurve[fb] += curve * w
			weight[fa] += w
			weight[fb] += w
	
	# divide contributions to face curvatures
	for i,f in enumerate(mesh.faces):
		area = 0.5 * length(cross(pts[f[1]]-pts[f[0]], pts[f[2]]-pts[f[0]])) * weight[i]
		if area:	facecurve[i] /= area
		else:		facecurve[i] = 0
		#debug.append(text.Text(
							#(pts[f[0]] + pts[f[1]] + pts[f[2]]) /3,
							#'{:.2g}'.format(facecurve[i]),
							#size=8,
							#align=('left', 'center'),
							#))
		
	# groupping gives priority to the less curved faces, so they can extend to the ambiguous limits if there is
	sortedfaces = sorted(range(len(mesh.faces)), key=lambda i:  abs(facecurve[i]))
	index = 0
	
	while True:
		# pick a non assigned face
		for index in range(index, len(sortedfaces)):
			if tracks[sortedfaces[index]] == -1:	break
		if index >= len(sortedfaces):	break
		i = sortedfaces[index]
		f = mesh.faces[i]
		
		# start propagation for this face
		tracks[i] = len(groups)
		estimate = facecurve[i]   # average curvature of the group
		sample = 1   # samples used in the average curvature of the group
		front = [(f[k-1], f[k-2])  for k in range(3)]   # propagation frontline
		reached = {i}  # faces reached by propagation
		
		# for triangles on the frontier of a curved surface, the curvature may appear to be null (or close), in this case look for a paired triangle
		if abs(facecurve[i]) < prec:
			best = None
			fence = inf
			for i,e in enumerate(front):
				j = conn.get(e)
				if j is None or tracks[j] != -1 or j in reached:	continue
				if edgecurve[edgekey(*e)] >= sharp:	continue
				score = abs(facecurve[j]-facecurve[i])
				if score < fence:
					best, fence = j, score
			if best is not None:
				estimate = facecurve[best]
		
		# propagation
		while front:
			e = front.pop()
			j = conn.get(e)
			# check if already processed
			if j is None or tracks[j] != -1 or j in reached:	continue
			reached.add(j)
			# check criterions:
			# split groups at maximum curvature
			if edgecurve[edgekey(*e)] >= sharp:	continue
			# split groups when curvature is too far from average curvature
			if abs(facecurve[j]-estimate) / (abs(estimate)+prec) > tolerance:	continue
			
			# improve estimation of average curvature
			tracks[j] = len(groups)
			estimate = (estimate*sample + facecurve[j]) / (sample+1)
			sample += 1
			
			# propagate
			f = mesh.faces[j]
			front.extend((f[k-1], f[k-2])  for k in range(3))
		
		groups.append(estimate)
		index += 1
			
	return Mesh(mesh.points, mesh.faces, tracks, groups)
	
	
	
# ---------   guess-stuff ----------

def guessjoint(solid1, solid2, surface1, surface2, guess=quat(), precision=1e-5) -> 'joint':
	''' Create a kinematic joint based on the surfaces given. The function will use `guesssurface` to find the surface kind and then deduce the appropriate joint to make the surfaces coincide.
	
		Parameters:
		
			solid1, solid2:  	 solids the joint applies on
			surface1, surface2:  the surface to retreive the joint definition from
			guess(quat, mat3):   orientation tip to orient properly the joint axis, if there is
			precision(float):    precision for `guesssurface`
	'''
	from .joints import Planar, Gliding, Punctiform, Ball
	
	joints = {
		('plane', 'plane'): Planar,
		#('cylinder', 'plane'): Rolling,
		('cylinder', 'cylinder'): Gliding,
		#('cylinder', 'sphere'): Anular,
		('plane', 'sphere'): Punctiform,
		('sphere', 'sphere'): Ball,
		}
	t1, p1 = guesssurface(surface1, precision)
	t2, p2 = guesssurface(surface2, precision)
	joint_type = joints.get(tuple(sorted([t1, t2])))
	if not joint_type:
		raise ValueError('not junction registered for {}-{}'.format(t1, t2))
		
	if joint_type in (Gliding, Planar):
		if dot(guess*p1[1], p2[1]) < 0:
			p1 = p1[0], -p1[1]
	return joint_type(solid1, solid2, p1, p2)

from operator import itemgetter
	
def guesssurface(surface, precision=1e-5, attempt=False):	
	''' guess the surface kind, returns its parameters.
	
		Return a tuple `('surface_type', parameters)`
		
		Parameters:
		
			surface(Mesh):      mesh from which to find the surface kind
			precision(float):	typical distance error from mesh to ideal surface in the surface regression
			attempt(bool):      if True, will not raise an error if the acheived error is too big
	'''
	if not isinstance(surface, Mesh):
		return surface
	
	center = surface.barycenter()
	scores = []
	
	solverargs = dict(
				ftol = precision, 
				max_nfev = 300, 
				method = 'lm')
	
	# plane regression
	def evaluate(x):
		# retreive surface parameters
		origin, normal = vec3(x[:3]), normalize(vec3(x[3:]))
		# allocate residuals
		residual = np.zeros((len(surface.faces)+2, 4), dtype='f8')
		# compute residuals
		residual[-1,0] = (length(vec3(x[3:])) - 1)**2
		residual[-2,0] = length2(noproject(center-origin, normal))
		for i,f in enumerate(surface.faces):
			fp = surface.facepoints(f)
			p = (fp[0]+fp[1]+fp[2]) / 3
			residual[i,0] = dot(p-origin, normal)**2  # distance to the face center
			for j,p in enumerate(fp):
				residual[i,j+1] = dot(p-origin, normal)**2  # distance to each point
		return residual.ravel()
	
	res = least_squares(evaluate, [*center, 1,1,1], **solverargs)
	scores.append((
			np.mean(res.fun),
			'plane',
			(vec3(res.x[:3]), vec3(res.x[3:])),
			))
	#print('plane', res.nfev, np.mean(res.fun), '\t', list(res.x))
	
	# sphere regression
	def evaluate(x):
		# retreive surface parameters
		origin, radius = vec3(x[:3]), float(x[3])
		# allocate residuals
		residual = np.zeros((len(surface.faces), 4), dtype='f8')
		# compute residuals
		for i,f in enumerate(surface.faces):
			fp = surface.facepoints(f)
			p = (fp[0]+fp[1]+fp[2]) / 3
			residual[i,0] = (distance(p, origin) - radius) **2
			for j,p in enumerate(fp):
				residual[i,j+1] = (distance(p, origin) - radius) **2
		return residual.ravel()
	
	res = least_squares(evaluate, [*center, 1], **solverargs)
	scores.append((
			np.mean(res.fun),
			'sphere',
			vec3(res.x[:3]),
			))
	#print('sphere', res.nfev, np.mean(res.fun), '\t', list(res.x))
		
	# cylinder regression
	def evaluate(x):
		# retreive surface parameters
		origin, direction, radius = vec3(x[:3]), vec3(x[3:6]), float(x[6])
		# allocate residuals
		residual = np.zeros((len(surface.faces)+2, 4), dtype='f8')
		# compute residuals
		residual[-1,0] = (length(vec3(x[3:6])) - 1) **2
		residual[-2,0] = length2(project(center-origin, direction))
		for i,f in enumerate(surface.faces):
			fp = surface.facepoints(f)
			p = (fp[0]+fp[1]+fp[2]) / 3
			residual[i,0] = (length(noproject(p-origin, direction)) - radius) **2
			for j,p in enumerate(fp):
				residual[i,j+1] = (length(noproject(p-origin, direction)) - radius) **2
		return residual.ravel()
	# estimate a first axis to avoid the solver to fall into a local extremum
	# sum contrbutions of curvature at edges
	direction = vec3(0)
	conn = connef(surface.faces)
	for e in conn:
		if (e[1], e[0]) in conn and e[0] < e[1]:
			edge = cross(surface.facenormal(conn[e]), surface.facenormal(conn[(e[1],e[0])]))
			direction += edge * (sign(dot(edge, direction)) or 1)
	# if the initial direction is too weak, then we are sure it's not a cylinder
	if length2(direction):
		if -min(direction) > max(direction):	direction = -direction
		# solve
		res = least_squares(evaluate, [*center, *normalize(direction), 1], **solverargs)
		scores.append((
				np.mean(res.fun),
				'cylinder',
				(vec3(res.x[:3]), vec3(res.x[3:])),
				))
		#print('cylinder', res.nfev, np.mean(res.fun), '\t', list(res.x))
		
	# pick best regression of all
	best = min(scores, key=itemgetter(0))
	if attempt or best[0] > precision:
		raise SolveError('unable to find a suitable surface type')
	
	#print('---', best[1])
	return best[1], best[2]



# replace et lisse les points des groupes (contours puis surfaces)
def homogenize(mesh):
	pass

# découpe les faces et replace les points résultants pour améliorer la résolution des surfaes courbes (groupe par groupe)
def subdivide(mesh, density):
	pass

# retire les points les moins importants a la forme pour réduire la densité
def simplify(mesh, density):
	pass
