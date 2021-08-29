from .mathutils import *
from .mesh import connef, edgekey

from scipy.optimize import minimize



def segmentation(mesh, tolerance=5, sharp=0.1) -> Mesh:
	''' surface segmentation based on curvature.
		This functions splits faces into groups based on curvature proximity.
		Ideally each resulting group is a set of faces with the same curvature. In practice such is not possible due to the mesh resolution introducing bias in curvature estimation. Thus a `tolerance` is used to decide what is a sufficiently close curvature to belong to the same group.
		In order to avoid too sharp edges to be considered a very low resolution but smooth surface, `sharp` is the value of the limit edge angle that can be considered in smooth surface.
		
		This function is particularly usefull when importing geometries from a format that doesn't manage groups (such as STL).
		
		Parameters:
		
			tolerance (float):	maximum difference factor between curvatures of a same group  (1 means +100% curvature is allowed)
			sharp (float):	minimum angle for a sharp edge (radians)
			
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
	assigned = [False] * len(mesh.faces)
	conn = connef(mesh.faces)
	facenormals = mesh.facenormals()
	vertexnormals = mesh.vertexnormals()
	
	# this is what this functions is working to create: new groups and tracks to assign it to faces
	tracks = [-1] * len(mesh.faces)
	groups = []
	
	# evaluate curvature around every edge
	edgecurve = {}
	facecurve = [0] * len(mesh.faces)
	weight = [0] * len(mesh.faces)
	for e, fa in conn.items():
		if edgekey(*e) in edgecurve:	continue
		fb = conn.get((e[1],e[0]))
		if not fb:	continue
		
		curve = anglebt(facenormals[fa], facenormals[fb])
		edgecurve[edgekey(*e)] = curve
		
		# report average curvature to neighboring faces
		l = distance(pts[e[0]], pts[e[1]])
		w = max(0, sharp-curve)   # weight computed in term of proximity to sharp
		facecurve[fa] += curve * l * w  # multiply angle at edge by   (triangle height) ~= surface/(edge length)
		facecurve[fb] += curve * l * w
		weight[fa] += w
		weight[fb] += w
	
	# divide contributions to face curvatures
	for i,f in enumerate(mesh.faces):
		facecurve[i] /= length(cross(pts[f[1]]-pts[f[0]], pts[f[2]]-pts[f[0]])) * weight[i]
		#debug.append(text.Text(
							#(pts[f[0]] + pts[f[1]] + pts[f[2]]) /3,
							#'{:.2f}'.format(facecurve[i]),
							#size=8,
							#align=('left', 'center'),
							#))
		
	# groupping gives priority to the less curved faces, so they can extend to the ambiguous limits if there is
	sortedfaces = sorted(range(len(mesh.faces)), key=lambda i:  facecurve[i])
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
			if abs(facecurve[j]-estimate) / (estimate+NUMPREC) >= tolerance:	continue
			
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

def guessjoint(solid1, solid2, surface1, surface2):
	from .joints import Planar, Gliding, Punctiform, Ball
	
	joints = {
		('plane', 'plane'): Planar,
		#('cylinder', 'plane'): Rolling,,
		('cylinder', 'cylinder'): Gliding,
		#('cylinder', 'sphere'): Anular,
		('plane', 'sphere'): Punctiform,
		('sphere', 'sphere'): Ball,
		}
	t1, p1 = guesssurface(surface1)
	t2, p2 = guesssurface(surface2)
	return joints[sorted([t1, t2])](solid1, solid2, p1, p2)
	
def guesssurface(surface):	
	if not isinstance(surface, Mesh):
		return surface
	
	center = surface.barycenter()
	precision = surface.maxnum() * 1e-5
	scores = []
	# plane regression
	def evaluate(x):
		score = (length2(vec3(x[3:])) - 1)**2
		origin, normal = vec3(x[:3]), normalize(vec3(x[3:]))
		for f in surface.faces:
			fp = surface.facepoints(f)
			p = (fp[0]+fp[1]+fp[2]) / 3
			score += dot(p-origin, normal)**2
		return score / len(surface.faces)
	res = minimize(evaluate, [*center, 1,1,1], tol=precision, method='CG')
	print('plane', res.nfev, res.x, res.fun)
	scores.append((
			res.fun,
			(vec3(res.x[:3]), vec3(res.x[3:])),
			))
	
	# sphere regression
	def evaluate(x):
		score = 0
		origin = vec3(x[:3])
		radius = x[3]
		for f in surface.faces:
			fp = surface.facepoints(f)
			p = (fp[0]+fp[1]+fp[2]) / 3
			score += (distance(p, origin) - radius) **2
		return score / len(surface.faces)
	res = minimize(evaluate, [*center, 1], tol=precision, method='CG')
	print('sphere', res.nfev, res.x, res.fun)
	scores.append((
			res.fun,
			vec3(res.x[:3]),
			))
		
	# cylinder regression
	def evaluate(x):
		score = (length(vec3(x[3:6])) - 1) **2
		origin, direction = vec3(x[:3]), vec3(x[3:6])
		radius = x[6]
		for f in surface.faces:
			fp = surface.facepoints(f)
			p = (fp[0]+fp[1]+fp[2]) / 3
			score += (length(noproject(p-origin, direction)) - radius) **2
		return score / len(surface.faces)
	res = minimize(evaluate, [*center, 1,1,1, 1], tol=precision, method='CG')
	print('cylinder', res.nfev, res.x, res.fun)
	scores.append((
			res.fun,
			(vec3(res.x[:3]), vec3(res.x[3:])),
			))
		
	best = imax(-score[0] for score in scores)
	return (
			['plane', 'sphere', 'cylinder'][best], 
			scores[best][1],
			)


# associe des groupes aux faces, en fonction des régularités topologiques
def guess(mesh):
	pass

# replace et lisse les points des groupes (contours puis surfaces)
def homogenize(mesh):
	pass

# découpe les faces et replace les points résultants pour améliorer la résolution des surfaes courbes (groupe par groupe)
def subdivide(mesh, density):
	pass

# retire les points les moins importants a la forme pour réduire la densité
def simplify(mesh, density):
	pass
