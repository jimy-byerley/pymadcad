''' Regroupement des fonctions et classes math de pymadcad '''

import glm
from glm import *

'''
vec3 = dvec3
mat3 = dmat3
vec4 = dvec4
mat4 = dmat4
'''

NUMPREC = 1e-6
COMPREC = 1-NUMPREC

"""
# set default types: NOTE there is a precision problem with float64: the results are not more precise
for cls in ['vec2', 'vec3', 'vec4', 'mat2', 'mat3', 'mat4', 'quat']:
	exec('''
class {0}(d{0}):
	def __copy__(self):
		return {0}(self)
	def __deepcopy__(self, memo):
		copy = {0}(self)
		memo[id(self)] = copy
		return copy
		'''.format(cls))
"""

def project(vec, dir):
	return dot(vec, dir) * dir

# donne la matrice de transformation 4x4
def transform(translation=None, rotation=None):
	if rotation is not None:	transform = mat4(rotation)
	else:						transform = mat4(1)
	if translation is not None:	transform[3] = vec4(translation)
	return transform

# donne la matrice de transformation 4x4 inverse
def inversetransform(transform):
	if isinstance(transform, tuple):		return (-transform[0], inversetransform(transform[1]))
	elif isinstance(transform, quat):		return 1/transform
	elif isinstance(transform, mat3):		return transpose(transform)
	elif isinstance(transform, mat4):
		rot = mat3(transform)
		inv = mat4(transpose(rot))
		inv[3] = vec4(- rot * transform[3])
		return inv
	else:
		raise typeError('transform must be a a tuple (translation, rotation), a quaternion, or a matrix')
