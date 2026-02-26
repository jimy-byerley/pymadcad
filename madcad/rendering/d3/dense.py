import moderngl as mgl

import numpy as np
from pyglm.glm import fvec4, fmat4
from arrex import typedlist

from ...mathutils import vec3
from ...common import resourcedir
from ...mesh import typedlist_to_numpy
from .. import Display


class VoxelsDisplay(Display):
	''' display a voxel as a scalar field.
		its opacity is computed using its density as absorbance.
		
		Parameters:
			
			voxel:	a numpy array with dimension 3, no specific value range is required
			
		Attributes:
		
			space:	transformation matrix from voxel normalized coordinates (position in the array as range 0-1) to local coordinates
			world:	transformation matrix from local to world coordinates
			value_range:	
				
				tuple `(min, max)`  giving the min and max values to display
				- values below min will be transparent
				- values above max will be fully opaq
				
			color_range:	tuple `(fvec4, fvec4)` of colors matching the value range bounds
		
		value examples:
		
			* value_range
			
				```
				(0.4, 0.9)  # erase the lower value and saturate the maxium ones
				```
				
			* color_range
			
				```
				(fvec4(0,0,1,1), fvec4(0,1,0,1))	 # blue-green
				(fvec4(0,0.5,1,1), fvec4(1,1,0,1))   # lightblue-yellow
				(fvec4(1,0,0.3,1), fvec4(1,1,0,1))   # red-yellow
				(fvec4(0,0,1,1), fvec4(1,0.5,0.1,1)) # blue-orange
				(fvec4(0,0,1,1), fvec4(1,0.1,0.1,1)) # blue-red
				(fvec4(0,0,0,1), fvec4(1,1,1,1))     # black-white
				(fvec4(1,1,1,1), fvec4(1,1,1,1))     # white-white
				```
	'''
	def __init__(self, scene, voxel: np.ndarray, space: fmat4, 
				value=(0, 1), 
				color=(fvec4(0,0,1,1), fvec4(0,1,0,1))):
		
		self.voxel = scene.context.texture3d(
			voxel.shape, 
			1, 
			voxel.transpose(2,1,0).copy().astype('f2'),  # axis permutation for opengl
			dtype='f2',
			)
		self.voxel.repeat_x = False
		self.voxel.repeat_y = False
		self.voxel.repeat_z = False
		self.space = space
		self.value_range = value
		self.color_range = color
		
		self.shader, self.vb = scene.share(type(self), self._share)
		self.va = scene.context.vertex_array(
				self.shader,
				[(self.vb, '3f', 'v_position')],
				)
	
	def _share(self, scene):
		from ... import generation
		
		# load shader
		shader = scene.context.program(
					vertex_shader=open(resourcedir+'/shaders/voxel.vert').read(),
					fragment_shader=open(resourcedir+'/shaders/voxel.frag').read(),
					)
		# load vertex buffer for the brick
		brick = generation.brick(min=vec3(0), max=vec3(1)) .flip()
		pts = typedlist(vec3)
		for face in brick.faces:
			pts.extend(brick.facepoints(face))
		vb = scene.context.buffer(typedlist_to_numpy(pts, 'f4'))
		
		return shader, vb
	
	def stack(self, scene):
		return (self, 'screen', 10, self._render),

	def _render(self, view):
		view.scene.context.enable_only(mgl.DEPTH_TEST | mgl.BLEND | mgl.CULL_FACE)
		self.voxel.use(0)
		self.va.program['value_min'] = self.value_range[0]
		self.va.program['value_max'] = self.value_range[1]
		self.va.program['color_min'].write(self.color_range[0])
		self.va.program['color_max'].write(self.color_range[1])
		self.va.program['view'].write(view.uniforms['view'] * self.world * self.space)
		self.va.program['proj'].write(view.uniforms['proj'])
		self.va.render(mgl.TRIANGLES)
		view.scene.context.enable_only(mgl.DEPTH_TEST | mgl.BLEND)
