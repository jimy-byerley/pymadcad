""" Displays for tensor based data:  images, voxels, vector fields, ... """


class VoxelDisplay(Display):
	''' Display a voxel as a scalar field.
		Its opacity is computed using its density as absorbance.
		
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
		
		Value examples:
		
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
	def __init__(self, scene, voxel: 'ndarray', space: 'fmat4', 
				value=(0, 1), 
				color=(fvec4(0,0,1,1), fvec4(0,1,0,1))):
		
		self.voxel = scene.ctx.texture3d(
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

		def load(scene):
			from . import generation
			
			# load shader
			shader = scene.ctx.program(
						vertex_shader=open(resourcedir+'/shaders/voxel.vert').read(),
						fragment_shader=open(resourcedir+'/shaders/voxel.frag').read(),
						)
			# load vertex buffer for the brick
			brick = generation.brick(min=vec3(0), max=vec3(1)) .flip()
			pts = typedlist(vec3)
			for face in brick.faces:
				pts.extend(brick.facepoints(face))
			vb = scene.ctx.buffer(typedlist_to_numpy(pts, 'f4'))
			
			return shader, vb
		
		self.shader, self.vb = scene.resource('shader_voxel', load)
		self.va = scene.ctx.vertex_array(
				self.shader,
				[(self.vb, '3f', 'v_position')],
				)
				
	def __del__(self):
		self.va.release()
		self.voxel.release()

	def render(self, view):
		view.scene.ctx.enable(mgl.DEPTH_TEST | mgl.CULL_FACE)
		self.shader['value_min'] = self.value_range[0]
		self.shader['value_max'] = self.value_range[1]
		self.shader['color_min'].write(self.color_range[0])
		self.shader['color_max'].write(self.color_range[1])
		self.shader['view'].write(view.uniforms['view'] * self.world * self.space)
		self.shader['proj'].write(view.uniforms['proj'])
		self.voxel.use(0)
		self.va.render(mgl.vertex_array.TRIANGLES)
		
	def stack(self, scene):
		return ((), 'screen', 10, self.render),



class VoxelDisplay:
    pass


class FieldDisplay:
    pass
