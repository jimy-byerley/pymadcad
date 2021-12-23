#version 330

in vec3 v_position;
uniform mat4 view;
uniform mat4 proj;
out vec3 sight;
flat out mat4 inverse_view;
flat out vec3 voxel_width;
flat out float voxel_radius;

void main() {
	vec4 tmp = view*vec4(v_position,1);
	gl_Position = proj*tmp;
	
	sight = vec3(tmp);										// vector to the surface in camera space
	inverse_view = inverse(view);
	voxel_width = 0.5*vec3(view[0]+view[1]+view[2]);		// voxel width in camera space
	voxel_radius = length(voxel_width);
}
