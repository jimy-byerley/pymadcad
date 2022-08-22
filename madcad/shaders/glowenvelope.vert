/*
	shader for solid objects, with diffuse aspect based on the angle to the camera and a skybox reflect
*/
#version 330

in vec3 v_position;	// vertex position
in vec3 v_normal;	// vertex normal
uniform mat4 view;	// view matrix (camera orientation)
uniform mat4 proj;	// projection matrix (perspective or orthographic)

// to compute
out vec3 sight;		// vector from object to eye
out vec3 normal;	// normal to the surface

vec3 sight_direction(vec4 p) {
	float f = proj[3][3] / dot(transpose(proj)[3], p) - 1;
	return vec3(p) * vec3(f,f,-1);
}

void main() {
	vec4 p = view * vec4(v_position, 1);
	// view space vectors for the fragment shader
	sight = sight_direction(p);
	normal = mat3(view) * v_normal;
	
	gl_Position = proj * p;	// set vertex position for render
}
