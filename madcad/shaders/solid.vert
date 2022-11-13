/*
	shader for solid objects, with diffuse aspect based on the angle to the camera and a skybox reflect
*/
#version 330

in vec3 v_position;	// vertex position
in vec3 v_normal;	// vertex normal
in int v_flags;
uniform mat4 world;	// world matrix (object pose)
uniform mat4 view;	// view matrix (camera orientation)
uniform mat4 proj;	// projection matrix (perspective or orthographic)
uniform float layer;

// to compute
out vec3 sight;		// vector from object to eye
out vec3 normal;	// normal to the surface
flat out int flags;

vec3 sight_direction(vec4 p) {
	float f = proj[3][3] / dot(transpose(proj)[3], p) - 1;
	return vec3(p) * vec3(f,f,-1);
}

void main() {
	flags = v_flags;
	
	vec4 p = world * vec4(v_position, 1);
	// world space vectors for the fragment shader
	sight = transpose(mat3(view)) * sight_direction(view*p);
	normal = mat3(world) * v_normal;
	
	gl_Position = proj * view * p;	// set vertex position for render
	// with an offset to display wires on the top of faces
	if ((flags&1) != 0)	gl_Position[2] += 2*layer*gl_Position[3];
	else				gl_Position[2] += layer*gl_Position[3];
}
