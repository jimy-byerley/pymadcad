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

void main() {
	flags = v_flags;
	
	vec4 p = world * vec4(v_position, 1);
	// world space vectors for the fragment shader
	sight = vec3(inverse(view) * vec4(0,0,0,1) - p);
	normal = mat3(world) * v_normal;
	
	gl_Position = proj * view * p;	// set vertex position for render
	// with an offset to display wires on the top of faces
	if ((flags&1) != 0)	gl_Position[2] += 10*layer;
	else				gl_Position[2] += layer;
}
