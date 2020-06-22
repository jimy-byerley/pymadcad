/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

in vec3 v_position;	// vertex position
in int v_flags;
uniform mat4 view;	// view matrix (camera orientation)
uniform mat4 proj;	// projection matrix (perspective or orthographic)
flat out int flags;

void main() {
	flags = v_flags;
	gl_Position = proj * view * vec4(v_position,1) - vec4(0,0,1e-2,0);	// set vertex position for render, with an offset to display wires on the top of faces
	if ((flags & 1) != 0)	gl_Position[2] -= 1e-4;
}
