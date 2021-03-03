/*
	shader for render objects with uniform opaq color
*/
#version 330

in vec3 v_position;	// vertex position
uniform mat4 view;	// view matrix (camera orientation)
uniform mat4 proj;	// projection matrix (perspective or orthographic)
uniform float layer;

void main() {
	gl_Position = proj * view * vec4(v_position,1) + vec4(0,0,layer,0);	// set vertex position for render, with an offset to display wires on the top of faces
}
