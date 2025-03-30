/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

in vec3 v_position;	// vertex position
uniform mat4 view;	// view matrix (camera orientation)
uniform mat4 proj;	// projection matrix (perspective or orthographic)
uniform float layer;
uniform uint ident;
flat out uint identcolor;

void main() {
	identcolor = ident;
	gl_Position = proj * view * vec4(v_position,1)  + vec4(0,0,layer,0);	// set vertex position for render
}
