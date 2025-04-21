/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

in vec3 v_position;	// vertex position
in int v_flags;
uniform int u_flags;
uniform mat4 view;	// view matrix (camera orientation)
uniform mat4 proj;	// projection matrix (perspective or orthographic)
uniform float layer;
flat out int flags;

#define HOVERED   1<<0
#define SELECTED  1<<1

void main() {
	flags = v_flags | u_flags;
	
	gl_Position = proj * view * vec4(v_position,1); 				// set vertex position for render
	// with an offset to display wires on the top of faces
	int highlight = (1+int((flags&HOVERED)!=0)+int((flags&SELECTED)!=0));
	gl_Position[2] += layer * highlight * gl_Position[3];
}
