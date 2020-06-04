/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

flat in int flags;
uniform vec3 color;
uniform vec3 select_color;
out vec4 outcolor;

void main() {
	if ((flags & 1) != 0)		outcolor = vec4(select_color, 1);
	else						outcolor = vec4(color,1);
	
}
