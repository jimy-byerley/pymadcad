/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

flat in int flags;
uniform vec4 color;
uniform vec4 select_color;
out vec4 outcolor;

void main() {
	if ((flags & 1) != 0)		outcolor = select_color;
	else						outcolor = color;
	
}
