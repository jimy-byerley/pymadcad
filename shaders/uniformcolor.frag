/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

uniform vec3 color;
out vec4 outcolor;

void main() {
	outcolor = vec4(color,1);
}
