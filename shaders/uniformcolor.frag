/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

uniform vec3 color;
out vec3 outcolor;

void main() {
	outcolor = color;
}
