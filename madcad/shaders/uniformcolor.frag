/*
	shader for render objects with uniform opaq color
*/
#version 330

uniform vec4 color;
out vec4 outcolor;

void main() {
	outcolor = color;
}
