/*
	shader for render objects with uniform opaq color
*/
#version 330

uniform vec3 color;
out vec4 outcolor;

void main() {
	outcolor = vec4(color, 1);
}
