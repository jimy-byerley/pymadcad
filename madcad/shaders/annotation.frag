/*
	shader for render objects with uniform opaq color
*/
#version 330

uniform vec3 color;
in float alpha;
out vec4 outcolor;

void main() {
	outcolor = vec4(color, alpha);
}
