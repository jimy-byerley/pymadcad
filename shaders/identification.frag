/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

uniform uint ident;
out vec3 color;

void main() {
	color = vec3(float(ident % uint(256)), float(ident/uint(256)), 0)/255.;
}
