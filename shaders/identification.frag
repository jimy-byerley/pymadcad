/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

uniform uint ident;
out uint color;

void main() {
	color = ident;
}
