/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

flat in vec3 ident;
out vec3 color;

void main() {
	color = ident;
}
