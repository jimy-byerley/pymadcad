/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

flat in vec3 identcolor;
out vec3 color;

void main() {
	color = identcolor;
}
