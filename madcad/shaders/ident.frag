/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

flat in uint identcolor;
out uint color;

void main() {
	color = identcolor;
}
