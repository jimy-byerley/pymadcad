#version 330

flat in uint identcolor;
out uint color;

void main() {
	color = identcolor;
}
