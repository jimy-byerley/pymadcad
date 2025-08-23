#version 430

flat in uint identcolor;
out uint color;

void main() {
	color = identcolor;
}
