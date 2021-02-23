/*
	shader for render objects with uniform opaq color
*/
#version 330

in vec3 sight;
in vec3 normal;
uniform vec3 color;
out vec4 outcolor;

void main() {
	float diffuse = abs(dot(normalize(sight), normalize(normal)));
	float intensity = min(0.8, 0.1/max((diffuse-0.2)*1.5, 0.01));
 	outcolor = vec4(color, intensity);
}
