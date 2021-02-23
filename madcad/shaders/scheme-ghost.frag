#version 330

in vec3 sight;
in vec3 normal;
in vec4 color;
out vec4 o_color;

void main() {
	float diffuse = abs(dot(normalize(sight), normalize(normal)));
	float intensity = min(0.8, 0.1/max((diffuse-0.2)*1.5, 0.01));
	o_color = color * vec4(vec3(1), intensity);
}
