#version 330

in float alpha;
uniform vec3 color;

out vec4 out_color;

void main() {
	out_color = vec4(color, alpha);
}
