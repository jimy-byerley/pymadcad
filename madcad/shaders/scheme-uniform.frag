#version 330

in vec4 color;
out vec4 o_color;

uniform vec4 highlight;

void main() {
	o_color = vec4(mix(color.rgb, highlight.rgb, highlight.a), color.a);
}
