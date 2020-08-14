#version 330

in vec2 uv;
uniform vec3 color;
uniform sampler2D fonttex;

// render color
out vec4 out_color;

void main() {
	out_color = vec4(color, texture(fonttex, uv).r);
}
