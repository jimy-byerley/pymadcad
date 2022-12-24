#version 330

uniform mat4 proj;
uniform float centerdist;
uniform float size;
uniform vec4 color;
uniform float contrast;

in vec2 v_position;
in float v_opacity;

out vec4 v_color;

void main() {
	vec4 pos = proj * vec4(vec2(v_position*size), -centerdist, 1);
	float alpha = mix(length(0.8*vec2(pos.x,pos.y)) / pos.w, 1, 0.4);
	v_color = vec4(color.rgb, color.a * exp(contrast*v_opacity) * alpha);
	gl_Position = pos;
}
