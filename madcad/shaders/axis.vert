#version 330

in float v_absciss;
in float v_alpha;
uniform vec3 origin;
uniform vec3 direction;
uniform vec2 interval;
uniform mat4 projview;
uniform mat4 world;

out float alpha;

void main() {
	alpha = v_alpha;
	float x = interval[0] + (interval[1]-interval[0])*v_absciss;
	gl_Position = projview * world * vec4(origin + x*direction, 1);
}
