/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

in float v_absciss;
in float v_alpha;
uniform vec3 origin;
uniform vec3 direction;
uniform vec2 interval;
uniform mat4 projview;
uniform mat4 world;
uniform uint ident;
flat out uint identcolor;

void main() {
	identcolor = ident;
	float x = interval[0] + (interval[1]-interval[0])*v_absciss;
	gl_Position = projview * world * vec4(origin + x*direction, 1);
}
