#version 330

in uint space;
in vec3 v_position;
in vec3 v_normal;
in vec4 v_color;
in float v_layer;
in uint v_ident;
in uint v_flags;

uniform mat4 proj;
uniform mat4 spaces[32];
uniform uint startident;

out vec3 normal;
out vec3 sight;
out vec4 color;
flat out vec3 identcolor;

void main() 
{
	color = v_color;
	uint ident = startident + v_ident;
	identcolor = vec3(float(ident % uint(256)), float(ident/uint(256)), 0)/255.;
	vec4 position = spaces[space] * vec4(v_position,1);
	normal = mat3(spaces[space]) * v_normal;
	sight = vec3(position);
	gl_Position = proj * position + vec4(0,0,v_layer,0);
}
