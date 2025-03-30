#version 330

in vec2 v_uv;
uniform vec3 position;
uniform mat4 proj;
uniform mat4 view;
uniform vec2 ratio;
uniform uint ident;

flat out uint identcolor;

void main() {
	identcolor = ident;
	vec4 p = view * vec4(position, 1);
	gl_Position = proj*p	// origin point projection
				+ vec4((2*v_uv-1)*ratio, 0, 0) * dot(transpose(proj)[3], p)	// relative point position with perspective compensation
				+ vec4(0,0,-1e-3,0);	// layer
}
