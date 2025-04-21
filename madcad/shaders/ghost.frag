/*
	shader for solid objects, with diffuse aspect based on the angle to the camera and a skybox reflect
*/
#version 330

in vec3 sight;		// vector from object to eye
in vec3 normal;		// normal to the surface
flat in int flags;
uniform vec4 selected_color;
uniform vec4 hovered_color;
uniform vec3 normal_color;	// face color when not selected

// render color
out vec4 color;

#define HOVERED   1<<0
#define SELECTED  1<<1

vec4 highlight(vec4 dst, vec4 src) {
	return vec4(mix(dst.rgb, src.rgb, src.a), dst.a);
}

void main() {
	vec3 nsight = normalize(sight);
	vec3 nnormal = normalize(normal);

	float glow = min(0.7, pow(1 - dot(nsight, nnormal), 3)) * int(gl_FrontFacing);
	if (flags != 0)  glow = min(1, glow+0.08);
	
	color = vec4(normal_color, glow);
	if ((flags & HOVERED)  != 0)		color = highlight(color, hovered_color);
	if ((flags & SELECTED) != 0)		color = highlight(color, selected_color);
}
