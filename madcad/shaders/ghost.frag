/*
	shader for solid objects, with diffuse aspect based on the angle to the camera and a skybox reflect
*/
#version 330

in vec3 sight;		// vector from object to eye
in vec3 normal;		// normal to the surface
flat in int flags;
uniform vec3 normal_color;	// face color when not selected
uniform vec3 select_color;	// color for selected border

// render color
out vec4 color;

void main() {
	vec3 nsight = normalize(sight);
	vec3 nnormal = normalize(normal);
	float side;
	if (gl_FrontFacing)				side = min(0.7, pow(1 - dot(nsight, nnormal), 3));
	else							side = 0;
	
	if ((flags & 1) != 0)	color = vec4(select_color, min(1,side+0.08));
	else					color = vec4(normal_color, side);
}


