/*
	shader for solid objects, with diffuse aspect based on the angle to the camera and a skybox reflect
*/
#version 330

out vec3 sight;		// vector from object to eye
out vec3 normal;	// normal to the surface
uniform vec3 min_color;		// color for dark zones
uniform vec3 max_color;		// color for light zones

// render color
out vec3 color;

void main() {
	float diffuse = max(0, dot(normalize(sight), normalize(normal)));
	color = mix(min_color, max_color, diffuse);
}


