/*
	shader for solid objects, with diffuse aspect based on the angle to the camera and a skybox reflect
*/
#version 330

in vec3 sight;		// vector from object to eye
in vec3 normal;		// normal to the surface
flat in int flags;
uniform vec3 min_color;		// color for dark zones
uniform vec3 max_color;		// color for light zones
uniform vec3 refl_color;	// color for the reflect
uniform vec3 select_color;
uniform sampler2D reflectmap;	// color of the sky for reflection
uniform int selection;

// render color
out vec4 color;

vec2 skybox(vec3 tosky) {
	// We do cut the cube into 6 angular sectors, from each face to the center of the cube. 
	// A point is in a sector if the face's side coordinate is higher than the absulute value of other coordinates
	vec3 as = abs(tosky);
	vec2 sky_coord; // texture coordinates of the sky point viewed
	// selection of the face to use (so texture sector)
	if      (tosky.z>=  as.x && tosky.z>=  as.y)	sky_coord = vec2( tosky.y/as.z,   tosky.x/as.z)*0.15 + vec2(0.5, 0.4); // top
	else if (tosky.y>=  as.x && tosky.y>=  as.z)	sky_coord = vec2(-tosky.z/as.y,   tosky.x/as.y)*0.15 + vec2(0.8, 0.4);   // front
	else if (tosky.x>=  as.y && tosky.x>=  as.z)	sky_coord = vec2( tosky.z/as.x,   tosky.y/as.x)*0.15 + vec2(0.8, 0.8);   // left
	else if (tosky.z<= -as.x && tosky.z<= -as.y)	sky_coord = vec2( tosky.x/as.z,   tosky.y/as.z)*0.15 + vec2(0.5, 0.8); // bottom
	else if (tosky.y<= -as.x && tosky.y<= -as.z)	sky_coord = vec2( tosky.z/as.y,   tosky.x/as.y)*0.15 + vec2(0.2, 0.4); // back
	else                                            sky_coord = vec2(-tosky.z/as.x,   tosky.y/as.x)*0.15 + vec2(0.2, 0.8);   // right
	
	return sky_coord;
}

void main() {
	vec3 nsight = normalize(sight);
	vec3 nnormal = normalize(normal);
	float diffuse = max(0, dot(nsight, nnormal));
	vec3 tosky = -reflect(nsight, nnormal);
	vec3 refl = texture(reflectmap, skybox(tosky)).rgb;
	
	color = vec4(refl * refl_color + mix(min_color, max_color, diffuse), 1);
	if ((flags & 1) != 0)		color += vec4(select_color, 0) * min(1/max(0,(diffuse-0.1)), 10);
}


