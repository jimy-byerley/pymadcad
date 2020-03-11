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
	if      (tosky.z>=  as.x && tosky.z>=  as.y)	sky_coord = vec2( tosky.y/tosky.z+4, -tosky.x/tosky.z+2); // top
	else if (tosky.y>=  as.x && tosky.y>=  as.z)	sky_coord = vec2( tosky.x/tosky.y+2, -tosky.z/tosky.y);   // front
	else if (tosky.x>=  as.y && tosky.x>=  as.z)	sky_coord = vec2(-tosky.y/tosky.x+4, -tosky.z/tosky.x);   // left
	else if (tosky.z<= -as.x && tosky.z<= -as.y)	sky_coord = vec2( tosky.y/tosky.z,    tosky.x/tosky.z+2); // bottom
	else if (tosky.y<= -as.x && tosky.y<= -as.z)	sky_coord = vec2(-tosky.z/tosky.y+2,  tosky.x/tosky.y+2); // back
	else                                            sky_coord = vec2(-tosky.y/tosky.x,    tosky.z/tosky.x);   // right
	
	return (sky_coord+1)/6;
}

void main() {
	float diffuse = max(0, dot(normalize(sight), normalize(normal)));
	vec3 tosky = reflect(sight, normal);
	vec3 refl = texture2D(reflectmap, skybox(tosky)).rgb;
	
	color = vec4(refl * refl_color + mix(min_color, max_color, diffuse), 1);
	if ((flags & 1) != 0)		color += vec4(select_color, 0) * min(1/max(0,(diffuse-0.1)), 10);
}


