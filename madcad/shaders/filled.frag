#version 330

uniform vec3 user_color;
// flat in int flags;
// uniform vec3 select_color;

// render color
out vec4 color;

void main() {
	color = vec4(user_color, 1.);
	// if ((flags & 1) != 0)	color += vec4(select_color, 0);
}
