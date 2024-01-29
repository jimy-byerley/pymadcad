#version 330

uniform vec3 user_color;
// flat in int flags;
// uniform vec3 select_color;

// render color
out vec4 color;

void main() {
  // float rgb = smoothstep(0, 1, sin(gl_FragCoord.x * 0.1 + 0.7 * gl_FragCoord.y));
  float rgb = smoothstep(0., 1., 1. - mod(0.8 * gl_FragCoord.x + 0.4 * gl_FragCoord.y, 50.)) + smoothstep(0., 1., 1. - mod(0.8 * gl_FragCoord.x + 0.4 * gl_FragCoord.y + 10., 50.));
  vec3 all = vec3(rgb, rgb, rgb) + user_color;
  color = vec4(all, 1.);
	// if ((flags & 1) != 0)	color += vec4(select_color, 0);
}
