#version 330

uniform vec2 line_slope;      // vec2(dx, dy) where in reality, slope = dy / dx
uniform float pattern_gap;    // gap between patterns
uniform float repetition;     // number of parallel lines inside a pattern
uniform float repetition_gap; // gap betweenn repetitions inside a pattern
uniform vec3 user_color;
// flat in int flags;
// uniform vec3 select_color;

// render color
out vec4 color;

void main() {
  float rgb = 0.;
  for(int i = 0; i < repetition; i++){
    rgb += smoothstep(0., 1., 1. - mod(line_slope.x * gl_FragCoord.x + line_slope.y * gl_FragCoord.y + i * repetition_gap, pattern_gap));
  }
  vec3 all = vec3(rgb, rgb, rgb) + user_color;
  color = vec4(all, 1.);
	// if ((flags & 1) != 0)	color += vec4(select_color, 0);
}
