/*
	shader for identification of all objects (assign a color for each object)
*/
#version 330

flat in int flags;
uniform vec4 color;
uniform vec4 selected_color;
uniform vec4 hovered_color;
out vec4 outcolor;

#define HOVERED   1<<0
#define SELECTED  1<<1

vec4 highlight(vec4 dst, vec4 src) {
	return vec4(mix(dst.rgb, src.rgb, src.a), dst.a);
}

void main() {
	outcolor = color;
	if (((flags) & HOVERED)  != 0)	outcolor = highlight(outcolor, hovered_color);
	if (((flags) & SELECTED) != 0)	outcolor = highlight(outcolor, selected_color);
}
