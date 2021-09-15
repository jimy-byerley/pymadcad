#version 330

in vec3 sight;
flat in mat4 inverse_view;
flat in vec3 voxel_width;
flat in float voxel_radius;
uniform float value_min;
uniform float value_max;
uniform vec4 color_min;
uniform vec4 color_max;
uniform mat4 view;
uniform mat4 proj;
uniform sampler3D voxel;
out vec4 outcolor;

// samples each pixel in the direction of sight
#define DIV 100
// opacity above which the depth value of the voxel is used
#define DEPTHCUT 0.1

// min component of a vec3
float amin(vec3 a) {
	float m = a[0];
	if (a[1] < m)	m = a[1];
	if (a[2] < m)	m = a[2];
	return m;
}


void main() {
	
	vec3 voxel_center = vec3(view[3]) + voxel_width;	// voxel center in camera space
	vec3 view_direction = normalize(sight);	// direction from camera to point in camera space
	// origin and direction of sampling in voxel space
	float start = max(0, dot(voxel_center, view_direction) - voxel_radius);
	vec3 origin = vec3(inverse_view * vec4(start * view_direction, 1) );	// origin of the sampling axis in voxel space
	vec3 direction = 2*voxel_radius * mat3(inverse_view) * view_direction; // direction of the sampling axis in voxel space
	
	
	// integrate color and opacity over the depth
	float integral = 0;
	vec4 melt = color_min;
	float far = 2*sight.z;
	float unique_depth = far;
	for (int i=1; i<DIV; i++) {
		vec3 position = origin + direction*(float(i)/DIV);
		
		if (0 < position.x && position.x < 1
		&&	0 < position.y && position.y < 1
		&&	0 < position.z && position.z < 1) 
		{
			// opacity reduction when close to the voxel limits, to hide the plane cuts
			float border = clamp(amin(min((1-position), position)/abs(direction)) * DIV/2, 0, 1);
			// density retreived from the voxel and used for integration
			float density = clamp(
								(texture(voxel, position).x - value_min) 
								/ (value_max-value_min),
								0, 1);
// 			float density = mix(value_min, value_max, texture(voxel, position).x);
			
			vec4 color = mix(color_min, color_max, density);  // local coloration mixing (light occlusion)
// 			melt += color * border*exp(-integral);            // local coloration contributing (light addition, not realistic)

			melt = mix(melt, color, border*exp(-integral));		// color mixing according to opacity
			integral += border*density; // integration of global opacity
			
			if (exp(-integral) <= DEPTHCUT && unique_depth==far)
				unique_depth = max(far, 2 * (view*vec4(position,1)).z);
		}
	}
	
// 	outcolor = vec4(vec3(5e-2*-unique_depth), 1);		// depth coloration for debug
 	//melt = mix(color_min, color_max, 20*integral/DIV);	// global coloration
	outcolor = vec4(melt.rgb, melt.a*(1-exp(-integral)));
	
	gl_FragDepth = (proj[2][2]*unique_depth + proj[3][2]) / (proj[2][3]*unique_depth);
}

