from madcad import *
	
r = 1
h = 0.7
e = 0.1

# generate the shape to write on
shape = cylinder(vec3(0,0,-h), vec3(0,0,h), r)

# generate flat text as a Web (wireframe mesh)
label = text.text("foo bar", fill=False)
# wrap the mesh points around the cylinder
label.points = typedlist(vec3(cos(p.x/r), sin(p.x/r), p.y-0.5)*r  
                      for p in label.points)
# extrude the mesh and give it a cylindric filling
label = intersection(
		extrusion(
			label.transform(scale(vec3(1+e*1.1, 1+e*1.1, 1))),
			scale(vec3(1-e-0.1*r, 1-e-0.1*r, 1)), 
			), 
		inflate(shape, e),
		)
# merge it onto the cylinder
result = union(shape, label).finish()
print(repr(result))
