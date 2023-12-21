from madcad import *

# this is the content of the scene
cube = brick(width=vec3(1)) .transform(rotate(1, vec3(1,1,0)))
note = note_leading(cube.group(0), text="this is an annotation")

# render it and open the image using the OS tools
img = render([cube, note], size=uvec2(2160, 2160))
img.show()
img.save("kinematic.png")

# open a 3D view showing the same thing
show([cube, note])
