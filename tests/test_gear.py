from madcad.gear import *
from madcad.mesh import edgekey, facekeyo
from madcad.kinematic import Solid

# Colors
class bcolors:
	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKCYAN = '\033[96m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	ENDC = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m'

results = []

def test_gear(name, *args, **kwargs):
	print("{}: {}, {}".format(name, args, kwargs))
	try:
		mesh = gear(*args, **kwargs)
		if mesh.isenvelope():
			print(f"{bcolors.OKGREEN}Passed !\n{bcolors.ENDC}")
		else:
			print(f"{bcolors.FAIL}>>> Failed ! <<<\n{bcolors.ENDC}")
	except Exception as e:
		print(bcolors.WARNING + str(e) + bcolors.ENDC)
		print(f"{bcolors.FAIL}>>> Failed ! <<<\n{bcolors.ENDC}")
		raise
	else:
		results.append(mesh)
	#show([mesh])

def testing(function):
	def wrapper(name, *args, **kwargs):
		print("{}: {}, {}".format(name, args, kwargs))
		try:
			function(name, *args, **kwargs)
			print(f"{bcolors.OKGREEN}Passed !\n{bcolors.ENDC}")
		except Exception as e:
			print(bcolors.WARNING + str(e) + bcolors.ENDC)
			print(f"{bcolors.FAIL}>>> Failed ! <<<\n{bcolors.ENDC}")
			raise
	return wrapper

@testing
def test_full(name, *args, **kwargs):
    pattern_full(*args, **kwargs)

@testing
def test_circle(name, *args, **kwargs):
    pattern_circle(*args, **kwargs)

@testing
def test_rect(name, *args, **kwargs):
    pattern_rect(*args, **kwargs)

@testing
def test_rounded(name, *args, **kwargs):
    pattern_rounded(*args, **kwargs)

# Automatic part

test_gear("full test", 10, 30, 10)
test_gear("helical test", 10, 30, 10, helix_angle = radians(20))
test_gear("chamfer test", 10, 30, 10, chamfer = pi / 8)
test_gear("circle test", 10, 30, 10, pattern = "circle")
test_gear("rect test", 10, 30, 10, pattern = "rect")
test_gear("rounded test", 10, 30, 10, pattern = "rounded")
test_gear("hub test", 10, 30, 10, bore_radius = 5)

# Custom part

test_gear("only bore test", 10, 30, 10, bore_radius = 5, hub_height = 0)
test_gear("height hub test", 10, 30, 10, bore_radius = 5, hub_height = 10)
test_gear("circle spec height test", 10, 30, 10, pattern = "circle", int_height = 2.5)
test_gear("rect spec height test", 10, 30, 10, pattern = "rect", int_height = 2.5)
test_gear("rounded spec height test", 10, 30, 10, pattern = "rounded", int_height = 2.5)

# Different values

test_gear("gearprofile values test", 10, 20, 15)
test_gear("gearprofile values test", 15, 25, 15)
test_gear("gearprofile values test", 1.2, 6, 2)
test_gear("gearprofile values test", 2, 10, 2)

test_gear("gearexterior values test", 10, 20, 15, chamfer = pi/4)
test_gear("gearexterior values test", 10, 20, 15, helix_angle = radians(20))
test_gear("gearexterior values test", 15, 25, 15, chamfer = pi/5)
test_gear("gearexterior values test", 15, 25, 15, helix_angle = radians(22))
test_gear("gearexterior values test", 1.2, 6, 2, chamfer = pi/6)
test_gear("gearexterior values test", 1.2, 6, 2, helix_angle = radians(20))
test_gear("gearexterior values test", 2, 10, 2, chamfer = pi/8)
test_gear("gearexterior values test", 2, 10, 2, helix_angle = radians(30))

# full pattern tests
test_full("test full pattern", 10, 5, 5, int_height = 2)
test_full("test full pattern", 20, 5, 15, int_height = 5)
test_full("test full pattern", 60.5, 52, 1, int_height = 0)
test_full("test full pattern", 15, 5, 15, int_height = 0)

# circle pattern tests
test_circle("test circle pattern", 10, 5, 5, int_height = 2)
test_circle("test circle pattern", 20, 5, 15, int_height = 5, ratio = 1.5)
test_circle("test circle pattern", 60.5, 52, 1, int_height = 0, ratio = 0.6)
test_circle("test circle pattern", 15, 5, 15, int_height = 0)
test_circle("test circle pattern", 30, 2, 15, int_height = 0, circles_radius = 5, circle_place = 15)
test_circle("test circle pattern", 60, 5, 15, int_height = 0, circles_radius = 5, circle_place = 25.5)
test_circle("test circle pattern", 5, 0.2, 15, int_height = 0, circles_radius = 0.2, circle_place = 3)
test_circle("test circle pattern", 15, 5, 15, int_height = 5, patterns = 6)
test_circle("test circle pattern", 15, 5, 15, int_height = 5, patterns = 4)
test_circle("test circle pattern", 15, 5, 15, int_height = 5, patterns = 3)
test_circle("test circle pattern", 15, 5, 15, int_height = 5, patterns = 2)

# rect pattern tests
test_rect("test rect pattern", 10, 5, 5, int_height = 2)
test_rect("test rect pattern", 20, 5, 15, int_height = 5, ratio = 0.8)
test_rect("test rect pattern", 60.5, 52, 1, int_height = 0, ratio = 1.5)
test_rect("test rect pattern", 15, 5, 15, int_height = 0) # ...
test_rect("test rect pattern", 30, 2, 15, int_height = 0, int_thickness = 3, ext_thickness = 0)
test_rect("test rect pattern", 60, 5, 15, int_height = 0, int_thickness = 5, ext_thickness = 4.5)
test_rect("test rect pattern", 5, 0.2, 15, int_height = 0, int_thickness = 0.8, ext_thickness = 2) # ...
test_rect("test rect pattern", 15, 5, 15, int_height = 5, patterns = 6)
test_rect("test rect pattern", 15, 5, 15, int_height = 5, patterns = 4) # ...
test_rect("test rect pattern", 15, 5, 15, int_height = 5, patterns = 3) # ...
test_rect("test rect pattern", 15, 5, 15, int_height = 5, patterns = 2)

# rounded pattern tests
test_rounded("test rounded pattern", 10, 5, 5, int_height = 2)
test_rounded("test rounded pattern", 20, 5, 15, int_height = 5, ratio = 0.8)
test_rounded("test rounded pattern", 60.5, 52, 1, int_height = 0, ratio = 1.5)
test_rounded("test rounded pattern", 15, 5, 15, int_height = 0) # ...
test_rounded("test rounded pattern", 30, 2, 15, int_height = 0, int_thickness = 3, ext_thickness = 0)
test_rounded("test rounded pattern", 60, 5, 15, int_height = 0, int_thickness = 5, ext_thickness = 4.5)
test_rounded("test rounded pattern", 5, 0.2, 15, int_height = 0, int_thickness = 0.8, ext_thickness = 2) # ...
test_rounded("test rounded pattern", 15, 5, 15, int_height = 5, patterns = 6)
test_rounded("test rounded pattern", 15, 5, 15, int_height = 5, patterns = 4) # ...
test_rounded("test rounded pattern", 15, 5, 15, int_height = 5, patterns = 3) # ...
test_rounded("test rounded pattern", 15, 5, 15, int_height = 5, patterns = 2)


x = 0
for i, part in enumerate(results):
	box = boundingbox(part)
	results[i] = solid = Solid(content=part)
	solid.position = vec3(x-box.min.x,0,0)
	x +=  box.width.x
show(results)
