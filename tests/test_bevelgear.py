#!/usr/bin/python3
from madcad.gear import *
from madcad.mesh import edgekey, facekeyo

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

def testing(function):
    def wrapper(name, *args, **kwargs):
        print("{}: {}, {}".format(name, args, kwargs))
        try:
            function(name, *args, **kwargs)
            print(f"{bcolors.OKGREEN}Passed !\n{bcolors.ENDC}")
        except Exception as e:
            print(bcolors.WARNING + str(e) + bcolors.ENDC)
            print(f"{bcolors.FAIL}>>> Failed ! <<<\n{bcolors.ENDC}")
    return wrapper

@testing
def test_bevel_gear(name, *args, **kwargs):
    bevel_gear(*args, **kwargs)

@testing
def test_spherical_rack_profile(name, *args, **kwargs):
    spherical_rack_profile(*args, **kwargs)


z_pinion = 25
z_wheel = 36
m = 4
shaft_angle = pi / 2
gamma_p = get_pitch_cone_angle(z_pinion, z_wheel)
pressure_angle = pi / 9

test_bevel_gear("Bevel gear pinion", pi * m, z_pinion, gamma_p, pressure_angle = pressure_angle)
test_bevel_gear("Bevel gear wheel", pi * m, z_wheel, shaft_angle -  gamma_p, pressure_angle = pressure_angle)
test_spherical_rack_profile("Spherical rack", z_pinion/sin(gamma_p))

z_pinion = 25
z_wheel = 36
m = 1
shaft_angle = pi / 2
gamma_p = get_pitch_cone_angle(z_pinion, z_wheel)
pressure_angle = radians(30)

test_bevel_gear("Bevel gear pinion", pi * m, z_pinion, gamma_p, pressure_angle = pressure_angle, bore_radius=0)
test_bevel_gear("Bevel gear wheel", pi * m, z_wheel, shaft_angle - gamma_p, pressure_angle = pressure_angle, bore_radius=0)
test_spherical_rack_profile("Spherical rack", z_pinion/sin(gamma_p))

z_pinion = 36
z_wheel = 36
m = 1
shaft_angle = pi / 2
gamma_p = get_pitch_cone_angle(z_pinion, z_wheel)
pressure_angle = radians(18)

test_bevel_gear("Bevel gear pinion", pi * m, z_pinion, gamma_p, pressure_angle = pressure_angle, bore_height=0)
test_bevel_gear("Bevel gear wheel", pi * m, z_wheel, shaft_angle - gamma_p, pressure_angle = pressure_angle, bore_height=0)
test_spherical_rack_profile("Spherical rack", z_pinion/sin(gamma_p))

z_pinion = 12
z_wheel = 40
m = 1
shaft_angle = pi / 2
gamma_p = get_pitch_cone_angle(z_pinion, z_wheel)
pressure_angle = pi / 9

test_bevel_gear("Bevel gear pinion", pi * m, z_pinion, gamma_p, pressure_angle = pressure_angle, bore_height=5.5)
test_bevel_gear("Bevel gear wheel", pi * m, z_wheel, shaft_angle - gamma_p, pressure_angle = pressure_angle, bore_height=5.5)
test_spherical_rack_profile("Spherical rack", z_pinion/sin(gamma_p))

z_pinion = 12
z_wheel = 40
m = 1
shaft_angle = pi / 3
gamma_p = get_pitch_cone_angle(z_pinion, z_wheel)
pressure_angle = radians(25)

test_bevel_gear("Bevel gear pinion", pi * m, z_pinion, gamma_p, pressure_angle = pressure_angle, ka = 0.8, kd = 1.5)
test_bevel_gear("Bevel gear wheel", pi * m, z_wheel, shaft_angle - gamma_p, pressure_angle = pressure_angle)
test_spherical_rack_profile("Spherical rack", z_pinion/sin(gamma_p))

z_pinion = 12
z_wheel = 40
m = 1
shaft_angle = pi / 3
gamma_p = get_pitch_cone_angle(z_pinion, z_wheel)
pressure_angle = pi / 9

test_bevel_gear("Bevel gear pinion", pi * m, z_pinion, gamma_p, pressure_angle = pressure_angle, ka=0.65, kd = 1)
test_bevel_gear("Bevel gear wheel", pi * m, z_wheel, shaft_angle - gamma_p, pressure_angle = pressure_angle)
test_spherical_rack_profile("Spherical rack", z_pinion/sin(gamma_p))
