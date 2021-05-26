#!/usr/bin/python3
from madcad.gear import *

def test_gear(name, *args, **kwargs):
    print("{}: {}, {}".format(name, args, kwargs))
    mesh = gear(*args, **kwargs)
    if mesh.isenvelope():
        print("Passed !\n")
    else:
        print(">>> Failed ! <<<\n")
        try:
            show([mesh])
        except Exception as e:
            print(e)

def test_full(name, *args, **kwargs):
    print("{}: {}, {}".format(name, args, kwargs))
    try:
        full_pattern(*args, **kwargs)
        print("Passed !\n")
    except Exception as e:
        print(e)
        print(">>> Failed ! <<<\n")

def test_circle(name, *args, **kwargs):
    print("{}: {}, {}".format(name, args, kwargs))
    try:
        circle_pattern(*args, **kwargs)
        print("Passed !\n")
    except Exception as e:
        print(e)
        print(">>> Failed ! <<<\n")

def test_rect(name, *args, **kwargs):
    print("{}: {}, {}".format(name, args, kwargs))
    try:
        rect_pattern(*args, **kwargs)
        print("Passed !\n")
    except Exception as e:
        print(e)
        print(">>> Failed ! <<<\n")

def test_rounded(name, *args, **kwargs):
    print("{}: {}, {}".format(name, args, kwargs))
    try:
        rounded_pattern(*args, **kwargs)
        print("Passed !\n")
    except Exception as e:
        print(e)
        print(">>> Failed ! <<<\n")

# # Automatic part
#
# test_gear("full test", 10, 30, 10)
# test_gear("helical test", 10, 30, 10, helix_angle = radians(20))
# test_gear("chamfer test", 10, 30, 10, chamfer = True)
# mesh = gear(10, 30, 10, chamfer = True)
# # show([mesh.outlines()], options={"display_points":True})
# # edges = mesh.outlines().edges
#
# # from madcad.mesh import edgekey, facekeyo
# # print(len(edges), len({edgekey(*e)  for e in edges}))
# # print(len({facekeyo(*f) for f in mesh.faces}), len(mesh.faces))
# show([mesh.outlines(), mesh], options = {"display_wire": True})
# test_gear("circle test", 10, 30, 10, pattern = "circle")
# test_gear("rect test", 10, 30, 10, pattern = "rect")
# test_gear("rounded test", 10, 30, 10, pattern = "rounded")
# test_gear("hub test", 10, 30, 10, bore_radius = 5)
#
# # Custom part
#
# test_gear("only bore test" ,10, 30, 10, bore_radius = 5, hub_height = 0)
# test_gear("height hub test" ,10, 30, 10, bore_radius = 5, hub_height = 10)
# test_gear("circle spec height test", 10, 30, 10, pattern = "circle", int_height = 2.5)
# test_gear("rect spec height test", 10, 30, 10, pattern = "rect", int_height = 2.5)
# test_gear("rounded spec height test", 10, 30, 10, pattern = "rounded", int_height = 2.5)
#
# # Different values
#
# test_gear("gearprofile values test", 10, 20, 15)
# test_gear("gearprofile values test", 15, 25, 15)
# test_gear("gearprofile values test", 1.2, 5, 2)
# test_gear("gearprofile values test", 2, 10, 2)
#
# test_gear("gearexterior values test", 10, 20, 15, chamfer = True)
# test_gear("gearexterior values test", 10, 20, 15, helix_angle = radians(20))
# test_gear("gearexterior values test", 15, 25, 15, chamfer = True)
# test_gear("gearexterior values test", 15, 25, 15, helix_angle = radians(22))
# test_gear("gearexterior values test", 1.2, 5, 2, chamfer = True)
# test_gear("gearexterior values test", 1.2, 5, 2, helix_angle = radians(20))
# test_gear("gearexterior values test", 2, 10, 2, chamfer = True)
# test_gear("gearexterior values test", 2, 10, 2, helix_angle = radians(30))
#
# # full pattern tests
# test_full("test full pattern", 10, 5, 5, int_height = 2)
# test_full("test full pattern", 20, 5, 15, int_height = 5)
# test_full("test full pattern", 60.5, 52, 1, int_height = 0)
# test_full("test full pattern", 15, 5, 15, int_height = 0)
#
# # circle pattern tests
# test_circle("test circle pattern", 10, 5, 5, int_height = 2)
# test_circle("test circle pattern", 20, 5, 15, int_height = 5, ratio = 1.5)
# test_circle("test circle pattern", 60.5, 52, 1, int_height = 0, ratio = 0.5)
# test_circle("test circle pattern", 15, 5, 15, int_height = 0)
# test_circle("test circle pattern", 30, 2, 15, int_height = 0, r_int = 5, r_ext = 15)
# test_circle("test circle pattern", 60, 5, 15, int_height = 0, r_int = 5, r_ext = 25.5)
# test_circle("test circle pattern", 5, 0.2, 15, int_height = 0, r_int = 0.2, r_ext = 3)
# test_circle("test circle pattern", 15, 5, 15, int_height = 5, n_circles = 6)
# test_circle("test circle pattern", 15, 5, 15, int_height = 5, n_circles = 4)
# test_circle("test circle pattern", 15, 5, 15, int_height = 5, n_circles = 3)
# test_circle("test circle pattern", 15, 5, 15, int_height = 5, n_circles = 2)

# circle pattern tests
# test_rect("test rect pattern", 10, 5, 5, int_height = 2)
# test_rect("test rect pattern", 20, 5, 15, int_height = 5, ratio = 1.5)
# test_rect("test rect pattern", 60.5, 52, 1, int_height = 0, ratio = 0.5)
# test_rect("test rect pattern", 15, 5, 15, int_height = 0)
# test_rect("test rect pattern", 30, 2, 15, int_height = 0, r_int = 5, r_ext = 15)
# test_rect("test rect pattern", 60, 5, 15, int_height = 0, r_int = 5, r_ext = 25.5)
# test_rect("test rect pattern", 5, 0.2, 15, int_height = 0, r_int = 0.2, r_ext = 3)
# test_rect("test rect pattern", 15, 5, 15, int_height = 5, n_patterns = 6)
# test_rect("test rect pattern", 15, 5, 15, int_height = 5, n_patterns = 4)
# test_rect("test rect pattern", 15, 5, 15, int_height = 5, n_patterns = 3)
# test_rect("test rect pattern", 15, 5, 15, int_height = 5, n_patterns = 2)
