![madcad-logo](docs/logo.png)

# Py-MADCAD

*>>> it's time to throw parametric softwares out !*

Simple yet powerful CAD (Computer Aided Design) library, written with Python.

- [Installation](https://pymadcad.readthedocs.io/en/latest/installation.html)
- [Documentation](https://pymadcad.readthedocs.io/en/latest/?)


[![Documentation Status](https://readthedocs.org/projects/pymadcad/badge/?version=latest)](https://pymadcad.readthedocs.io/en/latest/?badge=latest)

## Features

- surface generation (3D sketch primitives, extrusion, revolution, ...)
- fast boolean operations
- common mesh file format import/export
- kinematic manipulation
- implicit geometry definition through the constraint/solver system
- objects display with high-quality graphics

## Sample usage

```python
from madcad import *

# define points
O = vec3(0)
A = vec3(2,0,0)
B = vec3(1,2,0)
C = vec3(0,2,0)

# create a list of primitives
line = [
	Segment(O, A),          
	ArcThrough(A, B, C),
	Segment(C,O),           
	]

# create and solve constraints
solve([
		Tangent(line[0], line[1], A),   
		Tangent(line[1], line[2], C),   
		Radius(line[1], 1.5),           
	], fixed=[O])

# generate surfaces
part = extrusion(vec3(0,0,1), web(line))

# display in a 3D scene
quickdisplay([part])
```

The result will be this window
![example-window](docs/screenshots/readme-example.png)

## About

MADCAD is born from the idea that the current approach of parametric CADs for mechanical engineering is not the best possible. This library is part of a project targeting the best possible mechanical design tool for both engineers and handymen. See the 
[comparison](https://pymadcad.readthedocs.io/en/latest/concepts.html#comparison-with-existing-cad-softwares) 
for more details.

## License   ![LGPL logo](https://www.gnu.org/graphics/lgplv3-88x31.png)

Copyright 2019-2020 Yves Dejonghe <yves.dejonghe.y@gmail.com>

This library is distributed under the LGPL-v3 license. A copy of that license is provided with this software.


![python logo](https://www.python.org/static/img/python-logo.png)
![cython logo](https://avatars3.githubusercontent.com/u/486082?s=200&v=4)
![moderngl logo](https://avatars2.githubusercontent.com/u/26459939?s=200&v=4)
![qt logo](https://cdn2.hubspot.net/hubfs/149513/Qt2017/qt_logo_with_text_green_rgb.png)