# Read and write files

As mentioned in [installation](../installation.md), you must have installed some dependences to be able to read/write `.stl`, `.ply` and `.obj` files.
Reading and writing files is simple:

```python
from madcad import *
s = screw(10, 20) # s is a `Solid`
# Write
io.write(s["part"], "screw.stl")
# Read
read_mesh = io.read("screw.stl")
read_mesh.mergeclose()
show([read_mesh])
```
