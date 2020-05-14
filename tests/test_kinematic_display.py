from madcad.mathutils import vec3
from madcad.kinematic import *

s1 = Solid()
s2 = Solid(pose=(vec3(0,0,0.5), vec3(1,0,0)))
s3 = Solid()
A = vec3(2,0,0)
B = vec3(0,2,0)
C = vec3(0,-1,1)
x = vec3(1,0,0)
y = vec3(0,1,0)
csts = [Pivot(s1,s2, (A,x)), Pivot(s1,s2, (B,y)), Plane(s2,s3, (C,y), position=(C,C))]
sc1 = mkwiredisplay(s1, csts, 1, color=(1, 1, 1))
sc2 = mkwiredisplay(s2, csts, 1)
sc3 = mkwiredisplay(s3, csts, 1)

import sys
from PyQt5.QtWidgets import QApplication
from madcad.view import Scene

app = QApplication(sys.argv)
scn = Scene()
scn.add(sc1)
scn.add(sc2)
scn.add(sc3)

scn.show()
sys.exit(app.exec())
