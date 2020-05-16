from madcad.mathutils import vec3, normalize
from madcad.kinematic import *

s0 = Solid()
s1 = Solid()
s2 = Solid(pose=(vec3(2,0,1.5), 2*vec3(1,1,0)))
s3 = Solid()
s4 = Solid()
s5 = Solid()
csts = [
	Plane(s0,s1, (vec3(0), vec3(0,0,1))),  
	Pivot(s1,s2, (vec3(0,0,1), vec3(1,0,0)), (vec3(0,0,-1), normalize(vec3(1,1,0)))),
	#Pivot(s2,s3, (vec3(0,0,2), normalize(vec3(1,1,0))), (vec3(0,0,0), vec3(1,0,0))),
	
	#Pivot(s1,s2, (vec3(0,0,1), vec3(1,0,0)), (vec3(0,0,-1), normalize(vec3(1,0,0)))),
	Pivot(s2,s3, (vec3(0,0,2), normalize(vec3(1,0,0))), (vec3(0,0,0), vec3(1,0,0))),
	Pivot(s3,s4, (vec3(0,0,2), normalize(vec3(1,0,0))), (vec3(0,0,0), vec3(1,0,0))),
	Pivot(s4,s5, (vec3(0,0,2), normalize(vec3(1,0,0))), (vec3(0,0,0), vec3(1,0,0))),
	]
	
import sys
from PyQt5.QtCore import Qt, QCoreApplication
from PyQt5.QtWidgets import QApplication
from madcad.view import Scene
from madcad.text import Text

QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
app = QApplication(sys.argv)
scn = Scene()
s0.visuals.append(mkwiredisplay(s0, csts))
s1.visuals.append(mkwiredisplay(s1, csts, color=(1,0.4,0.2)))
s2.visuals.append(mkwiredisplay(s2, csts))
s3.visuals.append(mkwiredisplay(s3, csts))
s4.visuals.append(mkwiredisplay(s4, csts))
s5.visuals.append(mkwiredisplay(s5, csts))
s5.visuals.append(Text(vec3(0,0.6,0), 'tool'))
#display_mechanism(scn, csts, s0)
scn.add(Kinemanip(scn, csts, s0))
scn.show()
sys.exit(app.exec())
