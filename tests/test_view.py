from madcad import brick, Box, settings
from madcad.rendering import *

import sys
from PyQt5.QtCore import Qt, QCoreApplication
from PyQt5.QtWidgets import QApplication
	
scn = Scene({
	'brick': brick(Box(width=vec3(1))),
	})

app = QApplication(sys.argv)
settings.use_qt_colors()
view = View(scn)
view.show()
sys.exit(app.exec())
