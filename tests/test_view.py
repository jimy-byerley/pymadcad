import sys

from PyQt6.QtCore import QCoreApplication, Qt
from PyQt6.QtWidgets import QApplication

from madcad import Box, brick, settings
from madcad.rendering import *

scn = Scene({
	'brick': brick(Box(width=vec3(1))),
	})

app = QApplication(sys.argv)
settings.use_qt_colors()
view = View(scn)
view.show()
sys.exit(app.exec())
