from view import Scene
import numpy.core as np
from mathutils import vec3, mat3, vec4, dot, cross, inverse, normalize, length, determinant, NUMPREC, COMPREC


class Mesh:
	def __init__(self, points, faces):
		self.points = points
		self.faces = faces
	
	def facepoints(self, index):
		f = self.faces[index]
		return self.points[f[0]], self.points[f[1]], self.points[f[2]]
	
	def translate(self, displt):
		for i in range(len(self.points)):
			self.points[i] += displt
	
	def transform(self, mat):
		for i in range(len(self.points)):
			self.points[i] = vec3(mat * vec4(self.points[i], 1))
		
	def facenormals(self):
		facenormals = []
		for face in self.faces:
			p0 = self.points[face[0]]
			e1 = self.points[face[1]] - p0
			e2 = self.points[face[2]] - p0
			facenormals.append(normalize(cross(e1, e2)))
		return facenormals
	
	def display(self, scene):
		import view
		fn = np.array([tuple(p) for p in self.facenormals()])
		points = np.array([tuple(p) for p in self.points], dtype=np.float32)
		lines = []
		for i in range(0, 3*len(self.faces), 3):
			lines.append((i, i+1))
			lines.append((i+1, i+2))
			lines.append((i, i+2))
		
		return view.SolidDisplay(scene,
			points[np.array(self.faces, dtype=np.uint32)].reshape((len(self.faces)*3,3)),
			np.hstack((fn, fn, fn)).reshape((len(self.faces)*3,3)),
			faces = np.array(range(3*len(self.faces)), dtype=np.uint32).reshape(len(self.faces),3),
			lines = np.array(lines, dtype=np.uint32),
			)
		

if __name__ == '__main__':
	from nprint import nprint
	from mathutils import vec4, mat4, rotate
	
	m1 = Mesh(
		[
			vec3(1.0, -1.0, -1.0),
            vec3(1.0, -1.0, 1.0),
            vec3(-1.0, -1.0, 1.0),
            vec3(-1.0, -1.0, -1.0),
            vec3(1.0, 1.0, -1.0),
            vec3(1.0, 1.0, 1.0),
            vec3(-1.0, 1.0, 1.0),
            vec3(-1.0, 1.0, -1.0)],
		[
            (0, 1, 2),
            (0, 2, 3),
            (4, 7, 6),
            (4, 6, 5),
            (0, 4, 5),
            (0, 5, 1),
            (1, 5, 6),
            (1, 6, 2),
            (2, 6, 7),
            (2, 7, 3),
            (4, 0, 3),
            (4, 3, 7)],
		)
		
	import view, text
	import sys
	from PyQt5 import Qt
	from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, QGraphicsScene, QGraphicsItem, QGraphicsView, QStackedLayout, QOpenGLWidget
	from PyQt5.QtCore import QRectF
	from PyQt5.QtGui import QPainter, QColor, QPen, QFont
	
	# test to draw text on top of 3D view
	class BetterScene(view.Scene):
		def render(self):
			painter = QPainter(self)
			painter.beginNativePainting()
			self.screen = self.ctx.detect_framebuffer()
			super().render()
			painter.endNativePainting()
			#painter.setPen(QColor(1, 1, 1))
			#painter.setFont(QFont('Sans', 10))
			#print('before text')
			painter.drawText(0, 0, self.width(), self.height(), 0, "Coucou\nbonjour a tous")
			#print('after text')
			painter.end()
			#print('end')
	
	app = QApplication(sys.argv)
	
	main = scn3D = BetterScene()
	
	#main = scn3D = view.Scene()
	
	scn3D.add(m1)
	
	main.show()
	sys.exit(app.exec())
	
