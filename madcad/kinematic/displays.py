# This file is part of pymadcad,  distributed under license LGPL v3
__all__ = ['ChainManip', 'KinematicManip', 'scale_solid', 'world_solid']

from dataclasses import dataclass
import numpy as np
import numpy.linalg as la
import moderngl as mgl
from PyQt5.QtCore import Qt, QEvent	

from ..common import resourcedir
from ..mathutils import *
from ..mesh import Mesh, Web, Wire, striplist, distance2_pm, typedlist_to_numpy
from .. import settings
from .. import rendering
from .. import scheme
from .. import nprint
from ..displays import BoxDisplay
from ..rendering import Group, displayable
from ..scheme import Scheme, halo_screen
from ..generation import revolution
from ..mesh import web
from .solver import Chain, Kinematic, KinematicError, regularize_grad, structure_state, flatten_state


class SolidDisplay(Group):
    ''' Movable `Group` for the rendering pipeline '''
    def __init__(self, scene, solid):
        super().__init__(scene, solid.content, local=solid.pose)
    
    def update(self, scene, solid):
        if not isinstance(solid, Solid):	return
        super().update(scene, solid.content)
        self.local = fmat4(solid.pose)
        return True
    
    def stack(self, scene):
        for key,display in self.displays.items():
            if key == 'annotations' and not scene.options['display_annotations'] and not self.selected:
                continue
            for sub,target,priority,func in display.stack(scene):
                yield ((key, *sub), target, priority, func)


class ChainManip(Group):
	''' object to display and interact with a robot in the 3d view

		Attributes:
			chain:  the kinematic chain this display is rendering
			pose:   joints poses in the last rendered frame
			parts:  solids poses in the last rendered frame
			toolcenter (vec3):   current end-solid rotation point in rotation mode, relative to last solid
	'''
	min_colinearity = 1e-2
	max_increment = 0.3
	prec = 1e-6
	
	def __init__(self, scene, chain, pose=None, toolcenter=None):
		super().__init__(scene)
		self.chain = chain
		self.toolcenter = toolcenter or chain.joints[-1].position[1]
		self.pose = pose or chain.default
		self.parts = self.chain.parts(self.pose)
		
		if chain.content:
			for key, solid in enumerate(chain.content):
				if displayable(solid):
					self.displays[key] = scene.display(solid)
		
		scheme, index = kinematic_scheme(chain.joints)
		scheme += kinematic_toolcenter(self.toolcenter)
		self.displays['scheme'] = scene.display(scheme)
		
	# TODO: add content update method
	
	def stack(self, scene):
		''' rendering stack requested by the madcad rendering system '''
		yield ((), 'screen', -1, self.place_solids)
		yield from super().stack(scene)

	def place_solids(self, view):
		world = self.world * self.local
		
		index = {}
		for i, joint in enumerate(self.chain.joints):
			solid = joint.solids[-1]
			index[solid] = i+1
			if display := self.displays.get(solid):
				display.word = word * fmat4(self.parts[i])
		
		for space in self.displays['scheme'].spacegens:
			if isinstance(space, (world_solid, scale_solid)) and space.solid in index:
				space.pose = world * fmat4(self.parts[index[space.solid]])
			elif isinstance(space, scheme.halo_screen):
				if view.scene.options['kinematic_manipulation'] == 'rotate':
					space.position = fvec3(self.parts[-1] * self.toolcenter)
				else:
					space.position = fvec3(nan)

	def control(self, view, key, sub, evt):
		''' user event manager, optional part of the madcad rendering system '''
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
			evt.accept()
			
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.RightButton:
			evt.accept()
			self.panel.setParent(view)
			self.panel.move(evt.pos())
			self.panel.show()
		
		if evt.type() == QEvent.MouseMove and evt.buttons() & Qt.LeftButton:
			evt.accept()
			# put the tool into the view, to handle events
			# TODO: allow changing mode during move
			if sub == ('scheme', index_toolcenter):
				view.tool.append(rendering.Tool(self.move_tool, view, sub, evt))
			else:
				view.tool.append(rendering.Tool(getattr(self, 'move_'+view.scene.options['kinematic_manipulation']), view, sub, evt))
				
	def move_tool(self, dispatcher, view, sub, evt):
		place = mat4(self.world) * self.parts[-1]
		init = place * vec3(self.toolcenter)
		offset = init - view.ptfrom(evt.pos(), init)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break
			
			self.toolcenter = affineInverse(place) * (view.ptfrom(evt.pos(), init) + offset)
	
	def move_joint(self, dispatcher, view, sub, evt):
		
		# find the clicked solid, and joint controled
		if sub[0] == 'scheme':	solid = sub[1]
		else:					solid = sub[0]
		
		# get 3d location of mouse
		click = view.ptat(view.somenear(evt.pos()))
		joint = max(0, solid-1)
		anchor = affineInverse(mat4(self.world * self.local) * self.parts[joint+1]) * vec4(click,1)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local) * self.parts[joint]
				direct = affineInverse(self.parts[joint]) * self.parts[joint+1]
				target = qtpos(evt.pos(), view)
				current = model * (direct * anchor)
				move = target - current.xy / current.w

				# solid move directions that can be acheived with this joint
				jac = regularize_grad(self.chain.joints[joint].grad(self.pose[joint]))
				# gradient of the screen position
				jac = np.stack([
					(model * (grad * anchor)).xy / current.w
					for grad in jac])
				# colinearity between the desired move and the gradient directions. it avoids the mechanisme to burst when close to a singularity and also when the mouse move is big
				colinearity = min(1, sum(
							dot(grad, move)**2 / (length2(grad) + length2(move) + self.prec)
							for grad in jac) / self.min_colinearity)
				# combination of the closest directions to the mouse position
				increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ (move * colinearity))
				self.pose[joint] = (np.asarray(self.pose[joint]) 
										+ increment * min(1, self.max_increment / np.abs(increment).max())
										) .clip(*self.chain.joints[joint].bounds)
				if hasattr(self.chain.joints[joint], 'normalize'):
					self.pose[joint] = self.chain.joints[joint].normalize(self.pose[joint])
				self.parts = self.chain.parts(self.pose)

	def move_translate(self, dispatcher, view, sub, evt):
		# translate the tool
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[-1]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[-1]
				jac = self.chain.grad(self.pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_anchor = model * solid * anchor
				target_anchor = qtpos(evt.pos(), view)
				
				move = np.concatenate([
					target_anchor - current_anchor.xy / current_anchor.w,
					np.ravel(mat3(init_solid - solid)),
					])
				jac = np.stack([
					np.concatenate([
						(model * grad * anchor).xy / current_anchor.w, 
						np.ravel(mat3(grad)), 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ (move * colinearity))
				
				self.pose = structure_state((
								flatten_state(self.pose)
								+ increment * min(1, self.max_increment / np.abs(increment).max())
								),
								self.pose)
				for i, joint in enumerate(self.chain.joints):
					if hasattr(joint, 'normalize'):
						self.pose[i] = joint.normalize(self.pose[i])
				self.parts = self.chain.parts(self.pose)

	def move_rotate(self, dispatcher, view, sub, evt):
		# translate the tool
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[-1]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		tool = vec4(self.toolcenter,1)
		init_tool = solid * tool
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[-1]
				jac = self.chain.grad(self.pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_tool = model * solid * tool
				target_anchor = qtpos(evt.pos(), view)
				target_tool = model * init_tool
				
				move = np.concatenate([
					(init_tool - solid * tool).xyz,
					(target_anchor - target_tool.xy/target_tool.w) 
						- (model * solid * normalize(anchor - tool)).xy / current_tool.w,
					])
				jac = np.stack([
					np.concatenate([
						(grad * tool).xyz, 
						(model * grad * normalize(anchor - tool)).xy / current_tool.w, 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ (move * colinearity))
				# increment = la.pinv(jac @ jac.transpose()) @ (jac @ (move * colinearity))
				
				self.pose = (self.pose 
								+ increment * min(1, self.max_increment / np.abs(increment).max())
								).clip(*self.chain.bounds)
				self.parts = self.chain.parts(self.pose)


		
class KinematicManip(Group):
	''' 
        Display that holds a kinematic structure and allows the user to move it
        
        Attributes:
            kinemaitc:  the kinematic this display is rendering
            pose:       joints poses in the last rendered frame
            parts:      solids poses in the last rendered frame
            toolcenter:  current solid rotation point in rotation mode, relative to kinematic ground
	'''
	min_colinearity = 1e-2
	max_increment = 0.3
	prec = 1e-6
	
	def __init__(self, scene, kinematic, pose=None, toolcenter=None):
		super().__init__(scene)
		self.kinematic = kinematic
		self.toolcenter = toolcenter or vec3(0)
		self.pose = self.kinematic.solve(close=pose or self.kinematic.default, maxiter=10000, precision=1e-2, strict=1e-1)
		self.parts = self.kinematic.parts(self.pose, precision=1e-1)
		
		if self.kinematic.content:
			for key, solid in self.kinematic.content.items():
				if displayable(solid):
					self.displays[key] = scene.display(solid)
		
		scheme, self.index = kinematic_scheme(kinematic.joints)
		scheme += kinematic_toolcenter(self.toolcenter)
		self.displays['scheme'] = scene.display(scheme)
		
	def stack(self, scene):
		''' rendering stack requested by the madcad rendering system '''
		yield ((), 'screen', -1, self.place_solids)
		yield from super().stack(scene)

	def place_solids(self, view):
		world = self.world * self.local
		
		for key in self.displays:
			if key in self.parts:
				self.displays[key].world = world * self.parts[key]
		
		for space in self.displays['scheme'].spacegens:
			if isinstance(space, (world_solid, scale_solid)) and space.solid in self.parts:
				space.pose = world * fmat4(self.parts[space.solid])
			elif isinstance(space, scheme.halo_screen):
				if view.scene.options['kinematic_manipulation'] == 'rotate':
					space.position = fvec3(self.toolcenter)
				else:
					space.position = fvec3(nan)

	def control(self, view, key, sub, evt):
		''' user event manager, optional part of the madcad rendering system '''
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.LeftButton:
			evt.accept()
			
		if evt.type() == QEvent.MouseButtonPress and evt.button() == Qt.RightButton:
			evt.accept()
			self.panel.setParent(view)
			self.panel.move(evt.pos())
			self.panel.show()
		
		if evt.type() == QEvent.MouseMove and evt.buttons() & Qt.LeftButton:
			evt.accept()
			# put the tool into the view, to handle events
			# TODO: allow changing mode during move
			if sub == ('scheme', index_toolcenter):
				view.tool.append(rendering.Tool(self.move_tool, view, sub, evt))
			else:
				view.tool.append(rendering.Tool(getattr(self, 'move_'+view.scene.options['kinematic_manipulation']), view, sub, evt))
	
	def move_tool(self, dispatcher, view, sub, evt):
		place = mat4(self.world)
		init = place * vec3(self.toolcenter)
		offset = init - view.ptfrom(evt.pos(), init)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break
			
			self.toolcenter = affineInverse(place) * (view.ptfrom(evt.pos(), init) + offset)
	
	def move_joint(self, dispatcher, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == 0:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		pose = (
			*self.pose, 
			kinematic.joints[-1].inverse(self.parts[moved]),
			)
		# constants during translation
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[moved]
				jac = kinematic.grad(pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_anchor = model * solid * anchor
				target_anchor = qtpos(evt.pos(), view)
				
				move = np.asarray(target_anchor - current_anchor.xy / current_anchor.w)
				jac = np.stack([
					np.asarray((model * grad * anchor).xy / current_anchor.w)
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ (move * colinearity))
				
				try:
					self.pose = self.kinematic.solve(close=structure_state(
						flatten_state(self.pose) 
						+ increment * min(1, self.max_increment / np.abs(increment).max()),
						self.pose), maxiter=20, precision=1e-3, strict=1e-1)
				except KinematicError:
					pass
				else:
					self.parts = self.kinematic.parts(self.pose, precision=1.)
					pose = (
						*self.pose, 
						kinematic.joints[-1].inverse(self.parts[moved]),
						)
	
	def move_joint(self, dispatcher, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == 0:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		pose = (
			*self.pose, 
			kinematic.joints[-1].inverse(self.parts[moved]),
			)
		# constants during translation
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				freejac = kinematic.grad(pose)
				target_anchor = qtpos(evt.pos(), view)
				
				for i in range(10):
					solid = self.parts[moved]
					model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
					current_anchor = model * solid * anchor
					
					move = np.concatenate([
						np.asarray(target_anchor - current_anchor.xy / current_anchor.w),
						-self.kinematic.cost_residuals(self.pose),
						])
					jac = np.hstack([
						np.stack([
							np.asarray((model * grad * anchor).xy / current_anchor.w)
							for grad in freejac]),
						self.kinematic.cost_jacobian(self.pose).transpose(),
						])
					colinearity = min(1, max(
						np.dot(grad[:2], move[:2])**2 / (normsq(grad[:2]) * normsq(move[:2]) + self.prec**4)
						for grad in jac) / 1)
					move[:2] *= 1e-4 / max(1e-4, (move[2:]**2).max())
					move[:2] *= colinearity
					increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ move)
					
					self.pose = structure_state(
						flatten_state(self.pose)
						+ increment * min(1, self.max_increment / np.abs(increment).max()),
						self.pose)
					
					for i, joint in enumerate(self.kinematic.joints):
						if hasattr(joint, 'normalize'):
							self.pose[i] = joint.normalize(self.pose[i])
					
					self.parts = self.kinematic.parts(self.pose, precision=10.)
				pose = (
					*self.pose, 
					kinematic.joints[-1].inverse(self.parts[moved]),
					)
	
	def move_translate(self, dispatcher, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == 0:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		pose = (
			*self.pose, 
			kinematic.joints[-1].inverse(self.parts[moved]),
			)
		# constants during translation
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[moved]
				jac = kinematic.grad(pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_anchor = model * solid * anchor
				target_anchor = qtpos(evt.pos(), view)
				
				move = np.concatenate([
					target_anchor - current_anchor.xy / current_anchor.w,
					np.ravel(mat3(init_solid - solid)),
					])
				jac = np.stack([
					np.concatenate([
						(model * grad * anchor).xy / current_anchor.w, 
						np.ravel(mat3(grad)), 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ (move * colinearity))
				
				# nprint('increment', increment, self.pose)
				
				self.pose = self.kinematic.solve(close=structure_state(
					flatten_state(self.pose) 
					+ increment.clip(-self.max_increment, self.max_increment),
					self.pose))
				# self.pose = structure_state(
				# 	flatten_state(self.pose) 
				# 	+ increment.clip(-self.max_increment, self.max_increment),
				# 	self.pose)
				self.parts = self.kinematic.parts(self.pose)
				pose = (
					*self.pose, 
					kinematic.joints[-1].inverse(self.parts[moved]),
					)
				
	def move_rotate(self, dispatcher, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == 0:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		pose = (
			*self.pose, 
			kinematic.joints[-1].inverse(self.parts[moved]),
			)
		# constants during translation
		clicked = view.ptat(view.somenear(evt.pos()))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world * self.local) * solid) * vec4(clicked,1)
		init_tool = vec4(self.toolcenter, 1)
		tool = affineInverse(solid) * vec4(self.toolcenter, 1)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.MouseMove, QEvent.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.LeftButton):
					break

				solid = self.parts[moved]
				jac = kinematic.grad(pose)
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world * self.local)
				current_tool = model * solid * tool
				target_anchor = qtpos(evt.pos(), view)
				target_tool = model * init_tool
				
				move = np.concatenate([
					(init_tool - solid * tool).xyz,
					(target_anchor - target_tool.xy/target_tool.w) 
						- (model * solid * normalize(anchor - tool)).xy / current_tool.w,
					])
				jac = np.stack([
					np.concatenate([
						(grad * tool).xyz, 
						(model * grad * normalize(anchor - tool)).xy / current_tool.w, 
						])
					for grad in jac])
				colinearity = min(1, sum(
					np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
					for grad in jac) / self.min_colinearity)
				
				increment = la.solve(jac @ jac.transpose() + np.eye(len(jac))*self.prec, jac @ (move * colinearity))
				
				# nprint('increment', increment, self.pose)
				
				self.pose = self.kinematic.solve(close=structure_state(
					flatten_state(self.pose) 
					+ increment.clip(-self.max_increment, self.max_increment),
					self.pose))
				# self.pose = structure_state(
				# 	flatten_state(self.pose) 
				# 	+ increment.clip(-self.max_increment, self.max_increment),
				# 	self.pose)
				self.parts = self.kinematic.parts(self.pose)
				pose = (
					*self.pose, 
					kinematic.joints[-1].inverse(self.parts[moved]),
					)




@dataclass
class scale_solid:
	''' scheme space scaling around a point in a given solid '''
	solid: object
	center: fvec3
	size: float
	pose: fmat4 = fmat4()
	
	def __call__(self, view):
		m = view.uniforms['view'] * view.uniforms['world'] * self.pose
		e = view.uniforms['proj'] * fvec4(1,1,(m*self.center).z,1)
		e /= e[3]
		return m * translate(self.center) * scale(fvec3(min(self.size, 2 / (e[1]*view.target.height))))

@dataclass
class world_solid:
	solid: object
	pose: fmat4 = fmat4()
	
	def __call__(self, view):
		return view.uniforms['view'] * view.uniforms['world'] * self.pose



index_toolcenter = 10000
	
def kinematic_toolcenter(toolcenter):
	''' create a scheme for drawing the toolcenter in kinematic manipulation '''
	color = settings.display['annotation_color']
	angle = 2
	radius = 60
	size = 3
	tend = -sin(angle)*X + cos(angle)*Y
	rend = cos(angle)*X + sin(angle)*Y
	sch = Scheme(
		space=halo_screen(fvec3(toolcenter)), 
		layer=1e-4,
		shader='line',
		track=index_toolcenter,
		)
	sch.add([size*cos(t)*X + size*sin(t)*Y   for t in linrange(0, 2*pi, step=0.2)], color=fvec4(color,1))
	sch.add([radius*cos(t)*X + radius*sin(t)*Y  for t in linrange(0, angle, step=0.1)], color=fvec4(color,1))
	sch.add([radius*cos(t)*X + radius*sin(t)*Y  for t in linrange(angle, 2*pi, step=0.1)], color=fvec4(color,0.2))
	sch.add(
		revolution(2*pi, (radius*rend, tend), web([radius*rend, radius*rend-14*tend-3.6*rend]), resolution=('div', 8)), 
		shader='fill',
		color=fvec4(color,0.8),
		)
	return sch

def kinematic_scheme(joints) -> '(Scheme, index)':
	''' create a kinematic scheme for the given joints '''
	index = {}
	centers = {}
	for joint in joints:
		if hasattr(joint, 'position'):
			for solid, position in zip(joint.solids, joint.position):
				centers[solid] = centers.get(solid, 0) + vec4(position, 1)
				index[solid] = index.get(solid, len(index))
	for solid, center in centers.items():
		centers[solid] = center.xyz / center.w  if center.w > 1 else None
	
	nprint('index', index)
	scheme = Scheme()
	for joint in joints:
		if hasattr(joint, 'position'):
			size = sum(
				vec2(distance(position, centers[solid]), 1)
				for solid, position in zip(joint.solids, joint.position)
				if centers[solid])
			size = 1.5*size.x / size.y
			
			sch = joint.scheme(index, size, centers[joint.solids[0]], centers[joint.solids[-1]])
			if sch is not NotImplemented:
				scheme += sch
	
	return scheme, {v: k  for k, v in index.items()}
		


def qtpos(qtpos, view):
	''' convert qt position in the widget to opengl screen coords in range (-1, 1) '''
	return vec2(
		+ (qtpos.x()/view.width() *2 -1),
		- (qtpos.y()/view.height() *2 -1),
		)

def normsq(x):
	x = x.ravel()
	return np.dot(x,x)
