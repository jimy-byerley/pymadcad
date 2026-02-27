# This file is part of pymadcad,  distributed under license LGPL v3
from __future__ import annotations
import warnings
from dataclasses import dataclass
import numpy as np
import numpy.linalg as la

from ..mathutils import (
		vec2, vec3, vec4, X, Y, Axis, linrange, fmat4, fvec3, affineInverse,
		mat3, mat4, dot, length2, distance, fvec4, scale, translate, normalize,
		dmat3x2, nan, inf, cos, sin, pi
		)
from .. import settings
from .. import scheme
from ..rendering import Group, receiver
from ..scheme import Scheme, halo_screen
from ..generation import revolution
from ..mesh import web
from .solver import Kinematic, KinematicError, regularize_grad, structure_state, flatten_state
from ..qt import Qt, QEvent, QTimer

__all__ = ['ChainManip', 'KinematicManip', 'scale_solid', 'world_solid']


		
			
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
	maxiter = 3
	prec = 1e-6
	
	def __init__(self, scene, chain, pose=None, toolcenter=None):
		super().__init__(scene)
		self.chain = chain
		self.toolcenter = toolcenter or chain.joints[-1].position[1]
		self.pose = pose or chain.default
		self.parts = self.chain.parts(self.pose)
		self.defered = DeferedSolving()
		
		if chain.content:
			for key, solid in enumerate(chain.content):
				if scene.displayable(solid):
					self.displays[key] = scene.display(solid)
		
		scheme, index = kinematic_scheme(chain.joints)
		scheme += kinematic_toolcenter(self.toolcenter)
		self.displays['scheme'] = scene.display(scheme)
		
	# TODO: add content update method
	
	def stack(self, scene):
		''' rendering stack requested by the madcad rendering system '''
		yield (self, 'screen', -1, self._place_solids)
		yield from super().stack(scene)

	def _place_solids(self, view):
		index = {}
		for i, joint in enumerate(self.chain.joints):
			solid = joint.solids[-1]
			index[solid] = i+1
			if display := self.displays.get(solid):
				display.world = self.world * fmat4(self.parts[i+1])
		
		for space in self.displays['scheme'].spacegens:
			if isinstance(space, (world_solid, scale_solid)) and space.solid in index:
				space.pose = self.world * fmat4(self.parts[index[space.solid]])
			elif isinstance(space, scheme.halo_screen):
				if view.scene.options['kinematic_manipulation'] == 'rotate':
					space.position = fvec3(self.parts[-1] * self.toolcenter)
				else:
					space.position = fvec3(nan)

	def control(self, view, key, sub, evt):
		''' user event manager, optional part of the madcad rendering system '''
		if evt.type() == QEvent.Type.MouseButtonPress and evt.button() == Qt.MouseButton.LeftButton:
			evt.accept()
		
		if evt.type() == QEvent.Type.MouseMove and evt.buttons() & Qt.MouseButton.LeftButton:
			evt.accept()
			# put the tool into the view, to handle events
			# TODO: allow changing mode during move
			if sub == ('scheme', index_toolcenter):
				generator = self._move_tool(view, sub, evt)
			else:
				generator = getattr(self, '_move_'+view.scene.options['kinematic_manipulation'])(view, sub, evt)
			view.callbacks.append(receiver(generator))
				
	def _move_tool(self, view, sub, evt):
		place = mat4(self.world) * self.parts[-1]
		init = place * vec3(self.toolcenter)
		offset = init - vec3(view.ptfrom(evt.pos(), init))
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break
			
			self.toolcenter = affineInverse(place) * (vec3(view.ptfrom(evt.pos(), init)) + offset)
	
	def _move_joint(self, view, sub, evt):
		
		# find the clicked solid, and joint controled
		if sub[0] == 'scheme':	solid = sub[1]
		else:					solid = sub[0]
		
		# get 3d location of mouse
		click = vec3(view.ptat(view.somenear(evt.pos())))
		joint = max(0, solid-1)
		anchor = affineInverse(mat4(self.world) * self.parts[joint+1]) * vec4(click,1)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break

				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world) * self.parts[joint]
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
				increment = la.lstsq(jac.transpose(), move * colinearity, 1e-6)[0]
				self.pose[joint] = self.chain.joints[joint].normalize(
										(np.asarray(self.pose[joint]) 
										+ increment * min(1, self.max_increment / np.abs(increment).max())
									) .clip(*self.chain.joints[joint].bounds))
				self.parts = self.chain.parts(self.pose)

	def _move(self, move, jac):
		''' newton method step for moving the mechanism
		'''
		colinearity = min(1, sum(
			np.dot(grad, move)**2 / (normsq(grad) + normsq(move) + self.prec)
			for grad in jac) / self.min_colinearity)
		
		increment = la.lstsq(jac.transpose(), move * colinearity, 1e-6)[0]
		
		self.pose = self.chain.normalize(structure_state((
						flatten_state(self.pose)
						+ increment * min(1, self.max_increment / np.abs(increment).max())
						),
						self.pose))
		self.parts = self.chain.parts(self.pose)

	def _move_translate(self, view, sub, evt):
		# translate the tool
		clicked = vec3(view.ptat(view.somenear(evt.pos())))
		solid = self.parts[-1]
		anchor = affineInverse(mat4(self.world) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break

				solid = self.parts[-1]
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world)
				current_anchor = model * solid * anchor
				target_anchor = qtpos(evt.pos(), view)
				
				def prepare(problem):
					problem.jac = self.chain.grad(self.pose)
				def solve(problem):
					self._move(
						move = np.concatenate([
							target_anchor - current_anchor.xy / current_anchor.w,
							np.ravel(mat3(init_solid - solid)),
							]),
						jac = np.stack([
							np.concatenate([
								(model * grad * anchor).xy / current_anchor.w, 
								np.ravel(mat3(grad)), 
								])
							for grad in problem.jac]),
						)
				self.defered.set(prepare, solve, self.maxiter)

	def _move_rotate(self, view, sub, evt):
		# translate the tool
		clicked = vec3(view.ptat(view.somenear(evt.pos())))
		solid = self.parts[-1]
		anchor = affineInverse(mat4(self.world) * solid) * vec4(clicked,1)
		tool = vec4(self.toolcenter,1)
		init_tool = solid * tool
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break

				solid = self.parts[-1]
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world)
				current_tool = model * solid * tool
				target_anchor = qtpos(evt.pos(), view)
				target_tool = model * init_tool
				
				def prepare(problem):
					problem.jac = self.chain.grad(self.pose)
				def solve(problem):
					self._move(
						move = np.concatenate([
							(init_tool - solid * tool).xyz,
							(target_anchor - target_tool.xy/target_tool.w) 
								- (model * solid * normalize(anchor - tool)).xy / current_tool.w,
							]),
						jac = np.stack([
							np.concatenate([
								(grad * tool).xyz, 
								(model * grad * normalize(anchor - tool)).xy / current_tool.w, 
								])
							for grad in problem.jac]),
						)
				self.defered.set(prepare, solve, self.maxiter)
		

		
class KinematicManip(Group):
	''' 
        Display that holds a kinematic structure and allows the user to move it
        
        Attributes:
            kinemaitc:  the kinematic this display is rendering
            pose:       joints poses in the last rendered frame
            parts:      solids poses in the last rendered frame
            toolcenter:  current solid rotation point in rotation mode, relative to kinematic ground
	'''
	max_increment = 0.3
	prec = 1e-6
	
	move_maxiter = 5
	stay_maxiter = 10000
	move_precision = 1e-4
	stay_precision = 1e-6
	tolerated_precision = 1e-1
	damping = 0.9
	
	def __init__(self, scene, kinematic, pose=None, toolcenter=None):
		super().__init__(scene)
		self.kinematic = kinematic
		self.defered = DeferedSolving()
		self.toolcenter = toolcenter or vec3(0)
		self.pose = pose or self.kinematic.default
		try:
			self.pose = self.kinematic.solve(
				close=pose or self.kinematic.default, 
				maxiter=self.stay_maxiter, 
				precision=self.move_precision, 
				strict=self.tolerated_precision,
				)
		except KinematicError:
			warnings.warn("could not solve kinematic {} for displaying, falling back to infinite tolerance".format(self))
			self.pose = self.kinematic.default
			self.tolerated_precision = inf
		self.parts = self.kinematic.parts(self.pose, precision=self.tolerated_precision)
		
		if self.kinematic.content:
			for key, solid in self.kinematic.content.items():
				if scene.displayable(solid):
					self.displays[key] = scene.display(solid)
		
		scheme, self.index = kinematic_scheme(kinematic.joints)
		scheme += kinematic_toolcenter(self.toolcenter)
		self.displays['scheme'] = scene.display(scheme)
		
	def stack(self, scene):
		''' rendering stack requested by the madcad rendering system '''
		self.prepare(scene)
		yield (self, 'screen', -1, self._place_solids)
		for key, display in self.displays.items():
			display.world = self.world
			display.key = (*self.key, key)
			if key == 'scheme' and not scene.options['display_annotations'] and not self.selected:
				continue
			yield from display.stack(scene)

	def _place_solids(self, view):
		for key in self.displays:
			if key in self.parts:
				self.displays[key].world = self.world * fmat4(self.parts[key])
		
		if not 'scheme' in self.displays:
			from pnprint import nprint
			nprint(self.displays)
		for space in self.displays['scheme'].spacegens:
			if isinstance(space, (world_solid, scale_solid)) and space.solid in self.parts:
				space.pose = fmat4(self.parts[space.solid])
			elif isinstance(space, scheme.halo_screen):
				if view.scene.options['kinematic_manipulation'] == 'rotate':
					space.position = fvec3(self.toolcenter)
				else:
					space.position = fvec3(nan)

	def control(self, view, key, sub, evt):
		''' user event manager, optional part of the madcad rendering system '''
		# give priority to sub elements
		disp = self.displays
		for i in range(1,len(sub)):
			disp = disp[sub[i-1]]
			disp.control(view, sub[:i], sub[i:], evt)
			if evt.isAccepted(): return
		
		if evt.type() == QEvent.Type.MouseButtonPress and evt.button() == Qt.MouseButton.LeftButton:
			# accept mouse pressing to tell we are interested in mouse movements
			evt.accept()
		
		if evt.type() == QEvent.Type.MouseMove and evt.buttons() & Qt.MouseButton.LeftButton:
			evt.accept()
			# put the tool into the view, to handle events
			# TODO: allow changing mode during move
			if sub == ('scheme', index_toolcenter):
				generator = self._move_tool(view, sub, evt)
			else:
				generator = getattr(self, '_move_'+view.scene.options['kinematic_manipulation'])(view, sub, evt)
			view.callbacks.append(receiver(generator))
	
	def _move_tool(self, view, sub, evt):
		place = mat4(self.world)
		init = place * vec3(self.toolcenter)
		offset = init - vec3(view.ptfrom(evt.pos(), init))
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break
			
			self.toolcenter = affineInverse(place) * (vec3(view.ptfrom(evt.pos(), init)) + offset)
					
	def _move_opt(self, optmove, optjac):
		'''
			newton method step for moving the mechanism
			
			Parameters:
				optmove:  the vector of the desired optional movement
				optjac:   the jacobian of the residuals for the desired optional movement
		'''
		costmove = -self.kinematic.cost_residuals(self.pose)
		# priority factor, lowering the optional move while the closed loop error is big
		priority = self.move_precision / max(self.move_precision, (costmove**2).max(initial=0))
		# colinearity factor, lowering the optional move while it doesn't match the mechanism degrees of freedom
		colinearity = min(1, max(
			np.dot(optgrad, optmove)**2 / (normsq(optgrad) * normsq(optmove) + self.prec**4)
			for optgrad in optjac))
		
		# complete problem to solve
		move = np.hstack([
			optmove * colinearity * priority, 
			costmove,
			])
		jac = np.hstack([
			optjac,
			self.kinematic.cost_jacobian(self.pose).transpose(),
			])
		increment = la.lstsq(jac.T, move, 1e-6)[0]
		# increment = la.solve(jac @ jac.T, jac @ move)
		# increment = la.inv(jac @ jac.T) @ jac @ move
		# assemble the new pose and normalize it
		newpose = self.kinematic.normalize(structure_state(
			flatten_state(self.pose)
			+ self.damping * increment,
			# + self.damping * increment * min(1, self.max_increment / np.abs(increment).max()),
			self.pose))
		
		# try to move
		try:
			self.parts = self.kinematic.parts(newpose, precision=self.tolerated_precision)
		except KinematicError as err:
			print(err)
			return
		self.pose = newpose
	
	def _move_joint(self, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == self.kinematic.ground:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		# constants during translation
		clicked = vec3(view.ptat(view.somenear(evt.pos())))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world) * solid) * vec4(clicked,1)
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break
				
				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world)
				target_anchor = qtpos(evt.pos(), view)
				def prepare(problem):
					problem.freejac = kinematic.grad((
						*self.pose, 
						kinematic.joints[-1].inverse(self.parts[moved]),
						))
					
				def solve(problem, target_anchor=target_anchor, model=model):
					solid = self.parts[moved]
					current_anchor = model * solid * anchor
					
					self._move_opt(
						optmove = np.asarray(
							# keep grasped position under mouse
							target_anchor - current_anchor.xy / current_anchor.w),
						optjac = np.stack([
							# keep grasped position under mouse
							np.asarray((model * grad * anchor).xy / current_anchor.w)
							for grad in problem.freejac]),
						)
					view.update()
				
				# process solving and move independently of events ticks to gain perf
				self.defered.set(prepare, solve, self.move_maxiter)
	
	def _move_translate(self, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == self.kinematic.ground:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		# constants during translation
		clicked = vec3(view.ptat(view.somenear(evt.pos())))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world) * solid) * vec4(clicked,1)
		init_solid = solid
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break

				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world)
				target_anchor = qtpos(evt.pos(), view)
				def prepare(problem):
					problem.jac = kinematic.grad((
						*self.pose, 
						kinematic.joints[-1].inverse(self.parts[moved]),
						))
					
				def solve(problem, model=model, target_anchor=target_anchor):
					solid = self.parts[moved]
					current_anchor = model * solid * anchor
				
					self._move_opt(
						optmove = np.concatenate([
							# move grasped point under mouse
							target_anchor - current_anchor.xy / current_anchor.w,
								# keep initial orientation
							np.ravel(dmat3x2(init_solid - solid)),
							]),
						optjac = np.stack([
							np.concatenate([
								# move grasped point under mouse
								(model * grad * anchor).xy / current_anchor.w, 
								# keep initial orientation
								np.ravel(dmat3x2(grad)), 
								])
							for grad in problem.jac]),
						)
					view.update()
				
				# process solving and move independently of events ticks to gain perf
				self.defered.set(prepare, solve, self.move_maxiter)
				
	def _move_rotate(self, view, sub, evt):
		# identify the solid clicked
		if sub[0] == 'scheme':  moved = self.index[sub[1]]
		else:                   moved = sub[0]
		if moved == self.kinematic.ground:
			return
		# define the kinematic problem in term of that solid, so we can get a gradient
		kinematic = Kinematic(
			ground = self.kinematic.ground, 
			inputs = self.kinematic.joints,
			outputs = [moved],
			)
		# constants during translation
		clicked = vec3(view.ptat(view.somenear(evt.pos())))
		solid = self.parts[moved]
		anchor = affineInverse(mat4(self.world) * solid) * vec4(clicked,1)
		target_tool = vec4(self.toolcenter, 1)
		target_axis = normalize(target_tool - vec4(affineInverse(view.uniforms['view'] * self.world)[3]))
		axis = affineInverse(solid) * target_axis
		tool = affineInverse(solid) * target_tool
		
		while True:
			evt = yield
			# drag
			if evt.type() in (QEvent.Type.MouseMove, QEvent.Type.MouseButtonRelease):
				evt.accept()
				view.update()
				if not (evt.buttons() & Qt.MouseButton.LeftButton):
					break

				model = mat4(view.uniforms['proj'] * view.uniforms['view'] * self.world)
				target_anchor = qtpos(evt.pos(), view)
				def prepare(problem):
					pass
				
				def solve(problem, model=model, target_anchor=target_anchor, target_tool=target_tool):
					solid = self.parts[moved]
					current_tool = model * solid * tool
					current_anchor = model * solid * anchor
					
					problem.jac = kinematic.grad((
						*self.pose, 
						kinematic.joints[-1].inverse(self.parts[moved]),
						))
					
					self._move_opt(
						optmove = np.concatenate([
							# move grasped point under mouse
							(target_anchor - current_anchor.xy / current_anchor.w),
							# keep tool center point at the same place
							(target_tool - solid * tool).xyz / kinematic.scale,
							# keep rotation axis unchanged
							(target_axis - solid * axis).xyz,
							]),
						optjac = np.stack([
							np.concatenate([
								# move grasped point under mouse
								(model * grad * anchor).xy / current_anchor.w, 
								# keep tool center point at the same place
								(grad * tool).xyz / kinematic.scale, 
								# keep rotation axis unchanged
								(grad * axis).xyz,
								])
							for grad in problem.jac]),
						)
					view.update()

				# process solving and move independently of events ticks to gain perf
				self.defered.set(prepare, solve, self.move_maxiter)


class DeferedSolving:
	''' helper executing solver iterations following ticks of a QTimer '''
	def __init__(self):
		self.timer = None
		self.problem = None
	def set(self, prepare, solve, iterations):
		''' schedule solving steps starting now for the given count of iterations '''
		if not self.timer:
			self.timer = QTimer()
			self.timer.timeout.connect(self._step)
			self.timer.setInterval(10)
		self.problem = DeferedProblem(prepare, solve, iterations)
		if not self.timer.isActive():
			self.timer.start()
	def stop(self):
		''' stop iterations '''
		self.timer.stop()
	def _step(self):
		''' iteration body '''
		problem = self.problem
		try:
			if problem.prepare:
				problem.prepare(problem)
			problem.solve(problem)
		except Exception as err:
			print(err)
			self.problem = None
			self.timer.stop()
		else:
			problem.iterations -= 1
			if problem.iterations <= 0:
				self.problem = None
				self.timer.stop()
				
@dataclass
class DeferedProblem:
	prepare: callable
	solve: callable
	iterations: int
						


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
	color = settings.colors['annotation']
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
		revolution(web([radius*rend, radius*rend-14*tend-3.6*rend]), Axis(radius*rend, tend), resolution=('div', 8)), 
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
	
	scheme = Scheme()
	for joint in joints:
		if hasattr(joint, 'position'):
			size = vec2(sum(
				vec2(distance(position, centers[solid]), 1)
				for solid, position in zip(joint.solids, joint.position)
				if centers[solid]))
			if size.y:
				size = 1.5*size.x / size.y
			else:
				size = inf
			
			sch = joint.scheme(index, size, centers[joint.solids[0]], centers[joint.solids[-1]])
			if sch is not NotImplemented:
				scheme += sch
	
	return scheme, {v: k  for k, v in index.items()}

kinematic_color_names = ['annotation', 'schematic']
def kinematic_color(i):
	''' return the scheme color vector for solid `i` in a kinematic '''
	return fvec4(settings.colors[kinematic_color_names[i%2]], 1)



def qtpos(qtpos, view):
	''' convert qt position in the widget to opengl screen coords in range (-1, 1) '''
	return vec2(
		+ (qtpos.x()/view.width() *2 -1),
		- (qtpos.y()/view.height() *2 -1),
		)

def normsq(x):
	x = x.ravel()
	return np.dot(x,x)
	
