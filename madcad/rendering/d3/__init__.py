import numpy.core as np

from ...common import resourcedir
from ...mathutils import Box, fvec3
from .view import *

def load_shader_ident(scene):
	return scene.context.program(
		vertex_shader=open(resourcedir+'/shaders/object-ident.vert').read(),
		fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
		)

def load_shader_subident(scene):
	return scene.context.program(
		vertex_shader=open(resourcedir+'/shaders/object-item-ident.vert').read(),
		fragment_shader=open(resourcedir+'/shaders/ident.frag').read(),
		)        

def load_shader_wire(scene):
	return scene.context.program(
		vertex_shader=open(resourcedir+'/shaders/wire.vert').read(),
		fragment_shader=open(resourcedir+'/shaders/wire.frag').read(),
		)
	
def load_shader_uniformcolor(scene):
	return scene.context.program(
		vertex_shader=open(resourcedir+'/shaders/uniformcolor.vert').read(),
		fragment_shader=open(resourcedir+'/shaders/uniformcolor.frag').read(),
		)

def npboundingbox(points: np.ndarray) -> Box:
	''' boundingbox for numpy arrays of points on the 3 first components '''
	return Box(
		fvec3(np.min(points, axis=0)),
		fvec3(np.max(points, axis=0)),
		)

from . import view, marker, dense
