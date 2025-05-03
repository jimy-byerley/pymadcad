# This file is part of pymadcad,  distributed under license LGPL v3
'''
	implementation of views and displays for 3D
'''
import numpy as np

from ...common import resourcedir
from ...mathutils import Box, fvec3
from ..base import Display

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

def npboundingbox(points: np.ndarray, ignore=False) -> Box:
	''' boundingbox for numpy arrays of points on the 3 first components '''
	if not ignore:
		return Box(
			fvec3(np.min(points, axis=0)),
			fvec3(np.max(points, axis=0)),
			)
	else:
		return Box(
			fvec3(np.nan_to_num(np.min(points, axis=0), True, +np.inf)),
			fvec3(np.nan_to_num(np.max(points, axis=0), True, -np.inf)),
			)

from . import view, marker, dense
from .view import *
