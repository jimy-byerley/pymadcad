# This file is part of pymadcad,  distributed under license LGPL v3

''' This module defines the types and functions for kinematic manimulation and computation.

	
	Kinematics are a conceptual approach of mechanisms. It sort parts in groups called solids (in solids all parts have the same movement), and links the solids to each other using constraints named joints.
	That way no matter what are the parts, or what are their shape, or how parts interact - solids movements can be deduced only from joints.

	This allows designing the mechanisms before designing its parts. This also allows visualizing the mechanism whether it is complete or not.
	
	As parts in the same solid all have the same movement, solids are considered to be undeformable. This allows the to use the Screw theory to represent the force and movement variables (see https://en.wikipedia.org/wiki/Screw_theory). 
	In this module, screws are called ``Screw``.
	
	This module mainly features:
		- `Joint` - the base class for all joints, instances of joints define a kinematic
		- `Kinematic` - the general kinematic solver
		- `Chain` - a joint and kinematic solver dedicated to kinematic chains
			
	joints are defined in `madcad.joints`
'''

from .solver import *
from .assembly import *
