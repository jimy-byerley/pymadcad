from madcad import *
from madcad.joints import *

zcrown_out = 51
zcrown_in = 44 + 2*9
zplanet_in = 9
zplanet_out = 8
zsun = zcrown_in - 2*zplanet_in

axis = Axis(O,Z)
bot = Axis(O,Z)
top = Axis(10*Z,Z)
m = mat4()

joints = []
for i,p in enumerate(regon(axis, zsun+zplanet_in, 3).points):
	joints.extend([
		Gear(('sun', f'planet_{i}'), -zsun/zplanet_in, p, bot, bot),
		Gear(('crown_in', f'planet_{i}'), zcrown_in/zplanet_in, p, bot, bot),
		Gear(('crown_out', f'planet_{i}'), zcrown_out/zplanet_out, p, top, top),
		])
joints.append(Revolute(('sun', 'crown_in'), axis))
kin = Kinematic(joints, ground='crown_in', content={
	'crown_in': Segment(30*X,40*X), 
	'crown_out': Segment(30*X,40*X),
	'sun': Segment(0*X,20*X),
	})
