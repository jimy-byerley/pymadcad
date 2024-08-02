from madcad import *
from madcad.joints import *

zcrown = 60
zplanet = 20
zsun = zcrown - 2*zplanet

axis = Axis(O,Z)
bot = Axis(O,Z)
top = Axis(10*Z,Z)
m = mat4()

joints = []
for i,p in enumerate(regon(axis, zsun+zplanet, 3).points):
	joints.extend([
		Gear(('sun', f'planet_{i}'), -zsun/zplanet, p, bot, bot),
		Gear(('crown', f'planet_{i}'), zcrown/zplanet, p, bot, bot),
		Revolute(('carrier', f'planet_{i}'), Axis(p,Z), top),
		])
#joints.append(Revolute(('sun', 'crown'), axis))
kin = Kinematic(joints, ground='crown', content={
	'crown': Segment(zcrown*X, zcrown*1.2*X), 
	'carrier': Segment(zcrown*X, zcrown*1.2*X),
	'sun': Segment(0*X, zsun*X),
	})