import math

from rlbot.utils.structures.game_data_struct import Vector3 as UI_Vec3

def constrain(n):
	return max(min(n, -1), 1)

def constrain_pi(n):
	while n > math.pi:
		n -= math.pi * 2
	while n < -math.pi:
		n += math.pi * 2
	return n

def correct(target, val, mult = 1):
	rad = constrain_pi(target - val)
	return (rad * mult)

class Vec3:
	def __init__(self, x=0, y=0, z=0):
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)
	
	def __add__(self, val):
		return Vec3(self.x + val.x, self.y + val.y, self.z + val.z)
	
	def __sub__(self, val):
		return Vec3(self.x - val.x, self.y - val.y, self.z - val.z)
	
	def __mul__(self, val):
		return Vec3(self.x * val, self.y * val, self.z * val)
	
	def set(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
	
	def align_to(self, rot):
		v = Vec3(self.x, self.y, self.z)
		v.set(v.x, math.cos(rot.roll) * v.y + math.sin(rot.roll) * v.z, math.cos(rot.roll) * v.z - math.sin(rot.roll) * v.y)
		v.set(math.cos(-rot.pitch) * v.x + math.sin(-rot.pitch) * v.z, v.y, math.cos(-rot.pitch) * v.z - math.sin(-rot.pitch) * v.x)
		v.set(math.cos(-rot.yaw) * v.x + math.sin(-rot.yaw) * v.y, math.cos(-rot.yaw) * v.y - math.sin(-rot.yaw) * v.x, v.z)
		return v
	
	def align_from(self, rot):
		v = Vec3(self.x, self.y, self.z)
		v.set(math.cos(rot.yaw) * v.x + math.sin(rot.yaw) * v.y, math.cos(rot.yaw) * v.y - math.sin(rot.yaw) * v.x, v.z)
		v.set(math.cos(rot.pitch) * v.x + math.sin(rot.pitch) * v.z, v.y, math.cos(rot.pitch) * v.z - math.sin(rot.pitch) * v.x)
		v.set(v.x, math.cos(-rot.roll) * v.y + math.sin(-rot.roll) * v.z, math.cos(-rot.roll) * v.z - math.sin(-rot.roll) * v.y)
		return v
	
	def length(self):
		return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
	
	def cast_Vector3(self):
		return UI_Vec3(self.x, self.y, self.z)
	
	def copy(self):
		return Vec3(self.x, self.y, self.z)
	
	def flatten(self):
		return Vec3(self.x, self.y, 0.0)
	
	def normal(self, n = 1):
		l = max(self.length(), 0.0001)
		return Vec3(self.x / l * n, self.y / l * n, self.z / l * n)
	
	def cast(v):
		return Vec3(float(v.x), float(v.y), float(v.z))

def dot(v1, v2):
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

# Modified for zero gravity
def delta_v(car, position, t):
	car_pos = Vec3.cast(car.physics.location)
	car_vel = Vec3.cast(car.physics.velocity)
	return Vec3((position.x - car_vel.x * t - car_pos.x) / (0.5 * t * t), (position.y - car_vel.y * t - car_pos.y) / (0.5 * t * t), (position.z - car_vel.z * t - car_pos.z) / (0.5 * t * t))

def Align_Car_To(agent, packet, vector: Vec3, up = Vec3(0, 0, 0)):
	
	my_car = packet.game_cars[agent.index]
	
	car_rot = my_car.physics.rotation
	
	car_rot_vel = Vec3.cast(my_car.physics.angular_velocity)
	
	local_euler = car_rot_vel.align_from(car_rot)
	
	align_local = vector.align_from(car_rot)
	
	local_up = up.align_from(car_rot)
	
	# Improving this
	rot_ang_const = 0.25
	stick_correct = 6.0
	
	a1 = math.atan2(align_local.y, align_local.x)
	a2 = math.atan2(align_local.z, align_local.x)
	
	if local_up.y == 0 and local_up.z == 0:
		a3 = 0.0
	else:
		a3 = math.atan2(local_up.y, local_up.z)
	
	yaw = correct(0.0, -a1 + local_euler.z * rot_ang_const, stick_correct)
	pitch = correct(0.0, -a2 - local_euler.y * rot_ang_const, stick_correct)
	roll = correct(0.0, -a3 - local_euler.x * rot_ang_const, stick_correct)
	
	agent.controller_state.yaw = constrain(yaw)
	agent.controller_state.pitch = constrain(pitch)
	agent.controller_state.roll = constrain(roll)
	
	agent.controller_state.steer = 0





