

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from utils import *

class DivZero(BaseAgent):
	
	def initialize_agent(self):
		
		self.state = "kickoff"
		self.controller_state = SimpleControllerState()
		
		self.field_info = self.get_field_info()
	
	def get_output(self, packet):
		
		dv = self.get_dv(packet)
		
		car_face = Vec3(1).align_to(packet.game_cars[self.index].physics.rotation)
		
		dtc = dot(car_face, dv.normal())
		if dtc < 0.9:
			Align_Car_To(self, packet, dv, Vec3(0, 0, 1))
		else:
			Align_Car_To(self, packet, dv)
		
		self.controller_state.boost = dtc > 0.8 and dv.length() > 200
		self.controller_state.jump = True
		
		return self.controller_state
		
	
	def get_dv(self, packet):
		
		seconds_elapsed = 0
		
		my_car = packet.game_cars[self.index]
		
		ball_prediction = self.get_ball_prediction_struct()
		
		for i in range(ball_prediction.num_slices):
			
			slice = ball_prediction.slices[i]
			
			if slice.game_seconds > seconds_elapsed:
				dv = delta_v(my_car, Vec3.cast(slice.physics.location), slice.game_seconds - seconds_elapsed)
				if dv.length() < 1000:
					return dv
			
		
		my_goal = None
		for i in range(self.field_info.num_goals):
			goal = self.field_info.goals[i]
			if goal.team_num == self.team:
				my_goal = goal
				break
		
		return delta_v(my_car, Vec3.cast(my_goal.location), 1)
		

