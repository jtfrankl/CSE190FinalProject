#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture, moveService
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from read_config import read_config
from std_msgs.msg import Bool, Float32, String

class Robot():
	def __init__(self):
		self.C_GRAD = [31, 91, 33, 93, 32, 92]
		self.config = read_config()
		rospy.init_node("robot")
		self.tex_sensor = rospy.ServiceProxy(
				"requestTexture",
				requestTexture
		)
		self.move_service = rospy.ServiceProxy(
				"moveService",
				moveService
		)
		self.map_service = rospy.ServiceProxy(
				"requestMapData",
				requestMapData
		)
		
		self.temp_sensor = rospy.Subscriber(
				"/temp_sensor/data",
				temperatureMessage,
				self.handle_incoming_temp
		)
		self.start_temp = rospy.Publisher(
			"/temp_sensor/activation",
			Bool,
			queue_size = 10
		)
		self.temp_pub = rospy.Publisher(
				"/results/temperature_data",
				Float32,
				queue_size = 10
		)
		self.tex_pub = rospy.Publisher(
				"/results/texture_data",
				String,
				queue_size = 10
		)

		self.probabilities_pub = rospy.Publisher(
				"/results/probabilities",
				RobotProbabilities,
				queue_size = 10
		)

		self.finished_pub = rospy.Publisher(
				'/map_node/sim_complete',
				Bool,
				queue_size = 10
		)
		rospy.sleep(1)
		"""start temperature sensor"""
		self.start_temp.publish(True)
		"""read config file"""
		self.pipemap = self.config["pipe_map"]
		self.possible_moves = self.config["possible_moves"]
		self.map_length = len(self.pipemap)
		self.map_width = len(self.pipemap[0])
		tmpmap = []
		
		for i in range(0, len(self.pipemap)):
			tmpmap = tmpmap + self.pipemap[i]
		
		self.pipemap = tmpmap
		
		self.texmap = self.config["texture_map"]
		tmpmap = []
		for i in range(0, len(self.texmap)):
			tmpmap = tmpmap + self.texmap[i]
		self.texmap = tmpmap

		self.temp_std_dev = self.config["temp_noise_std_dev"]
		self.prob_tex_correct = self.config["prob_tex_correct"]
		self.map_size = self.map_length*self.map_width
		"""initialize probabilities"""
		self.location_prob = []
		for i in range(0, self.map_size):
			self.location_prob.append(float(1)/self.map_size)

		self.b = 3
		self.prob_max = 0
		self.moves = self.config['move_list']
		
		
		rospy.spin()


	def handle_incoming_temp(self, message):
		mtemp = message.temperature
		self.temp = mtemp
		self.temp_probabilities()
		self.temp_pub.publish(self.temp)

		texval = self.tex_sensor()
		self.tex = texval.data
		self.tex_probabilities()
		self.tex_pub.publish(self.tex)
		move_count = 0
		
		#if(move_count < 20):
		if(self.moves != []):
			if(self.b == 3):
				self.next_move = self.moves.pop(0)
				self.b = 1
				self.move_service(self.next_move)
				self.move_probabilities()
			else:
				#self.next_move = [0,0]
				self.b = self.b+1
			#self.next_move = r.choice(self.possible_moves)
			print self.b, self.next_move
			
			
			self.publish_beliefs()
		else:
			self.publish_beliefs()
			rospy.sleep(1)
			self.finished_pub.publish(True)
			rospy.sleep(2)
			rospy.signal_shutdown("out of moves")

	def temp_probabilities(self):
		tmp_list = []
		for i in range(0, self.map_size):
			#print self.pipemap[i]
			if(self.pipemap[i] == 'H'):
				mean = 40
			elif(self.pipemap[i] == '-'):
				mean = 25
			else:
				mean = 20

			norm = (1/(self.temp_std_dev*m.sqrt(2*m.pi)))*m.exp(-m.pow(self.temp-mean,2)/(2*m.pow(self.temp_std_dev,2)))
			tmp_list.append(norm*self.location_prob[i])
		
		sum = 0
		for i in tmp_list:
			sum = sum + i
		
		for i in range(0, len(tmp_list)):
			tmp_list[i] = tmp_list[i]/sum
		
		self.location_prob = tmp_list
		
			#print i/self.map_width, i%self.map_length
			#loc_temp = self.pipemap[i/self.map_width][i%self.map_length]
			#print loc_temp
		#print "temp prob"

	def tex_probabilities(self):
		tmp_list = []
		for i in range(0, self.map_size):
			texture = self.texmap[i]
			if self.tex == texture:
				ptex = self.prob_tex_correct
			else:
				ptex = 1 - self.prob_tex_correct
			tmp_list.append(ptex*self.location_prob[i])
		sum = 0
		for i in tmp_list:
			sum = sum + i
		
		for i in range(0, len(tmp_list)):
			tmp_list[i] = tmp_list[i]/sum
		self.location_prob = tmp_list



		#print "text prob"


	def move_probabilities(self):
		tmp_list = [0] * self.map_size
		wrong_moves = deepcopy(self.config['possible_moves'])
		wrong_moves.remove(self.next_move)
		pmove_correct = self.config['prob_move_correct']
		pmove_wrong = (1-pmove_correct)/4
		for i in range(0, self.map_size):
			if(i%self.map_width == 0):
				l_index = i + (self.map_width - 1)
				r_index = i + 1
			elif((i + 1)% self.map_width== 0):
				l_index = i - 1
				r_index = i - (self.map_width - 1)
			else:
				l_index = i - 1
				r_index = i + 1
			
			if(i < self.map_width):
				u_index = i + self.map_width*(self.map_length - 1)
				d_index = i + self.map_width
			elif(i >= (self.map_size - self.map_length - 1)):
				u_index = i - self.map_width 
				d_index = i % self.map_width
			else:
				u_index = i - (self.map_width - 1) -1
				d_index = i + self.map_width


			#print i, "left: ", l_index, "right:", r_index, "down: ", d_index, "up: ", u_index

			
			if(self.next_move == [0,0]):
				tmp_list[i] = tmp_list[i] + pmove_correct*self.location_prob[i]
				tmp_list[l_index] = tmp_list[l_index] + pmove_wrong*self.location_prob[i]
				tmp_list[r_index] = tmp_list[r_index] + pmove_wrong*self.location_prob[i]
				tmp_list[d_index] = tmp_list[d_index] + pmove_wrong*self.location_prob[i]
				tmp_list[u_index] = tmp_list[u_index] + pmove_wrong*self.location_prob[i]
			elif(self.next_move == [1,0]):
				tmp_list[i] = tmp_list[i] + pmove_wrong*self.location_prob[i]
				tmp_list[l_index] = tmp_list[l_index] + pmove_wrong*self.location_prob[i]
				tmp_list[r_index] = tmp_list[r_index] + pmove_wrong*self.location_prob[i]
				tmp_list[d_index] = tmp_list[d_index] + pmove_correct*self.location_prob[i]
				tmp_list[u_index] = tmp_list[u_index] + pmove_wrong*self.location_prob[i]
			elif(self.next_move == [-1,0]):
				tmp_list[i] = tmp_list[i] + pmove_wrong*self.location_prob[i]
				tmp_list[l_index] = tmp_list[l_index] + pmove_wrong*self.location_prob[i]
				tmp_list[r_index] = tmp_list[r_index] + pmove_wrong*self.location_prob[i]
				tmp_list[d_index] = tmp_list[d_index] + pmove_wrong*self.location_prob[i]
				tmp_list[u_index] = tmp_list[u_index] + pmove_correct*self.location_prob[i]
			elif(self.next_move == [0,1]):
				tmp_list[i] = tmp_list[i] + pmove_wrong*self.location_prob[i]
				tmp_list[l_index] = tmp_list[l_index] + pmove_wrong*self.location_prob[i]
				tmp_list[r_index] = tmp_list[r_index] + pmove_correct*self.location_prob[i]
				tmp_list[d_index] = tmp_list[d_index] + pmove_wrong*self.location_prob[i]
				tmp_list[u_index] = tmp_list[u_index] + pmove_wrong*self.location_prob[i]
			elif(self.next_move == [0,-1]):
				tmp_list[i] = tmp_list[i] + pmove_wrong*self.location_prob[i]
				tmp_list[l_index] = tmp_list[l_index] + pmove_correct*self.location_prob[i]
				tmp_list[r_index] = tmp_list[r_index] + pmove_wrong*self.location_prob[i]
				tmp_list[d_index] = tmp_list[d_index] + pmove_wrong*self.location_prob[i]
				tmp_list[u_index] = tmp_list[u_index] + pmove_wrong*self.location_prob[i]

		"""newmax = False
		print self.moves
		for p in tmp_list:
			if p > self.prob_max:
				prob_max = p
				newmax = True
		if(newmax):
			tmp_move = self.next_move

			self.next_move = [-1*self.next_move[0], -1*self.next_move[1]]

			self.moves = [self.next_move, tmp_move] + self.moves

		print prob_max, self.next_move
		print self.moves
"""

		#print tmp_list
		self.location_prob = tmp_list
	def publish_beliefs(self):
		self.print_2d_floats(self.location_prob)
		self.probabilities_pub.publish(self.location_prob)


	

	# Utility
	def map_2d(self, f, a):
		return [[f(c) for c in r] for r in a]

	def zipwith(self, f, a, b):
		return [f(ac, bc) for ac, bc in zip(a, b)]

	def zipwith_2d(self, f, a, b):
		return [zipwith(f, ar, br) for ar, br in zip(a, b)]

	def color(self, code, s):
		return '\033[{}m{}\033[00m'.format(code, s)

	def print_2d_floats(self, a):

		tmp = []
		for i in range(4):
			inner = []
			for j in range(5):
				inner.append(a[i*5+j])
			tmp.append(inner)
		a = tmp
		rowline = '\n+{}\n'.format(('-' * 5 + '+') * len(a[0]))
		print(rowline + rowline.join(
			'|{}|'.format('|'.join(
				self.color(self.C_GRAD[min(len(self.C_GRAD) - 1, int(c * 2 * len(self.C_GRAD)))],
					  '{:1.3f}'.format(c)) for c in r
			))
		for r in a) + rowline)

	
if __name__ == '__main__':
		r = Robot()
