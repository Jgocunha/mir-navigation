#!/usr/bin/env python

import rospy, tf
# rospy is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters.
# tf is a package that lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinate frames in a tree structure
# buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.
import time
# This module provides various time-related functions
import os
#This module provides a portable way of using operating system dependent functionality.
from os.path import expanduser


from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState, ModelStates
# import gazebo libraries

# Create a global variable containing the path to a certain model
home = expanduser("~")
path = home + '/.gazebo/models/construction_barrel/model.sdf'



class Moving():
	# define __init__ method
	# args: model_name, Spawning1, pose (x,y) 
	def __init__(self, model_name, Spawning1, x_pose, y_pose):
		self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
		self.model_name = model_name
		self.rate = rospy.Rate(10)
		self.x_model_pose = x_pose
		self.y_model_pose =	y_pose
		self.Spawning1 = Spawning1 
		self.flag = 0
		self.flag1 = 0

	# define method for spawning models 
	def spawning(self,):
		with open(path) as f:
			product_xml = f.read()
		item_name   =   "product_{0}_0".format(0)
		print("Spawning model:", self.model_name)
		item_pose   =   Pose(Point(x=self.x_model_pose, y=self.y_model_pose,    z=0.0),   Quaternion(0.,0.,0.0,0))
		self.Spawning1(self.model_name, product_xml, "", item_pose, "world")
	
	# define method to move in the x axis
	# args: moving distances
	def moving_x(self, distance):
		obstacle = ModelState()
		model = rospy.wait_for_message('gazebo/model_states', ModelStates)
		model_original_pose_x = model.pose[0].position.x
		for i in range(len(model.name)):
			if model.name[i] == self.model_name and round((model.pose[i].position.x),1) <= round(self.x_model_pose,1)+1.0 and self.flag1 == 0:
				obstacle.model_name = self.model_name
				obstacle.pose = model.pose[i]
				obstacle.twist = Twist()
				obstacle.twist.linear.x = distance
				obstacle.twist.angular.z = 0
				self.pub_model.publish(obstacle)
			elif model.name[i] == self.model_name and round(model.pose[i].position.x,1) != round(self.x_model_pose,1):
				self.flag1 = 1
				obstacle.model_name = self.model_name
				obstacle.pose = model.pose[i]
				obstacle.twist = Twist()
				obstacle.twist.linear.x = -distance
				obstacle.twist.angular.z = 0
				self.pub_model.publish(obstacle)
			elif  round(model.pose[i].position.x,1) == round(self.x_model_pose,1):
				self.flag1 = 0
	
	# define method to move in the y axis
	# args: moving distances
	def moving_y(self, distance):
		obstacle = ModelState()
		model = rospy.wait_for_message('gazebo/model_states', ModelStates)
		model_original_pose_y = model.pose[0].position.y
		for i in range(len(model.name)):
			if model.name[i] == self.model_name and round((model.pose[i].position.y),1) <= round(self.y_model_pose,1)+1.0 and self.flag1 == 0:
				obstacle.model_name = self.model_name
				obstacle.pose = model.pose[i]
				obstacle.twist = Twist()
				obstacle.twist.linear.y = distance
				obstacle.twist.angular.z = 0
				self.pub_model.publish(obstacle)
			elif model.name[i] == self.model_name and round(model.pose[i].position.y,1) != round(self.y_model_pose,1):
				self.flag1 = 1
				obstacle.model_name = self.model_name
				obstacle.pose = model.pose[i]
				obstacle.twist = Twist()
				obstacle.twist.linear.y = -distance
				obstacle.twist.angular.z = 0
				self.pub_model.publish(obstacle)
			elif  round(model.pose[i].position.y,1) == round(self.y_model_pose,1):
				self.flag1 = 0


def main():
	rospy.init_node('moving_obstacle')

	Spawning1 = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
	rospy.wait_for_service("gazebo/spawn_sdf_model")

	# define objects
	obj0 = Moving("construction_barrel_0", Spawning1, -7.0,  7.9) # top left corner (moving in the y-axis)
	obj1 = Moving("construction_barrel_1", Spawning1,  7.0,  7.9) # top right corner (moving in the y-axis)
	obj2 = Moving("construction_barrel_2", Spawning1, -7.0,  0.3) # bottom left corner (moving in the y-axis)
	obj3 = Moving("construction_barrel_3", Spawning1,  7.0,  0.3) # bottom right corner (moving in the y-axis)
	obj4 = Moving("construction_barrel_4", Spawning1, -4.0,  4.0) # centre left (moving in the x-axis)
	obj5 = Moving("construction_barrel_5", Spawning1,  4.0,  4.0) # centre right (moving in the x-axis)

	# spawn objects
	obj0.spawning()
	obj1.spawning()
	obj2.spawning()
	obj3.spawning()
	obj4.spawning()
	obj5.spawning()

	# travel distances
	distance_min=1.0
	distance_max=1.5

	while not rospy.is_shutdown():
		# move objects
		obj0.moving_y(distance_min)
		obj1.moving_y(distance_min)
		obj2.moving_y(distance_min)
		obj3.moving_y(distance_min)

		obj4.moving_x(distance_max)
		obj5.moving_x(distance_max)


if __name__ == '__main__':
	main()
