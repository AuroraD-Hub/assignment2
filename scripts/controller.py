#! /usr/bin/env python

"""
.. module::controller
   :platform: Ubuntu 20.04
   :snyopsis: This module represents the controller of the architecture.

.. moduleauthor::Aurora Durante

The controller is a SimpleActionServer that manages the motion of the robot:
it publishes feddbeck on the position and modifies the ontology whenever the
goal is reached.
"""

import rospy
import time
import assignment2
import actionlib
import param_name_mapper as pnm
from std_msgs.msg import Float64, Bool
from assignment2.msg import Position
from actionlib import SimpleActionServer
from armor_api.armor_client import ArmorClient
from assignment2.msg import Control_actFeedback, Control_actResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_CONTROLLER
name = pnm.NAME_ACTION

client = ArmorClient('client','assignment')

class Control(object):
	
	def __init__(self):
		self.sas = actionlib.SimpleActionServer(name, assignment2.msg.Control_actAction, execute_cb=self.execute, auto_start=False)
		self.sas.start()
		sub = rospy.Subscriber(pnm.TOPIC_POSITION, Position, self.get_position)
		self.pub = rospy.Publisher(pnm.TOPIC_POSITION, Position, queue_size=1, latch=True)
		self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		self.x_curr = Float64()
		self.y_curr = Float64()
		self.success = Bool()
		
	def execute(self, goal):
		client.call('MOUNT','','',[''])
		client.call('REASON','','',[''])
		if goal is None:
			rospy.logerr(pnm.tag_log('No location provided! This service will be aborted!', LOG_TAG))
			self.sas.set_aborted()
			return
		if self.sas.is_preempt_requested():
			self.sas.set_preempted()
			rospy.loginfo(pnm.tag_log('Action preempted!', LOG_TAG))
			return
		
		rospy.loginfo(pnm.tag_log(f'Going towards {goal}', LOG_TAG))
		# Starting position as feedback
		feedback = Control_actFeedback()
		feedback.x.append(self.x_curr)
		feedback.y.append(self.y_curr)
		self.sas.publish_feedback(feedback)
		
		# Define goal position to reach
		pos_x = client.query.dataprop_b2_ind('hasX', goal.loc)
		pos_x = float(pos_x[0][1:4])
		pos_y = client.query.dataprop_b2_ind('hasY', goal.loc)
		pos_y = float(pos_y[0][1:4])
		rospy.loginfo(pnm.tag_log(f'Goal position is [{pos_x}, {pos_y}]', LOG_TAG))
		mb_target = MoveBaseGoal()
		self.move_to(pos_x, pos_y, mb_target, goal)
		
		# Final position as result. It is also published in the */state/position* topic
		result = Control_actResult()
		result.x = pos_x
		result.y = pos_y
		self.sas.set_succeeded(result)
		pos = Position()
		pos.x = pos_x
		pos.y = pos_y
		self.pub.publish(pos)
		if self.success:
			rospy.loginfo(pnm.tag_log(f'Location {goal.loc} reached!', LOG_TAG))
		
	def get_position(self, data):
		self.x_curr = float(data.x)
		self.y_curr = float(data.y)
																																																																																																																																																									
	def move_to(self, pos_x, pos_y,mb_target, goal):
		mb_target.target_pose.pose.position.x = pos_x
		mb_target.target_pose.pose.position.y = pos_y
		self.move_client.send_goal(mb_target)
		self.move_client.wait_for_result()
		if (abs(self.x_curr-pos_x)<1) and (abs(self.y_curr-pos_y)<1): 
			self.room_reached(goal)
			self.success = True
		else:
			rospy.loginfo(pnm.tag_log(f'Something went wrong... I was not able to reach point [{pos_x, pos_y}]', LOG_TAG))
			self.success = False
		
	def room_reached(self, goal):
		# Retrive current position
		client.call('REASON','','',[''])
		#previous_pos = client.call('QUERY','OBJECTPROP','IND',['isIn', 'Robot1']) #previous position
		previous_pos = client.query.objectprop_b2_ind('isIn', 'Robot1') #previous position
		rospy.loginfo(pnm.tag_log('Robot1 was in location:',pnm.NODE_CONTROLLER))
		previous_pos = str(split_str(previous_pos))
		# Replace new position
		#client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', goal.loc, previous_pos)
		client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', goal.loc, previous_pos])
		client.call('REASON','','',[''])
		# Retrive current timestamp and last time loc was visited
		#current_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1']) #current time
		current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
		current_time = current_time[0][1:11]
		#last_visit = client.call('QUERY','DATAPROP','IND',['visitedAt', 'Robot1']) # last time loc was visited
		last_visit = client.query.dataprop_b2_ind('visitedAt', goal.loc) # last time loc was visited
		last_visit = last_visit[0][1:11]
		# Replace new timestamp and new visit to loc
		client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(int(time.time())), current_time])
		#client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), current_time)
		client.call('REPLACE','DATAPROP','IND',['visitedAt', goal.loc, 'Long', current_time, last_visit])
		#client.manipulation.replace_dataprop_b2_ind('visitedAt', goal.loc, 'Long', current_time, last_visit)
		client.call('REASON','','',['']) # call the Reasoner
		client.call('UNMOUNT','','',[''])
		
	def split_str(string):
		"""
		Helper function used to get queried object from string.
		   
		:param string: The string to split to get information needed
		:type string: string
		"""
		length = len(string)
		for i in range(0,length):
		   string[i] = string[i][32:-1]
		print(string)
		return string
	 	


if __name__ == '__main__':
	rospy.init_node(LOG_TAG)
	server = Control()
	rospy.spin()
