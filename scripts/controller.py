#! /usr/bin/env python

"""
.. module::build_map
   :platform: Ubuntu 20.04
   :snyopsis: This module .

.. moduleauthor::Aurora Durante


"""

import rospy
import time
import assignment2
import param_name_mapper as pnm
from std_msgs.msg import Float32
from actionlib import SimpleActionServer
from armor_api.armor_client import ArmorClient
from assignment2.action import Control_actFeedback, Control_actResult

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_CONTROLLER
name = pnm.NAME_ACTION

client = ArmorClient('client','assignment')

class Control(object):
	
	def __init__(self):
		self.sas = actionlib.SimpleActionServer(name, assignment2.action.Control_act, execute_cb=self.execute, auto_start=False)
		self.sas.start()
		self.x_curr = Float32()
		self.y_curr = Float32()
		
	def execute(self, goal):
		if goal is None:
			rospy.logerr(pnm.tag_log('No location provided! This service will be aborted!', LOG_TAG))
			self.sas.set_aborted()
			return
		if self.sas.is_preempt_requested():
			self.sas.set_preempted()
			rospy.loginfo(pnm.tag_log('Action preempted!', LOG_TAG))
			return
		sub = rospy.Subscriber(pnm.TOPIC_POSITION, Float32(), self.get_position)
		self.pub = rospy.Publisher(pnm.TOPIC_POSITION, Float32(), queue_size=1, latch=True)
		
		# Starting position as feedback
		feedback = Control_actFeedback()
		feedback.x = self.x_curr
		feedback.y = self.y_curr
		self.sas.publish_feedback(feedback)
		
		# Define goal position to reach
		client.utils.sync_buffered_reasoner()
		pos = client.query.dataprop_b2_ind('position', goal.loc)
		self.move_to(pos,goal)
		
		# Final position as result. It is also published in the */state/position* topic
		result = Control_actResult()
		result.x = pos[0]
		result.y = pos[1]
		self.pub.publish(pos)
		rospy.loginfo(pnm.tag_log(f'Location {goal.loc} reached!', LOG_TAG))
		self.sas_succeded(result)
		
	def get_position(self, data):
		self.x_curr = data.data.x
		self.y_curr = data.data.y
																																																																																																																																																									
	def move_to(self, pos, goal):
		if (self.x_curr==pos[0]) and (self.y_curr==pos[1]):
			self.room_reached(goal)
		
	def room_reached(self, goal):
		# Retrive current position
		client.utils.sync_buffered_reasoner()
		previous_pos = client.query.objectprop_b2_ind('isIn', 'Robot1') #previous position
		rospy.loginfo(pnm.tag_log('Robot1 was in location:',pnm.NODE_CONTROLLER))
		previous_pos = str(split_str(previous_pos))
		# Replace new position
		client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', goal.loc, previous_pos)
		client.utils.sync_buffered_reasoner()
		# Retrive current timestamp and last time loc was visited
		current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
		current_time = current_time[0][1:11]
		last_visit = client.query.dataprop_b2_ind('visitedAt', goal.loc) # last time loc was visited
		last_visit = last_visit[0][1:11]
		# Replace new timestamp and new visit to loc
		client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), current_time)
		client.manipulation.replace_dataprop_b2_ind('visitedAt', goal.loc, 'Long', current_time, last_visit)
		client.utils.sync_buffered_reasoner() # call the Reasoner
		
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
