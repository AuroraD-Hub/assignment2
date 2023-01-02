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
			rospy.logeinfo(pnm.tag_log('Action preempted!', LOG_TAG))
			return
		sub = rospy.Subscriber(pnm.TOPIC_POSITION, Float32[], self.get_position)
		
	def get_position(self, data):
		self.x_curr = data.data.x
		self.y_curr = data.data.y
	
	def go_to_point(self, goal):
		feedback = Control_actFeedback()
		feedback.x = self.x_curr
		feedback.y = self.y_curr
		self.sas.publish_feedback(feedback)
		client.utils.sync_buffered_reasoner()
		pos = client.query.dataprop_b2_ind('position', goal.loc)


if __name__ == '__main__':
	rospy.init_node(LOG_TAG)
	server = Control()
	rospy.spin()
