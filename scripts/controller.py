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
from nav_msgs.msg import Odometry
from assignment2.msg import Position
from actionlib import SimpleActionServer
from armor_api.armor_client import ArmorClient
from assignment2.msg import Control_actFeedback, Control_actResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_CONTROLLER
name = pnm.NAME_ACTION

client = ArmorClient('client_control','assignment')

class Control(object):
	"""
	Controller node.

	The controller communicates with the state machine to give command to *move_base*
	action client such to autonomously move the robot in the environment.
	It subscribes to */odom* topic and it publishes feedback on robot position. Once
	*move_base* finished the action, the Controller publishes the result also in
	*/state/position* topic.
	"""
	
	def __init__(self):
		# Here the node is initialized. Publishers and subscribers are:
		# * *sub*: it subscribes to */odom* topic to know robot position
		# * *pub*: it publishes to *state/position* the last position reached
		sub = rospy.Subscriber('/odom', Odometry, self.get_position) 
		self.pub = rospy.Publisher(pnm.TOPIC_POSITION, Position, queue_size=1, latch=True)
		self.curr_pos = Odometry()
		self.success = Bool()
		self.sas = actionlib.SimpleActionServer(name, assignment2.msg.Control_actAction, execute_cb=self.execute, auto_start=False)
		self.sas.start()	

	def execute(self, goal):
		"""
		Here, the action client of *move_base* is initialized and the goal position is 
		obtained by the ontology.
		
		The Controller publishes feedback on robot position and retrieve the goal position
		by querying the ontology. Then, it gives the coordinates to the action client of *move_base*
		that moves autonomously the robot towards them. Finally, it publishes the last position
		reached in the *state/position* topic
		"""
		client.utils.mount_on_ref()
		client.utils.sync_buffered_reasoner()
		move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		while not move_client.wait_for_server(rospy.Duration(10)):
			rospy.loginfo(pnm.tag_log('Waiting for move_base to log', LOG_TAG))
			
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
		feedback.x.append(self.curr_pos.pose.pose.position.x)
		feedback.y.append(self.curr_pos.pose.pose.position.y)
		self.sas.publish_feedback(feedback)
		
		# Define goal position to reach
		pos_x = client.call('QUERY','DATAPROP','IND',['hasX', goal.loc])
		pos_x = pos_x.queried_objects
		pos_x = float(pos_x[0][1:4])
		pos_y = client.call('QUERY','DATAPROP','IND',['hasY', goal.loc])
		pos_y = pos_y.queried_objects
		pos_y = float(pos_y[0][1:4])
		rospy.loginfo(pnm.tag_log(f'Goal position is [{pos_x}, {pos_y}]', LOG_TAG))
		mb_target = MoveBaseGoal()
		self.move_to(pos_x, pos_y, mb_target, move_client, goal) #move towards the goal
		
		# Final position as result. It is also published in the */state/position* topic
		result = Control_actResult()
		result.x = self.curr_pos.pose.pose.position.x
		result.y = self.curr_pos.pose.pose.position.y
		self.sas.set_succeeded(result)
		pos = Position()
		pos.x = self.curr_pos.pose.pose.position.x
		pos.y = self.curr_pos.pose.pose.position.y
		self.pub.publish(pos)
		if self.success:
			rospy.loginfo(pnm.tag_log(f'Location {goal.loc} reached!', LOG_TAG))
		
	def get_position(self, data):
		"""
		Odometry subscriber callback function.
		It retrives the current position in the *odom* frame.
		    
		:param data: retrive odometry information.
		:type data: Odometry
		"""
		self.curr_pos = data
																																																																																																																																																									
	def move_to(self, pos_x, pos_y, mb_target, move_client, goal):
		"""
		Controller calls *move_base* action client to move the robot.
		It sets the target position to be given to *move_base*.
		    
		:param pos_x: x coordinate of the target.
		:type data: float
		:param pos_y: y coordinate of the target.
		:type data: float
		:param mb_target: *move_base* input command.
		:type data: MoveBaseGoal
		:param move_client: *move_base* client.
		:param goal: target information in the ontology.
		:type data: string
		"""
		mb_target.target_pose.header.frame_id = "map"
		mb_target.target_pose.header.stamp = rospy.Time.now()
		mb_target.target_pose.pose.position.x = pos_x
		mb_target.target_pose.pose.position.y = pos_y
		mb_target.target_pose.pose.orientation.w = 1.0
		move_client.send_goal(mb_target)
		move_client.wait_for_result()
		if (abs(self.curr_pos.pose.pose.position.x-pos_x)<1) and (abs(self.curr_pos.pose.pose.position.y-pos_y)<1): 
			self.room_reached(goal)
			self.success = True
		else:
			rospy.loginfo(pnm.tag_log(f'Something went wrong... I reached point [{self.curr_pos.pose.pose.position.x, self.curr_pos.pose.pose.position.y}] instead of [{pos_x, pos_y}]', LOG_TAG))
			rospy.loginfo(pnm.tag_log(f'Goal status is {move_client.get_state()}', LOG_TAG))
			self.success = False
			client.utils.unmount_from_ref()
		
	def room_reached(self, goal):
		"""
		The controller modifies the ontology when it reaches the goal location.
		It updates the timestamp and the position of the robot in the ondtology.
		    
		:param goal: istance of the ontology representing the goal to reach.
		:type data: string
		"""
		# Retrive current position
		client.utils.sync_buffered_reasoner()
		previous_pos = client.query.objectprop_b2_ind('isIn', 'Robot1') #previous position
		previous_pos = previous_pos
		rospy.loginfo(pnm.tag_log('Robot1 was in location:',pnm.NODE_CONTROLLER))
		previous_pos = str(self.split_str(previous_pos))
		# Replace new position
		client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', goal.loc, previous_pos])
		client.utils.sync_buffered_reasoner()
		# Retrive current timestamp and last time loc was visited
		current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
		current_time = current_time[0][1:11]
		last_visit = client.query.dataprop_b2_ind('visitedAt', goal.loc) # last time loc was visited
		last_visit = last_visit[0][1:11]
		# Replace new timestamp and new visit to loc
		client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(int(time.time())), current_time])
		client.call('REPLACE','DATAPROP','IND',['visitedAt', goal.loc, 'Long', current_time, last_visit])
		client.utils.sync_buffered_reasoner() # call the Reasoner
		client.utils.unmount_from_ref()
		
	def split_str(self, string):
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
