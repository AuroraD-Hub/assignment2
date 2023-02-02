#! /usr/bin/env python

"""
.. module::build_map
   :platform: Ubuntu 20.04
   :snyopsis: This module creates the topological map of the environment.

.. moduleauthor::Aurora Durante


"""

import rospy
import time
import param_name_mapper as pnm
from std_msgs.msg import Int8, Int32, Float64
from armor_api.armor_client import ArmorClient
from assignment2.srv import RoomInformation, RoomInformationRequest, RoomInformationResponse
from assignment2.msg import RoomConnection
from os.path import dirname, realpath

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_BUILDER

pub_ont = rospy.Publisher(pnm.TOPIC_LOAD_ONTOLOGY, Int8, queue_size=1, latch=True)
tot_markers = 0
pub_camera_body = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1, latch=True)
pub_camera_arm = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1, latch=True)
k_body = 1

client = ArmorClient('client','assignment')
id_client = rospy.ServiceProxy('/room_info', RoomInformation)
info = RoomInformationRequest()
map_info = RoomInformationResponse()

def move_camera():
	camera_pos = Float64()
	rospy.loginfo(pnm.tag_log(f'fbgv {k_body}',pnm.NODE_BUILDER))
	if k_body>15: # camera is traslated up to detct markers on the walls
		pub_camera_arm.publish(0.1)
		rospy.loginfo(pnm.tag_log(f'bhiyfd {k_body}',pnm.NODE_BUILDER))
	camera_pos = k_body*0.5
	pub_camera_body.publish(camera_pos)
	k_body = k_body+1
	rospy.loginfo(pnm.tag_log(f'awesz {k_body}',pnm.NODE_BUILDER))

def count_markers():
	tot_markers = tot_markers+1
	if (tot_markers==pnm.TOTAL_MARKERS):
		client.manipulation.disj_inds_of_class('LOCATION')
		client.manipulation.disj_inds_of_class('ROOM')
		client.manipulation.disj_inds_of_class('CORRIDOR')
		client.manipulation.disj_inds_of_class('URGENT')
		client.manipulation.disj_inds_of_class('DOOR')
		client.utils.sync_buffered_reasoner()
		pub_ont.publish(0)
		pub_camera_body.pub(self.camera_pos)
		pub_camera_arm.publish(0)
		rospy.loginfo(pnm.tag_log(f'The total number of markers detected is {tot_markers}',pnm.NODE_BUILDER))

def build_map(map_info):
	if (map_info.room == 'E') or (map_info.room == 'C1') or (map_info.room == 'C2'):
		client.manipulation.add_ind_to_class(map_info.room, 'CORRIDOR')
		conn = map_info.connections
		for i in range(0,len(conn)):
			client.manipulation.add_dataprop_to_ind('hasX', map_info.room, Float64, map_info.x)
			client.manipulation.add_dataprop_to_ind('hasY', map_info.room, Float64, map_info.y)
			client.manipulation.add_objectprop_to_ind('hasDoor', map_info.room, map_info.connections[i].through_door)
			client.manipulation.add_objectprop_to_ind('connectedTo', map_info.room, map_info.connections[i].connected_to)
		rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))
		count_markers()
	elif (map_info.room == 'R1') or (map_info.room == 'R2') or (map_info.room == 'R3') or (map_info.room == 'R4'):
		client.manipulation.add_ind_to_class(map_info.room, 'ROOM')
		client.manipulation.add_dataprop_to_ind('hasX', map_info.room, Float64, map_info.x)
		client.manipulation.add_dataprop_to_ind('hasY', map_info.room, Float64, map_info.y)
		client.manipulation.add_objectprop_to_ind('hasDoor', map_info.room, map_info.connections[0].through_door)
		client.manipulation.add_objectprop_to_ind('connectedTo', map_info.room, map_info.connections[0].connected_to)
		rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))
		count_markers()
	else:
		rospy.loginfo(pnm.tag_log(f'{map_info.room}',pnm.NODE_BUILDER))
				
def read_id(data):
	rospy.loginfo(pnm.tag_log(f'Marker {data.data} is detected',pnm.NODE_BUILDER))
	info.id = data.data
	# Call marker_server to retrieve the information from the marker
	map_info = id_client.call(info)
	# Add the information retrived to the map
	build_map(map_info)
	# Move the camera to detect other markers
	move_camera()

def main():
	
	# Initialise this node.
	rospy.init_node(pnm.NODE_BUILDER, log_level=rospy.INFO)
        
	# Retrive ontology file position 
	path = dirname(dirname(realpath(__file__)))
	path = path + "/utils/topological_map.owl"
	# Load it and mount on reference
	client.utils.load_ref_from_file(path, "http://bnc/exp-rob-lab/2022-23",True, "PELLET", True, False)
	client.utils.mount_on_ref()
	
	sub = rospy.Subscriber('/marker_id', Int32, read_id)	
	rospy.spin()

if __name__ == "__main__":
    main()
