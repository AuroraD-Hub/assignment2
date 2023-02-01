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
from std_msgs.msg import Int8, Int32, Float32, Float32MultiArray
from armor_api.armor_client import ArmorClient
from assignment2.srv import RoomInformation, RoomInformationRequest, RoomInformationResponse
from assignment2.msg import RoomConnection
from os.path import dirname, realpath

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_BUILDER

pub_ont = rospy.Publisher(pnm.TOPIC_LOAD_ONTOLOGY, Int8, queue_size=1, latch=True)
tot_markers = 0

client = ArmorClient('client','assignment')
id_client = rospy.ServiceProxy('/room_info', RoomInformation)
info = RoomInformationRequest()
map_info = RoomInformationResponse()

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
		rospy.loginfo(pnm.tag_log(f'The total number of markers detected is {tot_markers}',pnm.NODE_BUILDER))

def build_map(map_info):
	if (map_info.room == 'E') or (map_info.room == 'C1') or (map_info.room == 'C2'):
		client.manipulation.add_ind_to_class(map_info.room, 'CORRIDOR')
		for i in range(0,len(map_info.connections)):
			client.manipulation.add_dataprop_to_ind('position', map_info.room, Float32MultiArray, [map_info.x, map_info.y])
			client.manipulation.add_objectprop_to_ind('hasDoor', map_info.room, map_info.connections[i].through_door)
			client.manipulation.add_objectprop_to_ind('connectedTo', map_info.room, map_info.connections[i].connected_to)
		count_markers()
	elif (map_info.room == 'R1') or (map_info.room == 'R2') or (map_info.room == 'R3') or (map_info.room == 'R4'):
		client.manipulation.add_ind_to_class(map_info.room, 'ROOM')
		for i in range(0,len(map_info.connections)):
			client.manipulation.add_dataprop_to_ind('position', map_info.room, Float32MultiArray, [map_info.x, map_info.y])
			client.manipulation.add_objectprop_to_ind('hasDoor', map_info.room, map_info.connections[i].through_door)
			client.manipulation.add_objectprop_to_ind('connectedTo', map_info.room, map_info.connections[i].connected_to)
		count_markers()
	else:
		rospy.loginfo(pnm.tag_log(f'{map_info.room}',pnm.NODE_BUILDER))
				
	rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))

def read_id(data):
	rospy.loginfo(pnm.tag_log(f'Marker {data.data} is detected',pnm.NODE_BUILDER))
	info.id = data.data
	map_info = id_client.call(info)
	build_map(map_info)

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
