#! /usr/bin/env python

"""
.. module::build_map
   :platform: Ubuntu 20.04
   :snyopsis: This module creates the topological map of the environment.

.. moduleauthor::Aurora Durante


"""

import rospy
import time
from std_msgs.msg import Int32, Float32
from armor_api.armor_client import ArmorClient
from assignment2.srv import RoomInformation
from assignment2.msg import RoomConnection
from os.path import dirname, realpath

client = ArmorClient('client','assignment')
id_client = rospy.ServiceProxy('/room_info', RoomInformation)
info = RoomInformation()

def build_map(info):
	if info.room == 'E' || info.room == 'C1' || info.room == 'C2':
		client.manipulation.add_int_to_class(info.room, 'CORRIDOR')
		for i in range(0,len(info.connections)):
			client.manipulation.add_dataprop_to_ind('position', info.room, Float32, [info.x, info.y])
			client.manipulation.add_objectprop_to_ind('hasDoor', info.room, info.connections[i].through_door)
			client.manipulation.add_objectprop_to_ind('connectedTo', info.room, info.connections[i].connected_to)
	else:
		client.manipulation.add_int_to_class(info.room, 'ROOM')
		for i in range(0,len(info.connections)):
			client.manipulation.add_dataprop_to_ind('position', info.room, Float32, [info.x, info.y])
			client.manipulation.add_objectprop_to_ind('hasDoor', info.room, info.connections[i].through_door)
			client.manipulation.add_objectprop_to_ind('connectedTo', info.room, info.connections[i].connected_to)
			
	
	client.utils.sync_buffered_reasoner()
	client.manipulation.disj_inds_of_class('LOCATION')
	client.manipulation.disj_inds_of_class('ROOM')
	client.manipulation.disj_inds_of_class('CORRIDOR')
	client.manipulation.disj_inds_of_class('URGENT')
	client.manipulation.disj_inds_of_class('DOOR')
	client.utils.sync_buffered_reasoner()

def read_id(data):
	info.id = data.data
	id_client.call(info)
	build_map(info)

def main();
	
	# Retrive ontology file position 
	path = dirname(realpath(__file__))
	path = path + "/utils/topological_map.owl"
	# Load it and mount on reference
	client.utils.load_ref_from_file(path, "http://bnc/exp-rob-lab/2022-23",True, "PELLET", True, False)
	client.utils.mount_on_ref()
	client.utils.sync_buffered_reasoner()
	
	sub = rospy.Subscriber('/marker_id', Int32, read_id)

if __name__ == "__main__":
    main()
