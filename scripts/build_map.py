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
pub_camera_body = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1, latch=True)
pub_camera_arm = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=1, latch=True)
client = ArmorClient('client','assignment')
id_client = rospy.ServiceProxy('/room_info', RoomInformation)
info = RoomInformationRequest()
map_info = RoomInformationResponse()

tot_markers = 0
k_body = -1
marker_detected = []

def move_camera():
	global k_body
	camera_pos = Float64()
	k_body = k_body+1
	rospy.loginfo(pnm.tag_log(f'k = "{k_body}"',pnm.NODE_BUILDER))
	if k_body>12: # camera is traslated up to detect markers on the walls after a complete scan of the ground
		camera_arm_pos = Float64()
		camera_arm_pos = 0.2
		pub_camera_arm.publish(camera_arm_pos)
		k_body = 0
	camera_pos = -3.14+k_body*0.5
	pub_camera_body.publish(camera_pos)
	rospy.sleep(2) 

def count_markers():
	global tot_markers
	tot_markers = tot_markers+1
	if (tot_markers==pnm.TOTAL_MARKERS):
		client.call('DISJOINT','IND','CLASS',['LOCATION'])
		client.call('DISJOINT','IND','CLASS',['ROOM'])
		client.call('DISJOINT','IND','CLASS',['CORRIDOR'])
		client.call('DISJOINT','IND','CLASS',['URGENT'])
		client.call('DISJOINT','IND','CLASS',['DOOR'])
		client.call('REASON','','',[''])
		pub_camera_body.publish(self.camera_pos)
		pub_camera_arm.publish(0)
		client.call('UNMOUNT','','',[''])
		pub_ont.publish(0)
		rospy.loginfo(pnm.tag_log(f'The total number of markers detected is {tot_markers}',pnm.NODE_BUILDER))

def build_map(map_info):
	if (map_info.room == 'E') or (map_info.room == 'C1') or (map_info.room == 'C2'):
		client.call('ADD','IND','CLASS',[map_info.room, 'CORRIDOR'])
		conn = map_info.connections
		for i in range(0,len(conn)):
			client.call('ADD','DATAPROP','IND',['hasX', map_info.room, 'Float', str(map_info.x)])
			client.call('ADD','DATAPROP','IND',['hasY', map_info.room, 'Float', str(map_info.y)])
			client.call('ADD','OBJECTPROP','IND',['hasDoor', map_info.room, map_info.connections[i].through_door])
			client.call('ADD','OBJECTPROP','IND',['connectedTo', map_info.room, map_info.connections[i].connected_to])
		rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))
		count_markers()
	elif (map_info.room == 'R1') or (map_info.room == 'R2') or (map_info.room == 'R3') or (map_info.room == 'R4'):
		client.call('ADD','IND','CLASS',[map_info.room, 'ROOM'])
		client.call('ADD','DATAPROP','IND',['hasX', map_info.room, 'Float', str(map_info.x)])
		client.call('ADD','DATAPROP','IND',['hasY', map_info.room, 'Float', str(map_info.y)])
		client.call('ADD','OBJECTPROP','IND',['hasDoor', map_info.room, map_info.connections[0].through_door])
		client.call('ADD','OBJECTPROP','IND',['connectedTo', map_info.room, map_info.connections[0].connected_to])
		rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))
		count_markers()
	else:
		rospy.loginfo(pnm.tag_log(f'{map_info.room}',pnm.NODE_BUILDER))
				
def read_id(data):
	global marker_detected
	if data.data in marker_detected:
		rospy.loginfo(pnm.tag_log(f'Marker {data.data} information are already loaded in the ontology',pnm.NODE_BUILDER))
	else:	
		marker_detected.append(data.data)
		rospy.loginfo(pnm.tag_log(f'Marker {data.data} is detected. List of detected markers is: {marker_detected}',pnm.NODE_BUILDER))
		info.id = data.data
		# Call marker_server to retrieve the information from the marker
		map_info = id_client.call(info)
		# Add the information retrived to the map
		build_map(map_info)

def main():
	
	# Initialise this node.
	rospy.init_node(pnm.NODE_BUILDER, log_level=rospy.INFO)
        
	# Retrive ontology file position 
	path = dirname(dirname(realpath(__file__)))
	path = path + "/utils/topological_map.owl"
	# Load it and mount on reference
	client.utils.load_ref_from_file(path, "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
	client.utils.mount_on_ref()
	
	sub = rospy.Subscriber('/marker_id', Int32, read_id)
	global tot_markers
	while not (tot_markers==pnm.TOTAL_MARKERS):
			
		# Move the camera to detect other markers
		move_camera()
		

if __name__ == "__main__":
    main()
