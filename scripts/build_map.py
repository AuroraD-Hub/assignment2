#! /usr/bin/env python

"""
.. module::build_map
   :platform: Ubuntu 20.04
   :snyopsis: This module creates the topological map of the environment.

.. moduleauthor::Aurora Durante

The builder acquires information from the *marker_publisher* and adds the corresponding information 
to the ontology representing the map of the environment through the *marker_server* service.
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
pub_camera_arm2 = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=1, latch=True)
client = ArmorClient('client','assignment')
id_client = rospy.ServiceProxy('/room_info', RoomInformation)
info = RoomInformationRequest()
map_info = RoomInformationResponse()

tot_markers = 0
k_body = -1
marker_detected = []

def move_camera():
	"""
	Helper function to move the camera.
	
	The camera can be rotated around and translating along the z-axis to detect
	markers near the robot. The robot, firstly, rotate it of 360° and, then, translate
	it up so that it can rotate of 360° again.
	"""
	global k_body
	camera_pos = Float64()
	k_body = k_body+1
	if k_body>12: # camera is traslated up to detect markers on the walls after a complete scan of the ground
		camera_arm_pos = Float64()
		camera_arm_pos = 0.2
		pub_camera_arm.publish(camera_arm_pos)
		pub_camera_arm2.publish(camera_arm_pos)
		k_body = 0
	camera_pos = -3.14+k_body*0.5
	pub_camera_body.publish(camera_pos)
	rospy.sleep(1) 

def count_markers():
	"""
	Helper function to count how many correct markers are detected.
	
	Whenever the requested number of correctly detected markers is reached,
	the camera on the robot goes in its initial position and the builder publishes
	a message on */state/ontology* topic to let other nodes know the map is created.
	"""
	global tot_markers, marker_detected
	tot_markers = tot_markers+1
	if (tot_markers==pnm.TOTAL_MARKERS):
		client.call('DISJOINT','IND','CLASS',['LOCATION'])
		client.call('DISJOINT','IND','CLASS',['ROOM'])
		client.call('DISJOINT','IND','CLASS',['CORRIDOR'])
		client.call('DISJOINT','IND','CLASS',['URGENT'])
		client.call('DISJOINT','IND','CLASS',['DOOR'])
		client.call('REASON','','',[''])
		# Get the camera to its initial position
		pub_camera_body.publish(0)
		pub_camera_arm.publish(0)
		client.call('UNMOUNT','','',[''])
		# Publish message regarding the ending of ontology creation
		pub_ont.publish(0)
		rospy.loginfo(pnm.tag_log(f'The total number of correct markers detected is {tot_markers} and the complete list is: {marker_detected}',pnm.NODE_BUILDER))

def build_map(map_info):
	"""
	Helper function to add information of the location in the ontology.
	It uses the aRMOR client to create new instances in the ontology.
		    
	:param map_info: information of the location.
	:type data: RoomInformation
	"""
	if (map_info.room == 'E') or (map_info.room == 'C1') or (map_info.room == 'C2'):
		client.call('ADD','IND','CLASS',[map_info.room, 'CORRIDOR'])
		conn = map_info.connections
		for i in range(0,len(conn)):
			client.call('ADD','DATAPROP','IND',['hasX', map_info.room, 'Float', str(map_info.x)])
			client.call('ADD','DATAPROP','IND',['hasY', map_info.room, 'Float', str(map_info.y)])
			client.call('ADD','DATAPROP','IND',['visitedAt', map_info.room, 'Float', '0.0'])
			client.call('ADD','OBJECTPROP','IND',['hasDoor', map_info.room, map_info.connections[i].through_door])
			client.call('ADD','OBJECTPROP','IND',['connectedTo', map_info.room, map_info.connections[i].connected_to])
		rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))
		count_markers()
	elif (map_info.room == 'R1') or (map_info.room == 'R2') or (map_info.room == 'R3') or (map_info.room == 'R4'):
		client.call('ADD','IND','CLASS',[map_info.room, 'ROOM'])
		client.call('ADD','DATAPROP','IND',['hasX', map_info.room, 'Float', str(map_info.x)])
		client.call('ADD','DATAPROP','IND',['hasY', map_info.room, 'Float', str(map_info.y)])
		client.call('ADD','DATAPROP','IND',['visitedAt', map_info.room, 'Float', '0.0'])
		client.call('ADD','OBJECTPROP','IND',['hasDoor', map_info.room, map_info.connections[0].through_door])
		client.call('ADD','OBJECTPROP','IND',['connectedTo', map_info.room, map_info.connections[0].connected_to])
		rospy.loginfo(pnm.tag_log(f'Location "{map_info.room}" information are added to the map.',pnm.NODE_BUILDER))
		count_markers()
	else:
		rospy.loginfo(pnm.tag_log(f'{map_info.room}',pnm.NODE_BUILDER))
				
def read_id(data):
	"""
	Marker detection subscriber callback function.
	It retrives the id of the detected markers and calls the *marker_server* to 
	retrive the corresponding information.
		    
	:param data: retrive marker id.
	:type data: int
	"""
	global marker_detected
	if data.data not in marker_detected:
		marker_detected.append(data.data)
		rospy.loginfo(pnm.tag_log(f'Marker {data.data} is detected. List of detected markers is: {marker_detected}',pnm.NODE_BUILDER))
		info.id = data.data
		# Call marker_server to retrieve the information from the marker
		map_info = id_client.call(info)
		# Add the information retrived to the map
		build_map(map_info)

def main():
	"""
	The node is instantiated and the map created.
	     
	It loads the file in which the ontology will be created and subscribes to the
	*/marke_id* topic of *marker_publisher*. It continuosly move he camera untill
	all the markers are detected.
	"""
	
	# Initialise this node.
	rospy.init_node(pnm.NODE_BUILDER, log_level=rospy.INFO)
        
	# Retrive ontology file position 
	path = dirname(dirname(realpath(__file__)))
	path = path + "/utils/topological_map.owl"
	# Load it and mount on reference
	client.utils.load_ref_from_file(path, "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
	client.utils.mount_on_ref()
	
	sub = rospy.Subscriber('/marker_id', Int32, read_id)
	# Move the camera to detect markers
	global tot_markers
	while not (tot_markers==pnm.TOTAL_MARKERS):
		move_camera()
	
	# This loop is used only for displaying in the terminal detection result
	while not rospy.is_shutdown():
		rospy.sleep(1)
		

if __name__ == "__main__":
    main()
