#! /usr/bin/env python

"""
.. module::planner
   :platform: Ubuntu 20.04
   :snyopsis: This module defines the planner of the architecture.

.. moduleauthor::Aurora Durante

Here a service defining the controller of the architecture is created.
It plans actions based on which command the state machine is giving by querying
information about the ontology through the aRMOR API client by EmaroLab.
"""

import random
import rospy
import param_name_mapper as pnm
from std_msgs.msg import Int8
from assignment2.srv import Planner_srv, Planner_srvResponse
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_PLANNER

client = ArmorClient('client_planner','assignment')
res = Planner_srvResponse()

# An ArmorClient to simulate motion planning.
# It takes in input "command", which is a string defining what action the client have to do, and 
# "item", which is a Boolean value or an empty string based on which command is given.     
def plan():
     """
     Planner node initialized.
     
     Here, service planner is instantiated.
     """
     rospy.init_node(LOG_TAG)
     ser = rospy.Service(pnm.NAME_SERVICE, Planner_srv, handle_planner)
     rospy.spin()
     
def handle_planner(req):
    """
    Service callback.    
    The planner executes two different action based on what the state machine needs:
     1) **load**: it loads an available ontology
     2) **exit**: it exits from the current location
     
    :param req: The request of the service
    :type req: Planner_srv
    """
    client.utils.mount_on_ref()
    if req.command == 'exit': # exit from a location
       rospy.loginfo(pnm.tag_log('Getting reachable locations...',pnm.NODE_PLANNER))
       exit_from_loc()
       rospy.loginfo(pnm.tag_log('Reachable locations information retrived!',pnm.NODE_PLANNER))
       res.done = True
    client.utils.unmount_from_ref()
    return res
      
def exit_from_loc():
     """
     The planner queries all reachable locations.
     
     It defines which location are URGENT and which are just CORRIDORS and give them in response
     """
     client.utils.sync_buffered_reasoner()
     # Retrive reachable locations
     reachable_locs = client.query.objectprop_b2_ind('canReach', 'Robot1') # reachable locations
     rospy.loginfo(pnm.tag_log('Reachable locations are:',pnm.NODE_PLANNER))
     reachable_locs = split_str(reachable_locs)
     # Divide location in URGENT and CORRIDOR
     res.reachable_corridors = ''
     res.reachable_urgent = ''
     dim = len(reachable_locs)
     for i in range(0,dim):
       #cls = client.query.class_of_ind(reachable_locs[i],"false")
       cls = client.call('QUERY','CLASS','IND',[reachable_locs[i], "false"])
       cls = cls.queried_objects
       rospy.loginfo(pnm.tag_log(f'{reachable_locs[i]} belongs to classes:',pnm.NODE_PLANNER))
       cls = split_str(cls)
       if 'URGENT' in cls:
         res.reachable_urgent = res.reachable_urgent+','+reachable_locs[i]
       elif 'CORRIDOR' in cls:
         res.reachable_corridors = res.reachable_corridors+','+reachable_locs[i]
     res.reachable_corridors = res.reachable_corridors.split(',')[1:len(res.reachable_corridors)]
     res.reachable_urgent = res.reachable_urgent.split(',')[1:len(res.reachable_urgent)]

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
    # Instantiate the node manager service and wait.
    plan()
