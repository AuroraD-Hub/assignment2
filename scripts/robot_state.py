#!/usr/bin/env python

"""
.. module::robot_state
   :platform: Ubuntu 20.04
   :snyopsis: This module simulates robot state.

.. moduleauthor::Aurora Durante

This node simulates the robot state by defining battery level and new ontology needed.
For this purpose, it publishs to topics to which the state machine subscribes:
 1) *state/new_ontology*: it defines if a new ontology has to be loaded
 2) *state/battery_low*: it defines if battery level of the robot is low
"""

import threading
import rospy
from std_msgs.msg import Bool, Int8
import param_name_mapper as pnm

class RobotState:
    """
    Robot state is simulated and communicated to other nodes by custom topics.

    Topics run in different threads such that stimuli can be communicated real-time like.
    """

    def __init__(self):
        #Here, the node is initialized and publishers defined.
        #Two threads are started and each one refers to a specific publisher that publishes
        #messages on the corresponding topic. Publishers are:
        # * *pub_ont*: it publishes on *state/new_ontology* topic
        # * *pub_battery*: it publishes on *state/battery_low* topic
        #The corresponding messages are:
        # * *ont_needed* (int8): defines if a new ontology is needed to be loaded (initialized as 1).
        # * *battery_low* (bool): define if battery level of the robot is low (initialized as True).
           
        # Initialise this node.
        rospy.init_node(pnm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialise ontology state
        self._ont_needed = 1
        pub_ont = rospy.Publisher(pnm.TOPIC_LOAD_ONTOLOGY, Int8, queue_size=1, latch=True)
        pub_ont.publish(self._ont_needed)
        if self._ont_needed:
          log_msg = 'New ontology is needed'
        else:
          log_msg = 'No new ontology is needed'
        self._print_info(log_msg)
        
        # Initialise battery level.
        self._battery_low = True
        # Start publisher on a separate thread.
        th = threading.Thread(target=self.is_battery_low)
        th.start()
        
    def is_battery_low(self):
        """
        Callback function for the new thread in which battery level is simulated.
    
        It defines a lathered publisher to the *state/battery_low* topic and uses
        the helper function :mod:'_battery_notifier' to simulate battery usage and 
        recharging time.
        """
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        pub_battery = rospy.Publisher(pnm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        # Publish battery level changes randomly.
        self.battery_notifier(pub_battery)
        
    def battery_notifier(self, pub_battery):
        """
        Helper function used to simulate battery usage and recharging time.
        It implements a while loop in which usage and recharging time is simulated based
        on the value obtained by the topic. It also prints related messages to let the
        user know battery state.
      
        :param pub_battery: publisher to the *state/battery_low* topic
        """
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            pub_battery.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                log_msg = f'Robot got low battery after {delay} seconds.'
                delay = 10 # simulate time needed to recharge
            else:
                log_msg = f'Robot got a fully charged battery after {delay} seconds.'
                delay = 50 # simulate battery usage time
            self._print_info(log_msg)
            # Wait to simulate battery usage.
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low
            
    # Print logging only when random testing is active.
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def _print_info(self, msg):
         """
         Helper function used to print terminal messages regarding robot state.
    
         :param msg: it containes the message to print on terminal.
         :type msg: string
         """
         rospy.loginfo(pnm.tag_log(msg, pnm.NODE_ROBOT_STATE))
        

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
