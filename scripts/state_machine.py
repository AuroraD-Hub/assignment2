#!/usr/bin/env python

"""
.. module::state_machine
   :platform: Ubuntu 20.04
   :snyopsis: This module represents the state machine of the architecture.

.. moduleauthor::Aurora Durante

The state machine is composed of three states:
 1) Charging
 2) RandomMoving
 3) Waiting
All of them refers to the Planner and Controller node to execute specific commands.
"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
import assignment2
import actionlib
import param_name_mapper as pnm
import planner as Plan
from actionlib import SimpleActionServer
from assignment2.srv import Planner_srv, Planner_srvRequest, Planner_srvResponse
from assignment2.msg import Control_actGoal
from std_msgs.msg import Bool, Int8, Float64
from armor_api.armor_client import ArmorClient

plan_client = rospy.ServiceProxy(pnm.NAME_SERVICE, Planner_srv)
control_client = actionlib.SimpleActionClient(pnm.NAME_ACTION, assignment2.msg.Control_actAction)
plan = Planner_srvRequest()
plan_res = Planner_srvResponse()
control = Control_actGoal()

# state: DETECTING
class Detecting(smach.State):
    """
    This models state *Detecting*.
    Its outcomes are:
     1) **detecting**: robot stays in Detecting state.
     2) **low_battery**: robot goes in Charging state.

    Here, the robot is in location *E* and is scanning the environment around it to detect
    all the markers and build the ontology. It subscribes to topic *state/new_ontology* to
    know whether the map is built through the Float variable *ont_needed*.
    """
    
    def __init__(self):
        #Here, Detecting state is initialized.
        #The parameter is initialized to later obtain corresponding value from the Robot State Node
        #and it is:
        # * *ont_needed* (int8): define if a new ontology is needed to be loaded.
        smach.State.__init__(self, outcomes=['low_battery','detecting'])
        self.ont_needed = Int8()
        rospy.loginfo(pnm.tag_log('Detecting marker state',pnm.NODE_STATE_MACHINE))
       
    def execute(self, userdata):
       """
       Here subscriber to topic *state/new_ontology* is defined.
    
       From the subscriber current boolean value of **ont_needed** is obtained.
       Based on this value, the robot keeps detecting markers by rotating and tilting the camera
       or it goes to charging the battery.
       """
       # load ontology if needed
       sub_load = rospy.Subscriber(pnm.TOPIC_LOAD_ONTOLOGY, Int8, self.ontology_state)
       time.sleep(1) # needed for synchronisation between nodes
       if self.ont_needed == 1:
         rospy.loginfo(pnm.tag_log('Ontology loading...',pnm.NODE_STATE_MACHINE))
         return 'detecting'
       else:
         rospy.loginfo(pnm.tag_log('Ontology loaded! Going to charge the battery',pnm.NODE_STATE_MACHINE))
         return 'low_battery'
         
    # Definition of helper functions: 
    def ontology_state(self, data):
      """
      Load ontology subscriber callback function.
      It retrives the float value from Robot State Node representing if a new ontology is needed to be loaded.
    
      :param data: retrive state of the ontology.
      :type data: int8
      """
      self.ont_needed = data.data  
      

# state: CHARGING
class Charging(smach.State):
    """
    This models state *Charging*.
    Its outcomes are:
     1) **ready**: robot goes in RandomMoving state.
     2) **low_battery**: robot stays in Charging state.

    Here, the robot is in location *E* and is waiting some time to richarge its battery based on
    Boolean variable *battery_low*.
    """

    def __init__(self):
        #Here, Charging state is initialized.
        #The parameters are initialized to later obtain corresponding values from the Robot State Node
        #and they are:
        # * *battery_low* (bool): define if battery level of the robot is low.
        smach.State.__init__(self, outcomes=['ready','low_battery'])
        self.battery_low = Bool()
        rospy.loginfo(pnm.tag_log('Charging state',pnm.NODE_STATE_MACHINE))
            
    def execute(self, userdata):
      """
      Here subscriber to topic *state/battery_low* is defined.
    
      From the subscriber current boolean value of **battery_low** is obtained.
      Based on this value, corresponding action from the planner or controller node are defined.
      """
      
      # recharge battery if needed
      sub_battery = rospy.Subscriber(pnm.TOPIC_BATTERY_LOW, Bool, self.battery_level)
      if self.battery_low:
        rospy.loginfo(pnm.tag_log('Battery has to recharge',pnm.NODE_STATE_MACHINE))
        # preempt goal
        return 'low_battery'
      else:
        rospy.loginfo(pnm.tag_log('Battery is fully charged',pnm.NODE_STATE_MACHINE))
        plan.command = 'exit'
        plan_res = plan_client.call(plan)
        if len(plan_res.reachable_urgent)>0:
          loc = random.choice(plan_res.reachable_urgent)
        else:
          loc = random.choice(plan_res.reachable_corridors)
        control.loc = loc
        control_client.call(control)
        control_client.wait_for_result()
        return 'ready'
    
    # Definition of helper functions:          
    def battery_level(self,data):
      """
      Battery level subscriber callback function.
      It retrives the boolean value from Robot State Node representing if the battery level is low.
    
      :param data: retrive state of the battery
      :type data: bool
      """
      self.battery_low = data.data     

# state: RANDOM_MOVING
class RandomMoving(smach.State):
    """
    This models state *RandomMoving*.
    Its outcomes are:
     1) **goal_reached**: robot goes in Waiting state.
     2) **low_battery**: robot goes in Charging state.
     3) **ready**: robot stays in RandomMoving state.

    Here, the robot is randomly choosing a corridor to move to. If it find a reachable location which is URGENT, 
    then it calls Planner Node to move to it.
    Moreover, whenever battery level is low, it calls Planner Node to go to location *E* to recharge the battery.
    """

    def __init__(self):
        #Here, RandomMoving state is initialized.
        #The parameters are initialized to later obtain corresponding values from the Robot State Node
        #and they are:
        # * battery_low* (bool): define if battery level of the robot is low.
        smach.State.__init__(self, outcomes=['goal_reached', 'low_battery', 'ready'])
        self.battery_low = Bool()
        rospy.loginfo(pnm.tag_log('RandomMoving state',pnm.NODE_STATE_MACHINE))
        
    def execute(self, userdata):
        """
        Subscribers to topic *state/battery_low* is defined.
    
        Here three outcomes are possible:
         1) From the subscriber it obtaines current boolean value of *battery_low* and calls the Planner 
            Node to go in Charging state if battery is low.
         2) Defines whether there are URGENT location to visit and calls Planner Node accordingly to go in Waiting state.
         3) Otherwise it moves in a randomly chosen CORRIDOR.
        """
        sub_battery = rospy.Subscriber(pnm.TOPIC_BATTERY_LOW, Bool, self.battery_level)
        if self.battery_low: # battery level is low
          rospy.loginfo(pnm.tag_log('Battery has to recharge',pnm.NODE_STATE_MACHINE))
          #preempt goal
          control.loc = 'E'
          control_client.call(control)
          control_client.wait_for_result()
          return 'low_battery'
        plan.command = 'exit'
        plan_res = plan_client.call(plan)
        if len(plan_res.reachable_urgent)>0:
          loc = random.choice(plan_res.reachable_urgent)
          control.loc = loc
          control_client.call(control)
          control_client.wait_for_result()
          return 'goal_reached'
        else:
          loc = random.choice(plan_res.reachable_corridors)
          control.loc = loc
          control_client.call(control)
          control_client.wait_for_result()
          return 'ready'
                    
    # Definition of helper function:
    def battery_level(self, data):
      """
      Battery level subscriber callback function.
      It retrives the boolean value from Robot State Node representing if the battery level is low.
    
      :param data: retrive state of the battery
      :type data: bool
      """
      self.battery_low = data.data
                

# state: WAIT
class Waiting(smach.State):
    """
    This models state *Waiting*.
    Its outcomes are:
     1) **time_out**: robot goes in RandomMoving state.
     2) **low_battery**: robot goes in Charging state.

    Here, the robot is waiting an amount of time in the current location.
    Moreover, whenever battery level is low, it calls Planner Node to go to location *E* to recharge the battery.
    """

    def __init__(self):
        #Here, Waiting state is initialized.
        #The parameters are initialized to later obtain corresponding values from the Robot State Node
        #and they are:
        # * *battery_low* (bool): define if battery level of the robot is low.
        smach.State.__init__(self, outcomes=['time_out', 'low_battery'])
        self.battery_low = Bool()
        self.camera_pos = Float64()
        self.k = 1
        rospy.loginfo(pnm.tag_log('Waiting state',pnm.NODE_STATE_MACHINE))

    def execute(self,userdata):
        """
        Subscriber *state/battery_low* and publisher to */robot/joint3_position_controller/command* are defined.
    
        From the subscriber current boolean value of *battery_low* is obtained. Based on this value, 
        corresponding action from the Planner Node are defined.
        Otherwise, it patrols the room by publishing *camera_pos* in corresponding topic and then calls Planner
        Node to exit from current location.
        """
        sub_battery = rospy.Subscriber(pnm.TOPIC_BATTERY_LOW, Bool, self.battery_level)
        pub_camera = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=1, latch=True)
        if self.battery_low: # battery level is low
          rospy.loginfo(pnm.tag_log('Battery has to recharge',pnm.NODE_STATE_MACHINE))
          #preempt goal
          control.loc = 'E'
          control_client.call(control)
          control_client.wait_for_result()
          return 'low_battery'
        else:  # robot patrols the room
          while self.camera_pos<6.28: # scan the whole room
            self.camera_pos = self.k*0.5
            pub_camera.publish(self.camera_pos)
            self.k = self.k+1
          # Exit from the room and update timestamps
          plan.command = 'exit'
          plan_res = plan_client.call(plan)
          if len(plan_res.reachable_urgent)>0:
            loc = random.choice(plan_res.reachable_urgent)
          else:
            loc = random.choice(plan_res.reachable_corridors)
          control.loc = loc
          control_client.call(control)
          return 'time_out'
          
    # Definition of helper function:
    def battery_level(self, data):
      """
      Battery level subscriber callback function.
      It retrives the boolean value from Robot State Node representing if the battery level is low.
    
      :param data: retrive state of the battery
      :type data: bool
      """
      self.battery_low = data.data
     

def main():
    """
    The state machine is initialized and started.

    SMACH is used to create the state machine. It has four states and transitions between
    one another are defined.
    The Introspection Server is also cretated for visualization purpouse.
    At last, the state machine is executed and it runs untill the application is stopped.
    """
    rospy.init_node(pnm.NODE_STATE_MACHINE, log_level=rospy.INFO)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('DETECTING', Detecting(),
                               transitions={'detecting':'DETECTING',
                                            'low_battery':'CHARGING'})
        smach.StateMachine.add('CHARGING', Charging(),
                               transitions={'ready':'RANDOM_MOVING',
                                            'low_battery':'CHARGING'})
        smach.StateMachine.add('RANDOM_MOVING', RandomMoving(),
                               transitions={'goal_reached':'WAITING',
                                            'low_battery':'CHARGING',
                                            'ready':'RANDOM_MOVING'})
        smach.StateMachine.add('WAITING', Waiting(),
                               transitions={'time_out':'RANDOM_MOVING',
                                            'low_battery':'CHARGING'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ASSIGNMENT2')
    sis.start()
    
    # Initialize clients to planner and controller
    rospy.wait_for_service(pnm.NAME_SERVICE)
    #rospy.wait_for_service(pnm.NAME_ACTION)
    control_client.wait_for_server()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
