#!/usr/bin/env python

"""
.. module::architecture_parameters
   :platform: Ubuntu 20.04
   :snyopsis: This module contains architectures parameters.

.. moduleauthor::Aurora Durante

Here are defined names of parameters used in every module of the architecture.
It also defines a function to output messages on terminal while knowing which
module print it.
"""

import rospy

# ---------------------------------------------------------

# The name of the node representing the state machine of this scenario.
NODE_STATE_MACHINE = 'assignment_sm'
# ---------------------------------------------------------

# The name of the node representing the shared knowledge required for this scenario.
NODE_ROBOT_STATE = 'robot_state'

# The name of the topic where the ontology state is published.
TOPIC_LOAD_ONTOLOGY = 'state/new_ontology'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The name of the topic where the robot position is published.
TOPIC_POSITION = 'state/position'

# The number of total markers the robot has to detect.
TOTAL_MARKERS = 7

# The time the robot stays in a location when visiting it
WAITING_TIME = 5
# ---------------------------------------------------------

# The name of the planner node.
NODE_PLANNER = 'planner'
NAME_SERVICE = '/plan'

# -------------------------------------------------

# The name of the controller node.
NODE_CONTROLLER = 'controller'
NAME_ACTION = '/control'

# -------------------------------------------------

# The name of the map builder node.
NODE_BUILDER = 'builder'

# -------------------------------------------------

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    """
    Helper function to print on terminal specifying which module is writing.

    :param msg: The message to print.
    :type msg: str.
    :param producer_tag: It defines which module is printing the message.
    :type producer_tag: str.
    """
    return '@%s>> %s' % (producer_tag, msg)
