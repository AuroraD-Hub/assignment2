# ExpRobLab Assignment2
**ROS-based architecture for Experimental Robotics Laboratory second assignment.**  
Author: *Aurora Durante*, MS in Robotics Engineering, UNIGE, Genova, Italy.  
Contact: aurora.durante@coservizi.it

## Introduction
In this assignment a ROS-based software architecture for a robot with surveillance porpouse is defined.  
It is based on the **OWL-DL** approach to create an ontology of the environment and it uses [SMACH](http://wiki.ros.org/smach) to implement a **Finite State Machine** to control robot behaviour. The ontology is visible with Protégé and the architecture behaviour is based on [ARMOR](https://github.com/EmaroLab/armor).

Related documentation on the code of this solution can be found here:  
 

### Software tools needed
Use [Protégé](https://protege.stanford.edu/) to directly open and read the ontology used here while with [SMACH viewer](http://wiki.ros.org/smach_viewer) it is possible to see how the architecture behaves in run-time.

In order to correctly execute this possible solution, clone the [Aruco package](https://github.com/CarmineD8/aruco_ros) (modifica) and the [package for autonomous navigation](https://github.com/CarmineD8/planning). Also the [ARMOR API Client](https://github.com/EmaroLab/armor_py_api) repository from EmaroLab (UNIGE) is needed to be cloned in the same workspace where this repository is downloaded. Then, copy and paste the following code in the `armor_query_client.py` file in the API `/scripts` folder:
```
def class_of_ind(self, ind, bottom):
        """
        Query which class an individuals belong to.
    
        Args:
            ind(str): an individual of the ontology
            bottom(bool): 
    
        Returns:
            list(str): the class of the individual
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
        """
        try:
            res = self._client.call('QUERY', 'CLASS', 'IND', [ind, bottom])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon querying individuals belonging to class {0}".format(ind, bottom))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.queried_objects
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
```

## Software robot architecture
It is composed of four nodes:
* *state_machine*: it implements every state of the state machine and defines the behaviour of the robot;
* *robot_state*: it simulates stimuli of the robot regarding battery and ontology states;
* *build_map*: it creates the ontology from the detection of markers in the environment;
* *planner*: this service plans the necessary action that the robot should perform in a specific state and based on the stimuli;
* *controller*: this service executes actions such that the robot moves in the environment.

There is also a *param_name_mapper* interface that collects all the necessary information regarding names of the topics and services and values of parameters used in all the architecture. Moreover, *build_map*, *planner* and *controller* use the ARMOR API Client from EmaroLab.  
In the following components diagram it can be seen how these nodes interact:
![sw_architecture drawio](https://user-images.githubusercontent.com/72380912/204393486-a1d3392a-8ad5-47a4-bb35-e65b45778ddf.jpg)


### Robot state
The `robot_state` node is a publisher and it simulates stimuli of the robot as battery level and ontology state.
![rs_component drawio](https://user-images.githubusercontent.com/72380912/204393545-b0ee612b-12df-4d45-b502-5de7010a759b.jpg)  
It creates two topics to which it continuosly publishes related messages:
* */state/battery_low*: if the boolean message is *True*, then battery level is low;
* */state/new_ontology*: if the integer message is *1*, then a new ontology has to be loaded.

Simulation of the battery level is defined by a while-loop that modify the boolean value to publish accordingly to a specific delay. This delay is used to simulate both the charging time and battery usage time by setting different values based on topic message published.

### Planner
The `planner` node is a service and it plans the action that the robot should perform.  
![plan_component drawio](https://user-images.githubusercontent.com/72380912/204393610-7deeb839-e150-4458-a95f-ba08760f1363.jpg)  
The `Planner_srv` message is composed as follows:
* Request:
  * *command*: is a string message that defines the action to plan (load the ontology/exit from current location)
* Response:
  * *done*: is a boolean message that notify when the plan is done
  * *reachable_urgent*: is a string list that queries all the reachable locations that the robot should visit as they are urgent
  * *reachable_corridors*: is a string list that queries all the reachable corridors that the robot can move to if there are not urgent rooms

These tasks are performed by the `planner` throught the ARMOR API Client that uses its *utils* and *query* functions for loading the ontology and exiting the location respectively.

### Controller
The `controller` node is a service and it manages changes in the ontology when the robot moves from one location to another.  
![control_component drawio](https://user-images.githubusercontent.com/72380912/204393660-e8414434-49f7-4772-ad48-1003293eca84.jpg)  
The `Controller_srv` message is composed as follows:
* Request:
  * *loc*: is a string message that defines in which location the robot is moving to
* Response:
  * *done*: is a boolean message that notify when the robot moved in the new location

These tasks are performed by the `controller` throught the ARMOR API Client that uses its *manipulation* functions for updating the information about location, robot and timestamps in the ontology.

### State machine
The `state_machine` node implements the Finite State Machine that manages the behaviour of the robot.  
![sm_component drawio](https://user-images.githubusercontent.com/72380912/204393705-51c69087-1ec5-4f9f-a4b4-65da126b77fc.jpg)  
It subscribes to the two topics created in `robot_state` node and calls the `planner` and `controller` nodes to manipulate the ontology and move in the environment. To do that it uses the custom service requests.

## Software behaviour
The state machine is composed of four states: *Detecting*, *Charging*, *RandomMoving* and *Waiting*. They are depicted in the following state diagram with the corresponding transitions:  
![state_diagram drawio](https://user-images.githubusercontent.com/72380912/204393786-5cc769b0-c536-4147-98d9-cbbd2a177a20.jpg)  
*Detecting* state is the initial state of the architecture

In *Charging* state the robot waits in location E for the battery to get fully charged. It has two outcomes:
* *battery_low*: state machine stays in *Charging* until `/state/battery_low` topic informs it that battery is fully charged;
* *ready*: state machine goes in *RandomMoving* state as soon as battery is charged.  

In *RandomMoving* state the robot moves randomly in the environment staying mainly in CORRIDORs rather then in ROOMs untill a URGENT location is reachable. It has three outcomes:
* *ready*: state machine stays in *RandomMoving* until the robot is moving between CORRIDORs that are not URGENT;
* *battery_low*: state machine goes in *Charging* if `/state/battery_low` topic informs it that battery is low;
* *goal_reached*: state machine goes in *Waiting* state as soon as a URGENT location is reached.  

In *Waiting* state the robot has reached a URGENT location and, thus, it has to stay there for some time. It has two outcomes:
* *battery_low*: state machine goes in *Charging* if `/state/battery_low` topic informs it that battery is low;
* *time_out*: state machine goes in *RandomMoving* state as soon as waiting time has elapsed.  

## Installing and running
This architecture is based on ROS Noetic and is developped in the Docker environment provided to UNIGE Robotics Engineering students.  
Once that instructions in [Software tools needed section](#software-tools-needed) have been followed, do the following steps:
+ clone this repository in your workspace and source it
+ go in the `script` folder of this repository and run `chmod +x *`
+ go back in your workspace and run `catkin_make`

To run the assignment, open a terminal and execute:  
```
roslaunch assignment2 assignment2.launch 2>/dev/null
```  
Every node will display its log messages in different terminals while the ARMOR Server for the API Client will be automatically launched and a new window displaying the running state diagram with SMACH Viewer will open. Also the simulation on Gazebo will start.  
To see the robot model, open a new terminal and execute:
```
roslaunch assignment2 display.launch 2>/dev/null
```
In this way it is possible to see how the robot moves in the environment based on the information given by specific topics.

## Running code explanation
In the following video it can be seen how the architecture behaves:  



## Working hypothesis
To implement this solution, some hypothesis were made:
* Robot can charge its battery only in CORRIDOR E.
* Robot starts in location E and waits for the ontology to be created.
* After completely building the ontology, battery performance is simulated and starts with a low level.
* Topic `/state/battery_low` informs the robot if battery level is low but not critical: enough battery level for robot to move from its current location to CORRIDOR E is taken into consideration. In this way, current state doesn't need to be preempted.
* When `state_machine` is in *RandomMoving* or *Waiting* and battery level is low, robot is able to get back in location E even if it is not directly connected to current location.
* Topic `/state/new_ontology` informs the robot if the ontology is created and, thus, if the robot need to load it again.
* Robot moves mainly on randomly chosen CORRIDORs unless an URGENT location is reachable.
* An URGENT location in order to be reachable has to be directly connected with the location in which Robot1 is currently in.
* At first, all the locations (both CORRIDORs and ROOMs) are URGENT.

## Limitation and future improvements
