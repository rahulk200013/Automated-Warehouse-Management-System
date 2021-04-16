#! /usr/bin/env python

"""
ROS Node for UR5_2 arm to pick packages from conveyor belt and place them in respective bins
according to their colour.

When the order is dispatched from UR5_1 arm side, the details of the order are received by this
node. This information is used to determine the bin in which the package needs to be dropped and
to update the "ShippedOrders" sheet using Action Client. The position from where the arm will pick
the package from conveyor belt is determined on the basis of whether the arm will be able to pickup
the package from home position #1 or not. If not it will pick the package from home position #2.
This is done to make sure that the conveyor belt is kept on for most of the time and packages are
picked as soon as possible from the conveyor belt.
"""

from __future__ import print_function

import sys
from copy import deepcopy
import math
import time
import json

from datetime import datetime, timedelta

import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml


from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal

from pkg_task5.msg import DispatchedPackage

from hrwros_gazebo.msg import LogicalCameraImage


class Ur5Moveit(object):
    """
    Initialize necessary objects to operate a UR5 arm

    Initialize trajectory publisher, execute client, planning scene interface,
    move group commander, vacuum gripper service and other necessary things
    for proper interface of Ur5 arm with Gazebo, MoveIt! and RViz

    :param arg_robot_name: Name of Ur5 arm to control
    :type arg_robot_name: str

    :Example:

    ur5_2 = Ur5Moveit('ur5_2')
    """

    # pylint: disable=too-many-instance-attributes

    # Constructor
    def __init__(self, arg_robot_name):

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +
                                                      "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns +
                                                          "/robot_description",
                                                          ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +
                                                             '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns +
                                                                        '/execute_trajectory',
                                                                        moveit_msgs.msg.
                                                                        ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.vacuum_gripper_service = '/eyrc/vb/ur5/activate_vacuum_gripper/' + arg_robot_name
        self._computed_plan = ''
        self.incoming_pkg_list = {}
        self.pkg_not_pickable = True
        self.pkg_detected = False
        self.next_bin = 'red'
        self.prev_bin = 'None'
        self.home = ''

        self.pkg_start_time = 0
        self.pkg_final_time = 0
        self.time_lapsed_since_dispatch = 0
        self.home_position_updated = False

        self.orders_list = []

        ros_pkg = rospkg.RosPack()
        self._pkg_path = ros_pkg.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def get_file_path(self):
        """
        Return path to saved trajectories

        :return: Path of folder where trajectories are saved
        :rtype: str
        """
        return self._file_path

    def go_to_pose(self, arg_pose):
        """
        Make the vacuum gripper go to a certain position in space

        :param arg_pose: goal pose of the vacuum gripper
        :type arg_pose: geometry_msgs.msg.Pose()
        :return: Success flag
        :rtype: bool
        """
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):
        """
        Try going to a pose again and until success or max attempts exhausted

        :param arg_pose: goal pose of the vacuum gripper
        :type arg_pose: geometry_msgs.msg.Pose()
        :param arg_max_attempts: maximum number of attempts to try
        :type arg_max_attempts: int
        :return: None
        """
        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Set joint angles of arm to pre defined joint angles

        :param arg_list_joint_angles: list of goal joint angles of UR5 arm in radians
        :type arg_list_joint_angles: list
        :return: Success flag
        :rtype: bool
        """
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.execute(self._computed_plan, wait=True)

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        Try to set the joint angles again and again until success or max attempts exhausted

        :param arg_list_joint_angles: list of goal joint angles of UR5 arm in radians
        :type arg_list_joint_angles: list
        :param arg_max_attempts: maximum number of attempts to try
        :type arg_max_attempts: int
        :return: None
        """
        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def attach_box(self, box_name, timeout=4):
        """
        Attach box received in arguments to arm in RViz

        :param box_name: name of box to attach
        :type box_name: str
        :param timeout: time(s) to wait for state update else return False
        :type timeout: int
        :return: Success Flag
        :rtype: bool
        """
        grasping_group = 'manipulator'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, box_name, timeout=4):
        """
        Detach box received in arguments to arm in RViz

        :param box_name: name of box to detach
        :type box_name: str
        :param timeout: time(s) to wait for state update else return False
        :type timeout: int
        :return: Success Flag
        :rtype: bool
        """
        self._scene.remove_attached_object(self._eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def attach_box_in_gazebo(self):
        """
        Activate vacuum gripper of arm in gazebo

        :return: None
        """
        rospy.wait_for_service(self.vacuum_gripper_service)
        try:
            activate_vacuum_gripper = rospy.ServiceProxy(self.vacuum_gripper_service, vacuumGripper)
            resp1 = activate_vacuum_gripper(True)
            return resp1.result
        except rospy.ServiceException as exception:
            print("Service call failed: %s" % exception)

    def detach_box_in_gazebo(self):
        """
        Deactivate vacuum gripper of arm in gazebo

        :return: None
        """
        rospy.wait_for_service(self.vacuum_gripper_service)
        try:
            activate_vacuum_gripper = rospy.ServiceProxy(self.vacuum_gripper_service, vacuumGripper)
            resp1 = activate_vacuum_gripper(False)
            return resp1.result
        except rospy.ServiceException as exception:
            print("Service call failed: %s" % exception)

    def remove_box(self, box_name, timeout=4):
        """
        Remove box from planning scene in RViz

        :param box_name: name of box to remove
        :type box_name: str
        :param timeout: time(s) to wait for state update else return False
        :type timeout: int
        :return: Success flag
        :rtype: bool
        """
        self._scene.remove_world_object(box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False,
                                          box_is_known=False,
                                          timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Wait before all the required changes are made in the planning scene in RViz

        :param box_is_known: Flag whether the box is present in planning scene or not
        :type box_is_known: bool
        :param box_is_attached: flag whether the box is attached to arm or not in RViz
        :type box_is_attached: bool
        :param timeout: time(s) to wait for the state to update
        :type timeout: int
        :return: success flag
        :rtype: bool
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects([self._box_name])
            is_attached = bool(attached_objects.keys())

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self._box_name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def play_saved_trajectory(self, arg_file_path, arg_file_name):
        """
        Play a trajectory which was saved before

        :param arg_file_path: Path to folder where trajectories are saved
        :type arg_file_path: str
        :param arg_file_name: Name of the trajectory to be played
        :type arg_file_name: str
        :return: success flag
        :rtype: bool
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        return ret

    def hard_play_saved_trajectory(self, arg_file_path, arg_file_name, arg_max_attempts):
        """
        Play a saved trajectory by loading it from a file again and again until success or
        max attempts exhausted

        :param arg_file_path: Path to the saved trajectories folder
        :type arg_file_path: str
        :param arg_file_name: Name of the trajectory .yaml file
        :type arg_file_name: str
        :param arg_max_attempts: Number of attempts to try playing it until success
        :type arg_max_attempts: int
        :return: Success flag
        :rtype: bool
        """
        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.play_saved_trajectory(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return flag_success

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class OrderDetails(object):
    """
    Class to store details of the package coming on conveyor belt from UR5_1
    """

    def __init__(self):
        self.name = ''
        self.colour = ''
        self.details = ''
        self.dispatch_time = 0

    def set_name(self, name):
        """Set name of package."""
        self.name = name

    def set_colour(self, colour):
        """Set colour of the package."""
        self.colour = colour

    def set_details(self, details):
        """Set other details of package like City, Cost, etc."""
        self.details = details

    def set_dispatch_time(self, dispatch_time):
        """Set dispatch time(s) of the package."""
        self.dispatch_time = dispatch_time


class RosIotBridgeActionClient(object):
    """
    Action Client to act as a bridge between ROS Nodes and IOT to update
    google spreadsheets
    """

    # Constructor
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)

        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    def on_transition(self, goal_handle):
        """
        This function will be called when there is change of state in the Action Client State
        Machine

        :param goal_handle: unique id of goal
        :return: None
        """
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success is True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """
        This function is used to send Goals to Action Server

        :param arg_protocol: IoT protocol to follow (HTTP, MQTT, etc.)
        :type arg_protocol: str
        :param arg_mode: Mode whether to publish or subscribe
        :type arg_mode: str
        :param arg_topic: Topic to subscribe or publish to
        :type arg_topic: str
        :param arg_message: Message to push or publish
        :type arg_message: str
        :return: goal handle
        """
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")

        # self.on_transition - It is a function pointer to a function which will be called when
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle


def main():
    """
    Main node to control UR5_2 arm.

    This function is called to start the pick and place operation of UR5_2 arm.
    :return: None
    """

    # Initialize the node
    rospy.init_node('node_ur5_2_place')

    # Wait for models to spawn in gazebo
    while rospy.get_time() < 6:
        continue

    # Initialize an object of type Ur5Moveit to control ur5_2
    ur5_2 = Ur5Moveit('ur5_2')

    order = OrderDetails()

    orders_drop_time_list = []

    # Action Client initialized
    action_client = RosIotBridgeActionClient()

    # Subscribe topic to get data of incoming package
    rospy.Subscriber("eyrc/vb/dispatched_pkg",      # Ros topic to subscribe
                     DispatchedPackage,           # Message type
                     dispatched_pkg_callback,     # Callback function name
                     (ur5_2, order))              # Arguments to pass to callback function

    # Subscribe topic to get data of objects under logical camera 2
    rospy.Subscriber('/eyrc/vb/logical_camera_2',
                     LogicalCameraImage,
                     logical_camera_2_callback,
                     ur5_2)

    # To store number of packages dropped in the bins
    total_pkg_dropped = 0

    prev_bin = ur5_2.prev_bin

    # Go to home position #1
    go_to_home(ur5_2)

    # Main loop to drop the packages in the bin
    while True:
        # Break the loop once all 9 packages have been dropped
        if total_pkg_dropped == 9:
            break

        # If a package is coming
        if bool(ur5_2.orders_list):

            # Get the colour of the incoming package
            current_pkg_colour = ur5_2.orders_list[0].colour

            print("Colour: ", current_pkg_colour)

            # Find the appropriate home position to go
            appropriate_home_pose = find_appropriate_home_position(current_pkg_colour,
                                                                   prev_bin,
                                                                   ur5_2)

            # Go to the appropriate home position
            go_to_home_position(appropriate_home_pose, current_pkg_colour, prev_bin, ur5_2)

            # Assign colour of the bin to drop the package in
            ur5_2.next_bin = current_pkg_colour

            file_name = ur5_2.home + '_to_' + current_pkg_colour + '_bin.yaml'

            # Wait for the package to come at the right position to be picked up
            print("Waiting")
            while ur5_2.pkg_not_pickable:
                continue

            # Print the colour of the package
            print("Waiting Complete")
            print(current_pkg_colour)

            # Attach the package to the vacuum gripper to ur5_2
            print("Attaching box")
            ur5_2.attach_box_in_gazebo()
            rospy.sleep(0.1)

            ur5_2.home_position_updated = False

            # start the conveyor belt again at full speed after picking the package
            print("Starting Conveyor belt")
            start_conveyor_belt(100)

            # Play the saved trajectory to drop package in the respective bin
            ur5_2.hard_play_saved_trajectory(ur5_2.get_file_path(), file_name, 5)

            # Deactivate the vacuum gripper to drop the package
            ur5_2.detach_box_in_gazebo()

            orders_drop_time_list.append([ur5_2.orders_list[0].name, rospy.get_time()])

            update_orders_shipped_sheet(ur5_2.orders_list[0].details, action_client)

            prev_bin = ur5_2.next_bin

            # Calculate the time lapsed since the package dropped by the ur5_1 arm
            if len(ur5_2.orders_list) > 1:
                current_time = rospy.get_time()
                ur5_2.time_lapsed_since_dispatch = current_time - ur5_2.orders_list[1].dispatch_time
                print("Time lapsed since dispatch: ", ur5_2.time_lapsed_since_dispatch)
            else:
                ur5_2.time_lapsed_since_dispatch = 0

            total_pkg_dropped += 1

            ur5_2.pkg_detected = False
            ur5_2.pkg_not_pickable = True

            ur5_2.orders_list.pop(0)


# Incoming package callback
def dispatched_pkg_callback(pkg, args):
    """
    This function is called whenever a package is dispatched by UR5_1 arm.

    This function notes the time when the package was dispatched to decide the home
    position of UR5_2 arm to go to pickup the package and append a deepcopy of the
    order details to orders_list and notify the details on the terminal as well

    :param pkg: Dispatched package
    :type pkg: DispatchedPackage
    :param args: Orders list
    :type args: list
    :return: None
    """
    ur5_2 = args[0]
    order = args[1]

    order.set_name(pkg.name)
    order.set_colour(pkg.colour)
    order.set_details(pkg.order_details)
    order.set_dispatch_time(rospy.get_time())

    ur5_2.orders_list.append(deepcopy(order))

    # Print the details of incoming package on terminal
    print("######### Package Coming ###########")
    print("Name: packagen" + pkg.name)
    print("Colour: ", pkg.colour)
    print("Order Details: ", json.loads(pkg.order_details))
    print("")


def logical_camera_2_callback(logical_camera_image, ur5_2):
    """
    This function handles the job to stop the incoming package at the right location
    on the conveyor belt to keep the conveyor belt moving most of the time.

    :param logical_camera_image: Data captured by logical camera over conveyor belt
    :type logical_camera_image: LogicalCameraImage
    :param ur5_2: Object having details of UR5_2 arm
    :type ur5_2: Object of class Ur5Moveit
    :return: None
    """

    # If a model is detected by the logical camera
    if bool(logical_camera_image.models) and bool(ur5_2.orders_list):

        # Iterate through all the models detected by the logical camera
        for obj in logical_camera_image.models:
            # If home position is updated and a package is found under logical camera
            if (ur5_2.home_position_updated and
                    obj.type == "packagen" + ur5_2.orders_list[0].name):

                if ur5_2.home == 'home_1' and obj.pose.position.y < 0.5:
                    if ur5_2.pkg_detected is False:
                        stop_conveyor_belt()
                        ur5_2.pkg_detected = True
                        ur5_2.pkg_not_pickable = False
                elif ur5_2.home == 'home_2':
                    if obj.pose.position.y < 0.06 and ur5_2.pkg_detected is False:
                        stop_conveyor_belt()
                        ur5_2.pkg_detected = True
                        ur5_2.pkg_not_pickable = False
                elif ur5_2.home == 'home_3':
                    if obj.pose.position.y < -0.22 and ur5_2.pkg_detected is False:
                        stop_conveyor_belt()
                        ur5_2.pkg_detected = True
                        ur5_2.pkg_not_pickable = False


def update_orders_shipped_sheet(order_details, action_client):
    """
    Update the Shipped google spreadsheet by sending goal to the Ros-Iot Bridge
    Action Server

    :param order_details: Details of the shipped order
    :type order_details: json object
    :param action_client: Action client to send goal to the action server
    :type action_client: Object of class RosIotBridgeActionClient
    :return: None
    """
    order = json.loads(order_details)
    timestamp = int(time.time())
    value = datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')

    team_id = "VB#0574"
    unique_id = "isPicuTP"

    if order['Item'] == 'Medicine':
        priority = 'HP'
        cost = 450
        days_till_delivery = timedelta(1)
    elif order['Item'] == 'Food':
        priority = 'MP'
        cost = 250
        days_till_delivery = timedelta(3)
    else:
        priority = 'LP'
        cost = 150
        days_till_delivery = timedelta(5)

    estimated_date_of_delivery = value + days_till_delivery
    estimated_date_of_delivery = estimated_date_of_delivery.strftime('%Y-%m-%d')

    parameters = {"id": "OrdersShipped",
                  "Team Id": team_id,
                  "Unique Id": unique_id,
                  "Order ID": order['Order ID'],
                  "City": order['City'],
                  "Item": order['Item'],
                  "Priority": priority,
                  "Shipped Quantity": '1',
                  "Cost": cost,
                  "Shipped Status": "Yes",
                  "Shipped Date and Time": str_time,
                  "Estimated Time of Delivery": estimated_date_of_delivery}

    # Send data to our spreadsheet
    action_client.send_goal("http", "POST", "NA", json.dumps(parameters))

    # Send data to eyantra's spreadsheet
    action_client.send_goal("http", "POST", "eyantra", json.dumps(parameters))


def start_conveyor_belt(power):
    """
    Function to start the conveyor belt at a certain power

    :param power: power of the belt to be set
    :type power: int
    :return: success flag
    :rtype: bool
    """
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
        power_service = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        resp1 = power_service(power)
        return resp1.result

    except rospy.ServiceException as exception:
        print("Service call failed: %s" % exception)


def stop_conveyor_belt():
    """
    Function to stop the conveyor belt

    :return: Success flag
    :rtype: bool
    """
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
        power = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        resp1 = power(0)
        return resp1.result

    except rospy.ServiceException as exception:
        print("Service call failed: %s" % exception)


def go_to_home(ur5_2):
    """Move UR5_2 arm to home position #1"""

    # Home position #1
    ur5_2_home1 = [math.radians(-29),
                   math.radians(-139),
                   math.radians(-60),
                   math.radians(-71),
                   math.radians(90),
                   math.radians(-32)]

    ur5_2.hard_set_joint_angles(ur5_2_home1, 5)


def find_appropriate_home_position(current_pkg_color, prev_bin, ur5_2):
    """
    Move the UR5_2 arm to correct position over conveyor belt so that conveyor belt
    stays on for most of the time and need to stop it only momentarily

    This is achieved by noting the time elapsed since the package is dispatched to
    determine the current position of package on conveyor belt. This information is used to
    determine whether the arm can return to home position #1 before package reaches there. if not,
    then arm will be moved to home position #2.

    :param current_pkg_color:  Colour of the next package coming on conveyor belt
    :type current_pkg_color: str
    :param prev_bin: Colour of bin from where it will go to home position
    :type prev_bin: str
    :param ur5_2: Ur5Moveit object to control UR5_2 arm
    :type ur5_2: Object of class Ur5Moveit
    :return: None
    """

    appropriate_home_pose = ''

    if prev_bin == 'red':
        if (current_pkg_color == 'red' or current_pkg_color == 'yellow') and \
                ur5_2.time_lapsed_since_dispatch < 5.3:
            appropriate_home_pose = 'home_1'
        elif current_pkg_color == 'green':
            appropriate_home_pose = 'home_3'
        else:
            appropriate_home_pose = 'home_2'

    elif prev_bin == 'yellow':
        if (current_pkg_color == 'red' or current_pkg_color == 'yellow') and \
                ur5_2.time_lapsed_since_dispatch < 4:
            appropriate_home_pose = 'home_1'
        elif current_pkg_color == 'green':
            appropriate_home_pose = 'home_3'
        else:
            appropriate_home_pose = 'home_2'

    elif prev_bin == 'green':
        if (current_pkg_color == 'red' or current_pkg_color == 'yellow') and \
                ur5_2.time_lapsed_since_dispatch < 4.1:
            appropriate_home_pose = 'home_1'
        elif current_pkg_color == 'green':
            appropriate_home_pose = 'home_3'
        else:
            appropriate_home_pose = 'home_2'

    return appropriate_home_pose


def go_to_home_position(appropriate_home_pose, current_pkg_color, prev_bin, ur5_2):
    """
    Move the UR5#2 arm to the appropriate home position to wait for the next package.

    :param appropriate_home_pose: Home pose to go to
    :type appropriate_home_pose: str
    :param current_pkg_color: Colour of the package coming on the conveyor belt
    :type current_pkg_color: str
    :param prev_bin: Colour of the bin in which the last package was dropped
    :type prev_bin: str
    :param ur5_2: Object of class Ur5Moveit to control UR5#2 arm
    :type ur5_2: Object of class Ur5Moveit.
    """

    if appropriate_home_pose == 'home_1':
        print("Home position #1")
        ur5_2.home = 'home_1'
        ur5_2.home_position_updated = True
        ur5_2.hard_play_saved_trajectory(ur5_2.get_file_path(), prev_bin + '_bin_to_home_1.yaml', 5)

    elif appropriate_home_pose == 'home_2':
        print("Home position #2")
        ur5_2.home = 'home_2'
        ur5_2.home_position_updated = True
        ur5_2.hard_play_saved_trajectory(ur5_2.get_file_path(), prev_bin + '_bin_to_home_2.yaml', 5)

    elif appropriate_home_pose == 'home_3':
        print("Home position #3")
        ur5_2.home = 'home_3'
        ur5_2.home_position_updated = True
        ur5_2.hard_play_saved_trajectory(ur5_2.get_file_path(), prev_bin + '_bin_to_home_3.yaml', 5)

    else:
        if current_pkg_color == 'green':
            ur5_2.home = 'home_3'
            ur5_2.home_position_updated = True
            ur5_2.hard_play_saved_trajectory(ur5_2.get_file_path(), 'home_1_to_home_3.yaml', 5)
        else:
            ur5_2.home = 'home_1'
            ur5_2.home_position_updated = True


if __name__ == '__main__':
    main()
