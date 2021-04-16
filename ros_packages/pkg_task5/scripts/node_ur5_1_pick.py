#! /usr/bin/env python

"""
ROS Node for UR5_1 arm to pick packages from shelf and place them on conveyor belt.

At first, the colour of the packages are detected by importing the qr node. Then the inventory
sheet is updated by sending a goal to the action server. After that whenever a new order is
placed on the MQTT topic "/eyrc/vb/isPicuTP/orders", it is appended to a local list. This list is
sorted according to the priority of the orders and then the order  at the top of list is processed
and picked up from the shelf and placed on conveyor belt. The details of the dispatched order are
than updated in the "DispatchedOrder" sheet by sending a goal to the action server.
"""

from __future__ import print_function
import math
import time
import sys
import datetime
import json

import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg
import yaml

from pkg_vb_sim.srv import conveyorBeltPowerMsg

from pkg_vb_sim.srv import vacuumGripper

from pkg_task5.msg import Camera1Image
from pkg_task5.msg import DispatchedPackage
from pkg_task5.msg import NewOrder

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal

from qr import qr_decode


class Ur5Moveit(object):
    """
    Initialize necessary objects to operate a UR5 arm

    Initialize trajectory publisher, execute client, planning scene interface,
    move group commander, vacuum gripper service and other necessary things
    for proper interface of Ur5 arm with Gazebo, MoveIt! and RViz

    :param arg_robot_name: Name of Ur5 arm to control
    :type arg_robot_name: str

    :Example:

    ur5_1 = Ur5Moveit('ur5_1')
    """

    # pylint: disable=too-many-instance-attributes

    # Constructor
    def __init__(self, arg_robot_name):

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +
                                                      "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns +
                                                          "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +
                                                             '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        self._execute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns +
                                                                       '/execute_trajectory',
                                                                       moveit_msgs.msg.
                                                                       ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.vacuum_gripper_service = '/eyrc/vb/ur5/activate_vacuum_gripper/' + arg_robot_name
        self._computed_plan = ''

        ros_pkg = rospkg.RosPack()
        self._pkg_path = ros_pkg.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # Waiting for new order
        self.waiting = False

        # Time when the UR5#1 arm started waiting for new orders
        self.waiting_start_time = 0

        # Flag whether we should check if there is any benefit in going to the waiting position
        self.check_waiting_position_benefit = False

        # Flag to tell if UR5#1 arm is at waiting pose or not
        self.at_waiting_pose = False

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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

    def play_saved_trajectory(self, trajectory):
        """
        Play a trajectory which was saved before

        :param trajectory: Trajectory to be played
        :type trajectory: JointTrajectory
        :return: success flag
        :rtype: bool
        """
        ret = self._group.execute(trajectory)
        return ret

    def hard_play_saved_trajectory(self, trajectory, arg_max_attempts):
        """
        Try playing a trajectory again and again until success or max attempts exhausted

        :param trajectory: Trajectory to be played
        :type trajectory: JointTrajectory
        :param arg_max_attempts: Max number of attempts to try again in case of failure
        :type arg_max_attempts: int
        :return: success flag
        :rtype: bool
        """
        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.play_saved_trajectory(trajectory)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return True

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class ShelfCamera(object):
    """
    Store information regarding colour of packages on shelf as recognised by the 2D Camera
    """

    def __init__(self):
        self.camera_sub = rospy.Subscriber("eyrc/package/colours", Camera1Image, self.sub_callback)
        self.pkg_colour_list = [['NA' for _column in range(3)] for _row in range(4)]

    def get_pkg_colour(self, row, column):
        """
        Return colour of package present on specific row and column

        :param row: row of package
        :type row: int
        :param column: column of package
        :type column: int
        :return: list of colour of all the packages present on shelf
        :rtype: list
        """
        return self.pkg_colour_list[row][column]

    def sub_callback(self, camera_image):
        """
        Update colour of packages in the local list identified by the 2D Camera

        :param camera_image: information of all the packages seen by the camera
        :type camera_image: Camera1Image
        :return: None
        """
        for row in range(4):
            for column in range(3):
                # Update local list of colour of all the packages present on the shelf
                self.pkg_colour_list[row][column] = camera_image.pkg_colour[3*row + column]

    def print_colours(self):
        """
        Print colour of all the packages present on the shelf with their name(row and column).

        :return: None
        """
        for row in range(4):
            for column in range(3):
                print("package" + str(row) + str(column), self.pkg_colour_list[row][column])

    def get_pkg_colour_list(self):
        """
        Return list of colour of all the packages present on the shelf

        :return: List of colour of all the packages present on the shelf
        :rtype: list
        """
        return self.pkg_colour_list


class DispatchedPkg(object):
    """ Class to store name and colour of the dispatched package"""

    def __init__(self):
        self.name = 'NA'
        self.colour = 'NA'

    def set_name(self, name):
        """
        Set name of the dispatched package

        :param name: name of the package
        :type name: str
        :return: None
        """
        self.name = name

    def set_colour(self, colour):
        """
        Set colour of the dispatched package

        :param colour: colour of the package
        :type colour: str
        :return: None
        """
        self.colour = colour


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
        # Comm State = 3 -> Waiting for Result
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
    Main node to control UR5_1 arm.

    This function is called to start the pick and place operation of UR5_1 arm.
    :return: None
    """

    # Initialize the node
    rospy.init_node('node_ur5_1_pick')

    # Wait for models to spawn in gazebo
    while rospy.get_time() < 6:
        continue

    # Action Client initialized
    action_client = RosIotBridgeActionClient()

    # Initialize object of class colourPkg to process colours of the packages
    camera = ShelfCamera()
    rospy.sleep(0.1)

    # Detect colours of all the packages
    qr_decode.detect_colours()
    rospy.sleep(0.1)

    orders_list = []

    rospy.Subscriber('eyrc/vb/orders', NewOrder, new_order_callback, orders_list)

    # Update spreadsheet with Packages available in Inventory
    update_inventory(camera.get_pkg_colour_list(), action_client)
    rospy.sleep(0.5)

    # Dictionary to load and store all the saved trajectories
    saved_trajectories = {}

    # Load saved trajectories and store them in dictionary for future use
    load_saved_trajectories(saved_trajectories)

    # Initialize object of class Ur5Moveit to control ur5_1
    ur5_1 = Ur5Moveit('ur5_1')

    # Initialize object to store data of the dispatched package
    dispatched_pkg = DispatchedPkg()

    # Topic to publish the dispatched pkg
    dispatched_pkg_pub = rospy.Publisher('eyrc/vb/dispatched_pkg', DispatchedPackage, queue_size=10)

    # Go to home position
    go_to_home_position(ur5_1)

    # Start the conveyor belt at 100% speed
    start_conveyor_belt(100)

    # Main loop
    while True:

        if ur5_1.check_waiting_position_benefit:
            # Play the saved to trajectory to go to the waiting position
            benefit = waiting_position_benefit(camera)

            # If there is any benefit
            if benefit:
                trajectory_name = 'home_to_waiting_pose_1.yaml'
                ur5_1.hard_play_saved_trajectory(saved_trajectories[trajectory_name], 5)

                ur5_1.at_waiting_pose = True

            # Reset check benefit flag
            ur5_1.check_waiting_position_benefit = False

        # If there is any order we need to process
        if orders_list:

            # Pick out the first order in the sorted orders list
            current_order = orders_list[0]

            # Remove the order we are processing from the list
            orders_list.pop(0)

            # Find the location of the package to pick from the shelf
            row, column = find_correct_pkg_on_shelf(camera, current_order, ur5_1.at_waiting_pose)

            # If correct package is found
            if row or column is not None:

                # Move UR5#1 arm to the package to pick it up
                go_to_package(ur5_1, saved_trajectories, row, column)

                # Reset waiting mechanism
                ur5_1.waiting = False
                ur5_1.at_waiting_pose = False

                # Attach the package EE of ur5_1
                ur5_1.attach_box_in_gazebo()

                # Play the trajectory to drop the package on conveyor belt
                trajectory_name = 'pkgn' + str(row) + str(column) + '_to_home.yaml'
                ur5_1.hard_play_saved_trajectory(saved_trajectories[trajectory_name], 5)

                # Deactivate the vacuum gripper to drop the package
                ur5_1.detach_box_in_gazebo()

                # Store data of dispatched package
                dispatched_pkg.set_name(str(row) + str(column))
                dispatched_pkg.set_colour(current_order["Colour"])

                # Publish the data of the package that has been just dropped
                dispatched_pkg_pub.publish(dispatched_pkg.name,
                                           dispatched_pkg.colour,
                                           json.dumps(current_order))

                # Update DispatchedOrders sheet
                update_orders_dispatched_sheet(current_order, action_client)

        # If arm not at waiting position currently
        elif not ur5_1.at_waiting_pose:
            # If arm has just processed an order and no new order left to process, start waiting
            if not ur5_1.waiting:
                ur5_1.waiting_start_time = rospy.get_time()
                ur5_1.waiting = True

            # If waited for more than 2 seconds, set check benefit at waiting pose flag to true
            if rospy.get_time() - ur5_1.waiting_start_time > 2:
                ur5_1.check_waiting_position_benefit = True

    while not rospy.is_shutdown():
        continue


def go_to_home_position(ur5_1):
    """
    Move UR5#1 arm to its home position

    :param ur5_1: Object of class Ur5Moveit to control UR5#1 arm
    :type ur5_1: Object of class Ur5Moveit
    :return: None
    """

    # Home joint angles (position over the conveyor belt)
    ur5_1_home_joint_angles = [math.radians(4),
                               math.radians(-109),
                               math.radians(-82),
                               math.radians(-79),
                               math.radians(90),
                               math.radians(-90)]

    ur5_1.set_joint_angles(ur5_1_home_joint_angles)


def update_inventory(pkg_colour_list, action_client):
    """
    Update inventory spreadsheet

    :param pkg_colour_list: List of colour of all the packages present on the shelf
    :type pkg_colour_list: list
    :param action_client: action client to send goals to push data on spreadsheet
    :type action_client: Object of class RosIotBridgeActionClient
    :return: None
    """
    team_id = "VB#0574"
    unique_id = "isPicuTP"
    for row in range(4):
        for column in range(3):
            now = datetime.datetime.now()
            if pkg_colour_list[row][column] == 'red':
                sku = "R"+str(row)+str(column)+now.strftime("%m")+now.strftime("%y")
                item = "Medicine"
                priority = "HP"
                cost = 450
            elif pkg_colour_list[row][column] == 'yellow':
                sku = "Y" + str(row) + str(column) + now.strftime("%m") + now.strftime("%y")
                item = "Food"
                priority = "MP"
                cost = 250
            else:
                sku = "G" + str(row) + str(column) + now.strftime("%m") + now.strftime("%y")
                item = "Clothes"
                priority = "LP"
                cost = 150

            storage_no = "R"+str(row)+" C"+str(column)
            quantity = 1

            parameters = {"id": "Inventory",
                          "Team Id": team_id,
                          "Unique Id": unique_id,
                          "SKU": sku,
                          "Item": item,
                          "Priority": priority,
                          "Storage Number": storage_no,
                          "Cost": cost,
                          "Quantity": quantity}

            # Send data to our spreadsheet
            action_client.send_goal("http", "POST", "NA", json.dumps(parameters))

            # Send data to eyantra's spreadsheet
            action_client.send_goal("http", "POST", "eyantra", json.dumps(parameters))

            rospy.sleep(1)


def update_orders_dispatched_sheet(order, action_client):
    """
    Update dispatched orders spreadsheet

    :param order: Information of the order dispatched
    :type order: dict
    :param action_client: action client to send goal to push data on spreadsheet
    :type action_client: Object of class RosIotBridgeActionClient
    :return: None
    """
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime("%Y-%m-%d %H:%M:%S")

    team_id = "VB#0574"
    unique_id = "isPicuTP"

    if order["Item"] == "Medicine":
        priority = "HP"
        cost = 450
    elif order["Item"] == "Food":
        priority = "MP"
        cost = 250
    else:
        priority = "LP"
        cost = 150

    parameters = {"id": "OrdersDispatched",
                  "Team Id": team_id,
                  "Unique Id": unique_id,
                  "Order ID": order["Order ID"],
                  "City": order["City"],
                  "Item": order["Item"],
                  "Priority": priority,
                  "Dispatch Quantity": 1,
                  "Cost": cost,
                  "Dispatch Status": "Yes",
                  "Dispatch Date and Time": str_time}

    # Send data to our spreadsheet
    action_client.send_goal("http", "POST", "NA", json.dumps(parameters))

    # Send data to eyantra's spreadsheet
    action_client.send_goal("http", "POST", "eyantra", json.dumps(parameters))


def new_order_callback(msg, orders_list):
    """
    Callback function called whenever a new order is placed

    :param msg: Order details
    :type msg: json object
    :param orders_list: List of the current active orders
    :type orders_list: list
    :return: None
    """

    order = json.loads(msg.order)

    if order["Priority"] == "HP":
        order["Colour"] = "red"
    elif order["Priority"] == "MP":
        order["Colour"] = "yellow"
    else:
        order["Colour"] = "green"

    orders_list.append(order)

    sorted_orders_list = sorted(orders_list, key=lambda i: i["Cost"], reverse=True)
    del orders_list[:]
    orders_list.extend(sorted_orders_list)

    print("######### UPDATED ORDERS LIST ##########")
    print(orders_list)


def load_saved_trajectories(saved_trajectories):
    """
    Load all the saved trajectories and store them in a dictionary

    :param saved_trajectories: Dictionary to store the loaded trajectories
    :type saved_trajectories: dict
    :return: None
    """
    ros_pkg = rospkg.RosPack()
    arg_file_path = ros_pkg.get_path('pkg_task5') + '/config/saved_trajectories/'
    for row in range(4):
        for column in range(3):
            arg_file_name = 'home_to_pkgn' + str(row) + str(column) + '.yaml'
            file_path = arg_file_path + arg_file_name

            with open(file_path, 'r') as file_open:
                saved_trajectories[arg_file_name] = yaml.load(file_open)

            arg_file_name = 'pkgn' + str(row) + str(column) + '_to_home.yaml'
            file_path = arg_file_path + arg_file_name

            with open(file_path, 'r') as file_open:
                saved_trajectories[arg_file_name] = yaml.load(file_open)

    pkgs_in_waiting_area = ['12', '22', '02', '32', '20', '21']

    for pkg in pkgs_in_waiting_area:
        arg_file_name = 'waiting_pose_1_to_pkgn' + pkg + '.yaml'
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            saved_trajectories[arg_file_name] = yaml.load(file_open)

    arg_file_name = 'home_to_waiting_pose_1.yaml'
    file_path = arg_file_path + arg_file_name

    with open(file_path, 'r') as file_open:
        saved_trajectories[arg_file_name] = yaml.load(file_open)


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


def waiting_position_benefit(camera):
    """
    This function is called to check if there will be any benefit in picking up package quickly
    from the shelf if we go to waiting position or not. It checks whether there are 3 types of
    packages(red, yellow, and green) are available in the shelf area covered by the waiting
    position or not.

    :param camera: object storing information packages present on the shelf
    :type camera: Object of class ShelfCamera which stores all info related to packages on shelf
    :return: Benefit or not to go and wait at home position
    :rtype: bool
    """

    # Package found flags
    red_pkg_found = False
    yellow_pkg_found = False
    green_pkg_found = False

    # Benefit of waiting at waiting position or not
    benefit = False

    # Package numbers which can be reached quickly from the waiting position
    pkgs_in_waiting_area = [12, 22, 2, 32, 20, 21]

    for pkg_no in pkgs_in_waiting_area:
        row = int(pkg_no / 10)
        column = pkg_no % 10

        # If red package is found
        if camera.get_pkg_colour(row, column) == 'red':
            red_pkg_found = True
        # If yellow package is found
        elif camera.get_pkg_colour(row, column) == 'yellow':
            yellow_pkg_found = True
        # If green package is found
        elif camera.get_pkg_colour(row, column) == 'green':
            green_pkg_found = True

        # If all three types of packages are found
        if red_pkg_found and yellow_pkg_found and green_pkg_found:
            # Set benefit to true
            benefit = True
            break

    # Return benefit (True/False)
    return benefit


def find_correct_pkg_on_shelf(camera, current_order, at_waiting_pose):
    """
    Function to find the correct package to be picked up from the shelf

    :param camera: object storing information packages present on the shelf
    :type camera: Object of class ShelfCamera which stores all info related to packages on shelf
    :param current_order: Details of order to find on the shelf
    :type current_order: dict
    :param at_waiting_pose: If UR5#1 arm is at waiting pose or not
    :type at_waiting_pose: bool
    :return: row and column of the required package
    :rtype: int, int
    """
    # Search order of packages while selecting the package to pick up
    pkg_search_order = [12, 2, 22, 1, 32, 11, 0, 20, 10, 21, 30]

    # Package search order for packages covered by waiting position
    pkgs_in_waiting_area = [12, 22, 2, 32, 20, 21]

    row = None
    column = None

    if at_waiting_pose:
        for pkg_no in pkgs_in_waiting_area:
            row = int(pkg_no / 10)
            column = pkg_no % 10

            if camera.get_pkg_colour(row, column) == current_order['Colour']:
                # Update the inventory and mark the package as NA
                camera.pkg_colour_list[row][column] = 'NA'
                break
    else:
        for pkg_no in pkg_search_order:
            row = int(pkg_no / 10)
            column = pkg_no % 10

            if camera.get_pkg_colour(row, column) == current_order['Colour']:
                # Update the inventory and mark the package as NA
                camera.pkg_colour_list[row][column] = 'NA'
                break

    # Return row and column of the package to pick up
    return row, column


def go_to_package(ur5_1, saved_trajectories, row, column):
    """
    Move the UR5#1 arm to the package to pick it up

    :param ur5_1: Object of class Ur5Moveit to control UR5#1 arm
    :type ur5_1: Object of class Ur5Moveit
    :param saved_trajectories: Dictionary of all the saved trajectories already loaded
    :type saved_trajectories: dict
    :param row: Row of the package to pick up
    :type row: int
    :param column: Column of the package to pick up
    :type column: int
    """

    # If the arm is currently at the waiting position
    if ur5_1.at_waiting_pose:
        # Play the saved to trajectory to pick the package
        trajectory_name = 'waiting_pose_1_to_pkgn' + str(row) + str(column) + '.yaml'
        ur5_1.hard_play_saved_trajectory(saved_trajectories[trajectory_name], 5)
    else:
        # Play the saved to trajectory to pick the package
        trajectory_name = 'home_to_pkgn' + str(row) + str(column) + '.yaml'
        ur5_1.hard_play_saved_trajectory(saved_trajectories[trajectory_name], 5)


if __name__ == '__main__':
    main()
