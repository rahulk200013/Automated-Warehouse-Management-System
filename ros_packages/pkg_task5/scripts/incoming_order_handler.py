#! /usr/bin/env python

"""
This node is used to handle the job of processing the new order placed.

Node to handle any new order placed on MQTT topic. This node process the order
and add details like priority, cost, team ID and unique ID to the JSON object
and sends it to UR5_1 arm for use via ROS topic. This node subscribes to
ROS topic '/ros_iot_bridge/mqtt/sub' where anything published
on MQTT topic is published to get the details of the new order placed. After this a callback
new_order_callback() do the processing of the order and send it to UR5_1 arm for use.
"""

import json
import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgMqttSub

from pkg_task5.msg import IncomingOrder


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

        # Dictionary to Store all the goal handles
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
    Main function to initialize nodes, action client, subscriber and publisher
    to handle new order placed on MQTT topic
    """

    # Initialize the node
    rospy.init_node("incoming_order_handler")

    # Wait for all the models to spawn in Gazebo
    while rospy.get_time() < 5:
        continue

    # Action Client initialized
    action_client = RosIotBridgeActionClient()

    # Publisher to send the details of new order to UR5_1 arm
    order_pub = rospy.Publisher('eyrc/vb/orders', IncomingOrder, queue_size=10)

    # Listen for new orders placed on MQTT topic
    rospy.Subscriber('/ros_iot_bridge/mqtt/sub',
                     msgMqttSub,
                     new_order_callback,
                     (action_client, order_pub))

    while not rospy.is_shutdown():
        pass


def new_order_callback(msg, args):
    """
    Whenever a new order is placed on MQTT topic, this function is called to process
    the order details and send it to UR5_1 arm via ROS topic.

    :param msg: Contains the details of new order.
    :type msg: msgMqttSub
    :param args: list of Action Client and New order publisher.
    :type args: list
    """
    action_client = args[0]
    order_pub = args[1]

    new_order = json.loads(msg.message)

    team_id = "VB#0574"
    unique_id = "isPicuTP"

    if new_order["item"] == "Medicine":
        priority = "HP"
        cost = 450
    elif new_order["item"] == "Food":
        priority = "MP"
        cost = 250
    else:
        priority = "LP"
        cost = 150

    parameters = {"id": "IncomingOrders",
                  "Team Id": team_id,
                  "Unique Id": unique_id,
                  "Order ID": new_order["order_id"],
                  "Order Date and Time": new_order["order_time"],
                  "Item": new_order["item"],
                  "Priority": priority,
                  "Order Quantity": 1,
                  "City": new_order["city"],
                  "Longitude": new_order["lon"],
                  "Latitude": new_order["lat"],
                  "Cost": cost}

    order_pub.publish(json.dumps(parameters))

    # Send data to our spreadsheet
    action_client.send_goal("http", "POST", "NA", json.dumps(parameters))

    # Send data to eyantra's spreadsheet
    action_client.send_goal("http", "POST", "eyantra", json.dumps(parameters))


if __name__ == '__main__':
    main()
