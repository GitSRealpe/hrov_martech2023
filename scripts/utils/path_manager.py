#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from hrov_martech2023.msg import PIDAction, PIDActionGoal


def path_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient("path_execution", PIDAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.

    path = rospy.wait_for_message("/planner/path_result", Path)
    print(path.poses[0].pose)

    req = PIDActionGoal(goal=path.poses[0].pose)

    # Sends the goal to the action server.
    client.send_goal(req)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


# Initializes a rospy node so that the SimpleActionClient can
# publish and subscribe over ROS.
rospy.init_node("path_client_py")
result = path_client()
