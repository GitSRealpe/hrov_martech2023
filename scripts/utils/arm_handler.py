import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

# rospy.init_node("bravo_handler", anonymous=True)

# rospy.wait_for_service("/girona1000/bravo/controller_manager/switch_controller")
# try:
#     switch_controller = rospy.ServiceProxy(
#         "/girona1000/bravo/controller_manager/switch_controller", SwitchController
#     )

#     swReq = SwitchControllerRequest()
#     swReq.start_controllers = ["joint_position_controller"]
#     swReq.stop_controllers = ["joint_velocity_controller"]
#     swReq.strictness = SwitchControllerRequest.STRICT
#     res = switch_controller(swReq)
#     print(res)

# except rospy.ServiceException as e:
#     print("Service call failed: %s" % e)

# msg = Float64MultiArray()

# msg.data = [1, 0, 0, 0, 0, 0]

# print(msg)
# pub.publish(msg)

#! /usr/bin/env python
import rospy
import sys

import actionlib

# Brings in the messages used by the joints action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def joints_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (jointsAction) to the constructor.
    client = actionlib.SimpleActionClient("/girona1000/bravo/joint_position_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.header.stamp=rospy.Time.now()
    goal.trajectory.joint_names=["girona1000/bravo/joint1",
                                         "girona1000/bravo/joint2",
                                         "girona1000/bravo/joint3",
                                         "girona1000/bravo/joint4",
                                         "girona1000/bravo/joint5",
                                         "girona1000/bravo/joint6"]
    
    points_msg = JointTrajectoryPoint()
    points_msg.time_from_start = rospy.Duration(10)
    points_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal.trajectory.points.append(points_msg)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A jointsResult


if __name__ == "__main__":
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node("bravo_handler")
        result = joints_client()
        print("Result:", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)