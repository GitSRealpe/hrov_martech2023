#! /usr/bin/env python

import rospy
import numpy as np

import actionlib
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from hrov_martech2023.msg import PIDAction, PIDActionGoal, PIDActionFeedback


class Manager:
    error = 10

    def feedbackCB(self, msg: PIDActionFeedback):
        feedback = msg.current

        self.error = np.linalg.norm(
            [feedback.position.x, feedback.position.y, feedback.position.z]
        )
        # print(self.error)

    def client(self):
        print("client initialized")
        path = rospy.wait_for_message("/planner/path_result", Path)

        client = actionlib.SimpleActionClient("path_execution", PIDAction)
        client.wait_for_server()

        # req = PIDActionGoal(goal=path.poses[0].pose)
        for poset in path.poses:
            req = PIDActionGoal(goal=poset.pose)
            client.send_goal(req, feedback_cb=self.feedbackCB)
            while (
                (not rospy.is_shutdown())
                and (self.error > 0.2)
                and (client.get_state() != GoalStatus.PREEMPTED)
            ):
                rospy.Rate(10).sleep()
            self.error = 10


if __name__ == "__main__":
    rospy.init_node("path_client_py")
    Manager().client()
