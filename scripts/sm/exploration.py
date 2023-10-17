import rospy
import os
from statemachine import StateMachine, State

import actionlib.simple_action_client

from std_msgs.msg import String
from hrov_martech2023.msg import BaseGoal
from hrov_martech2023.srv import PlanGoal, PlanGoalRequest
from girona_utils.msg import PathAction, PathGoal
from nav_msgs.msg import Path

value = 1


class IAUVExploration(StateMachine):
    "A traffic light machine"

    clustering = State(initial=True)
    planning = State()
    following = State()

    doCluster = following.to(clustering)
    planPath = clustering.to(planning) | planning.to(planning) | following.to(planning)
    # rePlan = planning.to(planning) | following.to(planning)
    followPath = planning.to(following)

    # cycle = (
    #     clustering.to(planning)
    #     | rePlan
    #     | planning.to(following)
    #     | following.to(clustering)
    # )

    def on_enter_state(self, event, state):
        print(f"Now at state '{state.id}'")

    def on_exit_state(self, event, state):
        print(f"Exiting '{state.id}' state from '{event}' event.")

    def on_enter_planning(self):
        goal: BaseGoal = rospy.wait_for_message("/base_goal", BaseGoal)
        planSrv = rospy.ServiceProxy("getPath", PlanGoal)
        req = PlanGoalRequest()
        req.position = goal.position
        req.yaw = goal.yaw
        planSrv(req)

    def on_enter_following(self):
        client = actionlib.SimpleActionClient("path_manager", PathAction)
        print("before the wait")
        client.wait_for_server()
        print("after the wait")
        # Creates a goal to send to the action server.
        goal = PathGoal()
        goal.path = rospy.wait_for_message("planner/path_result", Path)
        # Sends the goal to the action server.
        client.send_goal(goal)

    # def on_exit_following(self):
    #     print("Go ahead!")


sm = IAUVExploration()
sm._graph().write_png(os.getcwd() + "/st.png")


def stageCB(msg: String):
    print("got stage: " + msg.data)
    if msg.data == "planPath":
        sm.send("planPath")
    if msg.data == "followPath":
        sm.send("followPath")


# print(sm._graph().to_string())

# sm.send("planPath")
# sm.send("rePlan")
# sm.send("followPath")
# sm.send("rePlan")

rospy.init_node("state_machine")
rospy.Subscriber("/stage", String, stageCB, queue_size=1)
rospy.spin()
