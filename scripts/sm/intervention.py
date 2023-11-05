import rospy
import os
from statemachine import StateMachine, State

import actionlib.simple_action_client

from std_msgs.msg import String
from optihrov2_exploration.msg import BaseGoal
from optihrov2_exploration.srv import PlanGoal, PlanGoalRequest
from std_srvs.srv import Trigger
from girona_utils.msg import PathAction, PathGoal
from nav_msgs.msg import Path


class IAUVIntervention(StateMachine):
    # states
    start = State(initial=True)
    auv_hovering = State()
    unfolding = State()
    arm_hovering = State()
    arm_positioning= State()
    valve_turning = State()

    # actions
    move_auv = start.to(auv_hovering)
    move_arm = auv_hovering.to(unfolding) | unfolding.to(arm_hovering) | arm_hovering.to(arm_positioning)
    turn_valve=arm_positioning.to(valve_turning)

    def on_enter_state(self, event, state):
        print(f"Now at state '{state.id}'")

    def on_exit_state(self, event, state):
        print(f"Exiting '{state.id}' state from '{event}' event.")

    def on_enter_auv_hovering(self):
        clusterSrv = rospy.ServiceProxy("getClusters", Trigger)
        clusterSrv.call()

    def on_enter_unfolding(self):
        goal: BaseGoal = rospy.wait_for_message("/base_goal", BaseGoal)
        planSrv = rospy.ServiceProxy("getPath", PlanGoal)
        req = PlanGoalRequest()
        req.position = goal.position
        req.yaw = goal.yaw
        planSrv(req)

    def on_enter_arm_hovering(self):
        client = actionlib.SimpleActionClient("path_manager", PathAction)
        client.wait_for_server()
        # Creates a goal to send to the action server.
        goal = PathGoal()
        goal.path = rospy.wait_for_message("planner/path_result", Path)
        # Sends the goal to the action server.
        client.send_goal(goal)
        client.wait_for_result()
        self.send("doCluster")

    # def on_exit_following(self):
    #     print("Go ahead!")


sm = IAUVIntervention()
sm._graph().write_png(os.getcwd() + "/intervention_states.png")


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
sm.send("doCluster")

rospy.init_node("state_machine")
rospy.Subscriber("/stage", String, stageCB, queue_size=1)
rospy.spin()
