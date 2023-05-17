import rospy
from cola2_msgs.msg import WorldWaypointReq, GoalDescriptor

rospy.init_node("pose_requester")


req_pub = rospy.Publisher(
    "/girona1000/controller/world_waypoint_req", WorldWaypointReq, queue_size=1
)

way_req = WorldWaypointReq()
way_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
way_req.goal.requester = "sebas"
way_req.header.frame_id = "world_ned"
way_req.altitude_mode = False

way_req.position.north = -2.0
way_req.position.east = 2.0
way_req.position.depth = 2.0
way_req.orientation.roll = 0.0
way_req.orientation.pitch = 0.0
way_req.orientation.yaw = 1.57

way_req.position_tolerance.x = 0.01
way_req.position_tolerance.y = 0.01
way_req.position_tolerance.z = 0.01

way_req.orientation_tolerance.roll = 0.05
way_req.orientation_tolerance.pitch = 0.05
way_req.orientation_tolerance.yaw = 0.05

while not rospy.is_shutdown():
    # important time
    way_req.header.stamp = rospy.Time().now()
    req_pub.publish(way_req)
    # important publish rate
    rospy.sleep(0.1)
