import rospy
from cola2_msgs.msg import WorldWaypointReq, GoalDescriptor

rospy.init_node("pose_requester")


req_pub = rospy.Publisher(
    "/girona1000/controller/world_waypoint_req", WorldWaypointReq, queue_size=1
)

world_waypoint_req = WorldWaypointReq()
world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
world_waypoint_req.goal.requester = "sebas"
world_waypoint_req.header.frame_id = "world_ned"
world_waypoint_req.altitude_mode = False

world_waypoint_req.position.north = -1.0
world_waypoint_req.position.east = -1.0
world_waypoint_req.position.depth = 3.0
world_waypoint_req.orientation.roll = 0.0
world_waypoint_req.orientation.pitch = 0.0
world_waypoint_req.orientation.yaw = 1.57

world_waypoint_req.position_tolerance.x = 0.3
world_waypoint_req.position_tolerance.y = 0.3
world_waypoint_req.position_tolerance.z = 0.3

world_waypoint_req.orientation_tolerance.roll = 0.05
world_waypoint_req.orientation_tolerance.pitch = 0.05
world_waypoint_req.orientation_tolerance.yaw = 0.05

while not rospy.is_shutdown():
    # important time
    world_waypoint_req.header.stamp = rospy.Time().now()
    req_pub.publish(world_waypoint_req)
    # important publish rate
    rospy.sleep(0.1)
