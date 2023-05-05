import rospy
from cola2_msgs.msg import WorldWaypointReq, GoalDescriptor

rospy.init_node("pose_requester")


req_pub = rospy.Publisher("/girona1000/controller/world_waypoint_req", WorldWaypointReq, queue_size=1)

world_waypoint_req = WorldWaypointReq()
# world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_TELEOPERATION_LOW
world_waypoint_req.goal.priority = GoalDescriptor.PRIORITY_NORMAL
world_waypoint_req.goal.requester = 'sebas'
world_waypoint_req.header.stamp = rospy.Time().now()
world_waypoint_req.header.frame_id = "world_ned"

world_waypoint_req.position.north = 1
world_waypoint_req.position.east = 1
world_waypoint_req.position.depth = 1
world_waypoint_req.orientation.roll = 0
world_waypoint_req.orientation.pitch = 0
world_waypoint_req.orientation.yaw = 0

while(rospy.is_shutdown):
    req_pub.publish(world_waypoint_req)
