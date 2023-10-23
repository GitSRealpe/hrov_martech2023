#!/usr/bin/env python
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node("tf2_turtle_listener")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    print("getting it")
    br = tf2_ros.TransformBroadcaster()
    trans = tf2_ros.TransformStamped()
    trans.transform.rotation.w = 1
    while not rospy.is_shutdown():
        try:
            trans: tf2_ros.TransformStamped = tfBuffer.lookup_transform(
                "world_ned", "2", rospy.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rate.sleep()

        trans.child_frame_id = "last_seen"
        trans.header.stamp = rospy.Time.now()
        br.sendTransform(trans)

        valve = tf2_ros.TransformStamped()
        valve.header.frame_id = "last_seen"
        valve.child_frame_id = "valve"
        valve.header.stamp = rospy.Time.now()
        valve.transform.translation.y = -0.30
        valve.transform.rotation.w = 1
        br.sendTransform(valve)

        try:
            valvew: tf2_ros.TransformStamped = tfBuffer.lookup_transform(
                "world_ned", "valve", rospy.Time()
            )
            print(valvew)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rate.sleep()

        rate.sleep()

    # br = tf2_ros.TransformBroadcaster()
    # print("got it")

    # trans.child_frame_id = "last_seen"
    # print(trans)
    # while not rospy.is_shutdown():
    #     trans.header.stamp = rospy.Time.now()
    #     br.sendTransform(trans)
    #     rate.sleep()
