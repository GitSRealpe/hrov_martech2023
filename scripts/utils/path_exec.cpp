#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <eigen_conversions/eigen_msg.h>

#include <cola2_msgs/WorldWaypointReq.h>

int main(int argc, char **argv)
{
    std::cout << "executing path\n";
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;

    nav_msgs::PathConstPtr path = ros::topic::waitForMessage<nav_msgs::Path>("/planner/path_result");

    ros::Publisher pub = nh.advertise<cola2_msgs::WorldWaypointReq>("/girona1000/controller/world_waypoint_req", 5, true);

    cola2_msgs::WorldWaypointReq way_req;
    way_req.goal.requester = "sebas";
    way_req.header.frame_id = "world_ned";
    way_req.altitude_mode = false;
    way_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;
    way_req.position_tolerance.x = 0.01;
    way_req.position_tolerance.y = 0.01;
    way_req.position_tolerance.z = 0.01;
    way_req.orientation_tolerance.roll = 0.01;
    way_req.orientation_tolerance.pitch = 0.01;
    way_req.orientation_tolerance.yaw = 0.01;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped t;

    while (true)
    {
        try
        {
            t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
    }

    Eigen::Isometry3d mat_curr;
    Eigen::Isometry3d mat_goal;
    for (auto pos : path->poses)
    {
        way_req.position.north = pos.pose.position.x;
        way_req.position.east = pos.pose.position.y;
        way_req.position.depth = pos.pose.position.z;
        // way_req.orientation.roll = 0.0;
        // way_req.orientation.pitch = 0.0;
        Eigen::Quaterniond q;
        tf2::fromMsg(pos.pose.orientation, q);
        way_req.orientation.yaw = q.toRotationMatrix().eulerAngles(2, 1, 0)[0];

        mat_curr = tf2::transformToEigen(t);
        tf2::fromMsg(pos.pose, mat_goal);
        // tf::poseMsgToEigen(pos.pose, mat_goal);
        while ((mat_curr.translation() - mat_goal.translation()).norm() > 0.4 && ros::ok())
        {
            t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
            mat_curr = tf2::transformToEigen(t);

            std::cout << (mat_curr.translation() - mat_goal.translation()).norm() << " error \n";
            way_req.header.stamp = ros::Time::now();
            pub.publish(way_req);
            ros::Duration(0.1).sleep();
        }
    }
    std::cout << "path done\n";

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    //     // ros::spin();
    // }
}
