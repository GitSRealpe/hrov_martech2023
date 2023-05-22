#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <eigen_conversions/eigen_msg.h>

#include <cola2_msgs/WorldWaypointReq.h>
#include <cola2_msgs/BodyVelocityReq.h>

int main(int argc, char **argv)
{
    std::cout << "executing path\n";
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;

    nav_msgs::PathConstPtr path = ros::topic::waitForMessage<nav_msgs::Path>("/planner/path_result");

    ros::Publisher pub = nh.advertise<cola2_msgs::BodyVelocityReq>("/girona1000/controller/body_velocity_req", 5, true);

    cola2_msgs::BodyVelocityReq vel_req;
    vel_req.header.frame_id = "world_ned";
    vel_req.goal.requester = "sebas";
    vel_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;

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
    // Eigen::Isometry3d l;
    // l.translation().x() = 1;
    // double l = 0.6;
    double l = 0;
    double kp = 0.1;
    double err_x, err_y, err_z = 10;
    // mas fancy haciendo un struct con cava error con su nombre
    // std::vector<double> err = {10, 10, 10};
    Eigen::Vector3d err(10, 10, 10);
    // for (auto pos : path->poses)
    // {
    tf2::fromMsg(path->poses.at(0).pose, mat_goal);

    while (err_x > 0.2 || err_y > 0.2 || err_z > 0.2)
    {
        t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
        mat_curr = tf2::transformToEigen(t);
        // mat_goal.translation() = mat_curr.translation() + l.translation();

        err_x = mat_goal.translation().x() - mat_curr.translation().x() + l;
        err_y = mat_goal.translation().y() - mat_curr.translation().y() + l;
        err_z = mat_goal.translation().z() - mat_curr.translation().z() + l;
        std::cout << "err_x: " << err_x << "\n";
        std::cout << "err_y: " << err_y << "\n";
        std::cout << "err_z: " << err_z << "\n";

        vel_req.twist.linear.x = kp * err_x;
        vel_req.twist.linear.y = kp * err_y;
        vel_req.twist.linear.z = kp * err_z;

        vel_req.header.stamp = ros::Time::now();
        pub.publish(vel_req);
        ros::Duration(0.1).sleep();
    }

    // }
    std::cout << "path done\n";

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.1).sleep();
    //     // ros::spin();
    // }
}
