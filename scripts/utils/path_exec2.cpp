#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <cola2_msgs/WorldWaypointReq.h>
#include <cola2_msgs/BodyVelocityReq.h>

class PID
{

public:
    ros::Publisher pub;
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped t;
    geometry_msgs::Pose setPoint;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2_ros::Buffer tfBuffer;

    tf2::Transform mat_curr;
    tf2::Transform mat_goal;
    tf2::Transform error;
    double roll, pitch, err_yaw, integral, derivative, last_error_x, last_error_y, last_error_z = 0;
    ros::Duration dt;
    ros::Time prev_t;
    Eigen::Matrix3d pid_err;
    bool near = false;

    cola2_msgs::BodyVelocityReq vel_req;

    PID(ros::NodeHandle &nh) : nh_(nh)
    {
        std::cout << "Initializing PID controller\n";
        tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
        while (ros::ok())
        {
            try
            {
                t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.5).sleep();
                continue;
            }
        }

        vel_req.header.frame_id = "world_ned";
        vel_req.goal.requester = "sebas";
        vel_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;

        pub = nh_.advertise<cola2_msgs::BodyVelocityReq>("/girona1000/controller/body_velocity_req", 5, true);
        prev_t = ros::Time::now();
    };

    void update(const ros::TimerEvent &event)
    {

        std::cout << "controlando\n";

        tf2::fromMsg(setPoint, mat_goal);
        t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
        tf2::fromMsg(t.transform, mat_curr);
        error = mat_curr.inverseTimes(mat_goal);

        // std::cout << "err_x: " << error.getOrigin().x() << "\n";
        // std::cout << "err_y: " << error.getOrigin().y() << "\n";
        // std::cout << "err_z: " << error.getOrigin().z() << "\n";
        tf2::Matrix3x3 m(error.getRotation());
        m.getRPY(roll, pitch, err_yaw, 1);

        // proportional
        pid_err(0, 0) = 0.5 * error.getOrigin().x();
        pid_err(1, 0) = 0.5 * error.getOrigin().y();
        pid_err(2, 0) = 0.5 * error.getOrigin().z();

        // integral for steady state error, but adds inestability
        dt = ros::Time::now() - prev_t;
        // integral += error.getOrigin().y() * dt.toSec();
        // pid_err(1, 1) = 0.1 * integral;

        // derivative smooths
        derivative = (error.getOrigin().x() - last_error_x) / dt.toSec();
        pid_err(0, 2) = 0.5 * derivative;
        derivative = (error.getOrigin().y() - last_error_y) / dt.toSec();
        pid_err(1, 2) = 0.5 * derivative;
        derivative = (error.getOrigin().z() - last_error_z) / dt.toSec();
        pid_err(2, 2) = 0.5 * derivative;

        last_error_x = error.getOrigin().x();
        last_error_y = error.getOrigin().y();
        last_error_z = error.getOrigin().z();

        prev_t = ros::Time::now();

        std::cout << "x action:" << std::clamp(pid_err.row(0).sum(), -0.1, 0.1) << "\n";
        std::cout << "y action:" << std::clamp(pid_err.row(1).sum(), -0.1, 0.1) << "\n";
        std::cout << "z action:" << std::clamp(pid_err.row(2).sum(), -0.1, 0.1) << "\n";
        std::cout << "yaw action:" << std::clamp(0.7 * err_yaw, -0.5, 0.5);
        std::cout << "raw_yaw: " << err_yaw << "\n";
        vel_req.twist.linear.x = std::clamp(pid_err.row(0).sum(), -0.1, 0.1);
        vel_req.twist.linear.y = std::clamp(pid_err.row(1).sum(), -0.1, 0.1);
        vel_req.twist.linear.z = std::clamp(pid_err.row(2).sum(), -0.1, 0.1);
        vel_req.twist.angular.z = std::clamp(0.7 * err_yaw, -0.785, 0.785);
        vel_req.header.stamp = ros::Time::now();

        pub.publish(vel_req);

        near = error.getOrigin().length() < 0.2;
    }
};

int main(int argc, char **argv)
{
    std::cout << "executing path\n";
    ros::init(argc, argv, "path_execution");
    ros::NodeHandle nh;
    nav_msgs::PathConstPtr path = ros::topic::waitForMessage<nav_msgs::Path>("/planner/path_result");

    PID pid(nh);
    ros::Timer timer = nh.createTimer(ros::Rate(10), &PID::update, &pid);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    for (auto pos : path->poses)
    {
        pid.setPoint = pos.pose;
        while (!pid.near && ros::ok())
        {
            // std::cout << pid.error.getOrigin().length() << "\n";
        }
        pid.near = false;
    }
    std::cout << "path done \n";
    ros::shutdown();

    // pid.setPoint = path->poses.at(9).pose;
    // ros::waitForShutdown();
}
