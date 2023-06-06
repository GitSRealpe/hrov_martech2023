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

    cola2_msgs::BodyVelocityReq vel_req;

    Eigen::Isometry3d mat_curr;
    Eigen::Isometry3d mat_goal;
    double l = 0;
    double kp = 5;
    double err_x, err_y, err_z, err_yaw = 10;

    PID(ros::NodeHandle &nh) : nh_(nh)
    {
        std::cout << "Initializing PID controller\n";
        tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
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
                ros::Duration(0.5).sleep();
                continue;
            }
        }

        vel_req.header.frame_id = "world_ned";
        vel_req.goal.requester = "sebas";
        vel_req.goal.priority = cola2_msgs::GoalDescriptor::PRIORITY_NORMAL;

        pub = nh_.advertise<cola2_msgs::BodyVelocityReq>("/girona1000/controller/body_velocity_req", 5, true);
    };

    void update(const ros::TimerEvent &event)
    {

        std::cout << "controlando\n";

        tf2::fromMsg(setPoint, mat_goal);
        t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
        // tf2::Transform::inverseTimes
        mat_curr = tf2::transformToEigen(t);

        err_x = setPoint.position.x - t.transform.translation.x + l;
        err_y = mat_goal.translation().y() - mat_curr.translation().y() + l;
        err_z = mat_goal.translation().z() - mat_curr.translation().z() + l;
        // std::cout << "err_x: " << err_x << "\n";
        // std::cout << "err_y: " << err_y << "\n";
        // std::cout << "err_z: " << err_z << "\n";

        // tf2::Quaternion q_curr(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        // tf2::Quaternion q_goal(setPoint.orientation.x, setPoint.orientation.y, setPoint.orientation.z, setPoint.orientation.w);
        tf2::Quaternion q_curr;
        tf2::Quaternion q_goal;
        tf2::fromMsg(t.transform.rotation, q_curr);
        tf2::fromMsg(setPoint.orientation, q_goal);
        // err_yaw = tf2::angleShortestPath(q_goal, q_curr);
        err_yaw = q_goal.getAngle() - q_curr.getAngle();
        // err_yaw = mat_curr.rotation().eulerAngles(2, 1, 0)[0] - mat_curr.rotation().eulerAngles(2, 1, 0)[0];
        // err_yaw = (mat_goal.rotation() * mat_curr.rotation().transpose()).eulerAngles(0, 1, 2)[2];
        std::cout << "goal_yaw: " << q_goal.getAngle() << "\n";
        std::cout << "curr_yaw: " << q_curr.getAngle() << "\n";
        std::cout << "err_yaw: " << err_yaw << "\n";

        vel_req.twist.linear.x = 1 * err_x;
        vel_req.twist.linear.y = 1 * err_y;
        vel_req.twist.linear.z = 1 * err_z;
        vel_req.twist.angular.z = 0 * err_yaw;
        vel_req.header.stamp = ros::Time::now();

        pub.publish(vel_req);
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
    while (ros::ok())
    {

        for (auto pos : path->poses)
        {
            pid.setPoint = pos.pose;
            std::cout << "segundo \n";
            ros::Duration(1).sleep();
        }
        std::cout << "path done \n";
        ros::shutdown();

        // pid.setPoint = path->poses.at(0).pose;
        // ros::waitForShutdown();
    }
}
