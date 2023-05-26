#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Odometry.h>

static std::string topic = "/";
static std::string parent_frame = "/";
static std::string child_frame = "/";

void odom_callback(const nav_msgs::OdometryConstPtr &odom)
{
    geometry_msgs::Pose odom_pose = odom->pose.pose;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.header = odom->header;

    transformStamped.transform.translation.x = odom->pose.pose.position.x;
    transformStamped.transform.translation.y = odom->pose.pose.position.y;
    transformStamped.transform.translation.z = odom->pose.pose.position.z;
    transformStamped.transform.rotation = odom->pose.pose.orientation;

    br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom2tf");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("odom_topic", topic);
    private_nh.getParam("parent_frame", parent_frame);
    private_nh.getParam("child_frame", child_frame);

    std::cout << "Topic: " << topic << std::endl;
    std::cout << "Parent frame: " << parent_frame << std::endl;
    std::cout << "Child frame: " << child_frame << std::endl;

    ros::Subscriber odom_sub = nh.subscribe(topic, 10, odom_callback);
    ros::spin();

    return 0;
}