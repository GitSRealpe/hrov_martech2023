#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud
{

public:
    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_;

    // std::string laser_topic_;

    LaserScanToPointCloud(ros::NodeHandle n) : n_(n),
                                               laser_sub_(n_, "girona1000/profiler", 10),
                                               laser_notifier_(laser_sub_, listener_, "girona1000/laser_link", 10)
    {
        laser_notifier_.registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud", 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
    {
        sensor_msgs::PointCloud2 cloud2;
        // std::cout << "yip yip\n";
        try
        {
            // projector_.transformLaserScanToPointCloud(
            //     "girona1000/base_link", *scan_in, cloud, listener_);
            projector_.transformLaserScanToPointCloud(
                "girona1000/laser_link", *scan_in, cloud2, listener_);
        }
        catch (tf::TransformException &e)
        {
            std::cout << e.what();
            return;
        }

        // Do something with cloud.

        scan_pub_.publish(cloud2);
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstopc(n);

    // ros::NodeHandle private_nh("~");
    // private_nh.getParam("laser_topic", lstopc.laser_topic_);

    ros::spin();

    return 0;
}