/*
 * Author: SRealpe
 *
 * Copyright (c) 8/2023 CIRS-UdG
 */

#include <ros/ros.h>
#include <hrov_martech2023/PointArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/front_end/GaussianMixturesModel.h>

int main(int argc, char **argv)
{

    std::cout << "yeah clustering\n";
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;

    // octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("/centroids", 10, false);

    hrov_martech2023::PointArrayConstPtr points_msg = ros::topic::waitForMessage<hrov_martech2023::PointArray>("/octo/unk_cells");
    std::vector<Eigen::Vector3d> pts;

    for (geometry_msgs::Point pt : points_msg->puntos)
    {
        pts.push_back({pt.x, pt.y, pt.z});
    }

    shared_ptr<GaussianMixturesModel<3>> gmm = bayesian_gmm_constructor(pts, 10);
    // gmm->balance_covariances(0.1);

    visualization_msgs::MarkerArray marr;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.frame_id = "world_ned";
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 1;
    marker.color.g = 0.796;
    marker.color.b = 0.0;
    marker.color.a = 1;

    marker.pose.orientation.w = 1;
    int i = 0;
    for (Eigen::Vector3d pt : *gmm->means())
    {
        std::cout << pt << "\n";
        std::cout << "weights\n"
                  << gmm->weights()->at(i) << "\n";
        std::cout << "----------------\n";

        // gmm->weights()->at(i);

        marker.id = i++;
        marker.pose.position.x = pt.x();
        marker.pose.position.y = pt.y();
        marker.pose.position.z = pt.z();
        marr.markers.push_back(marker);
    }

    while (ros::ok())
    {
        pub.publish(marr);
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    return 0;
}
