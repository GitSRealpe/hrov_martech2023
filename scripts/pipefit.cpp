/*
 * Author: SRealpe
 *
 * Copyright (c) 10/2023 CIRS-UdG
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>

#include <gmm_registration/front_end/GmmFrontEnd.hpp>
#include <gmm_registration/front_end/GaussianMixturesModel.h>
#include <Eigen/Eigenvalues>
// #include <eigen_conversions/eigen_msg.h>
// #include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    feedback->client_id;
}

class PipesMarker
{
public:
    PipesMarker(geometry_msgs::Pose &pose)
    {
        // Eigen::Quaterniond q;
        // q.FromTwoVectors(pose,dir);
        std::cout << "creando marcador\n";

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.pose.position = pose.position;
        int_marker.scale = 0.3;
        int_marker.name = "pipesMarker" + std::to_string(server->size());
        // int_marker.description = "pipesMarker" + std::to_string(server->size());

        // pipe model
        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        control.always_visible = true;
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = ("package://hrov_martech2023/data/pipeviz.dae");
        marker.pose.orientation.x = 1;
        // marker.pose.orientation.w = 0.7071068;
        // marker.pose.orientation.x = 0.7071068;
        // marker.pose.position.x = -0.70;
        // marker.pose.position.z = -1.30;
        marker.scale.x = marker.scale.y = marker.scale.z = 1;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 0.7;
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);

        control = visualization_msgs::InteractiveMarkerControl();
        control.orientation.w = 1;
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 0.7071;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 0.7071;
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 0.7071;
        control.orientation.x = 0;
        control.orientation.y = 0.7071;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        // add the interactive marker to our collection &
        // tell the server to call markerFeedback() when feedback arrives for it
        server->insert(int_marker, &markerFeedback);
        menu_handler.apply(*server, int_marker.name);
        // 'commit' changes and send to all clients
        server->applyChanges();

        std::cout << "marker creado\n";
    }
};

int main(int argc, char **argv)
{

    std::cout << "yeah pipefit\n";
    ros::init(argc, argv, "pipeviz");
    ros::NodeHandle nh;

    // create an interactive marker server on the topic namespace pose_markers
    server.reset(new interactive_markers::InteractiveMarkerServer("pipe", "pipe"));

    geometry_msgs::Pose pipespose;
    pipespose.orientation.w = 0;
    PipesMarker valve(pipespose);

    while (ros::ok())
    {

        // ros::spinOnce();
        // ros::Duration(1).sleep();
        ros::spin();
    }
    server.reset();

    return 0;
}
