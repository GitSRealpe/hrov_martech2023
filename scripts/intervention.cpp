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

ros::Publisher stagePub, goalPub;

void modCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Goal Selected");
    ROS_INFO_STREAM(feedback->marker_name);
    ROS_INFO_STREAM(feedback->menu_entry_id);

    interactive_markers::MenuHandler::EntryHandle handle = feedback->menu_entry_id;
    interactive_markers::MenuHandler::CheckState state;
    menu_handler.getCheckState(handle, state);

    visualization_msgs::InteractiveMarker im;
    server->get(feedback->marker_name, im);
    if (state == interactive_markers::MenuHandler::CHECKED)
    {
        menu_handler.setCheckState(handle, interactive_markers::MenuHandler::UNCHECKED);
        im.controls[0].markers[0].color.b = 0;
        im.controls[0].markers[0].color.a = 1;
        im.controls[1].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        im.controls[2].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        im.controls[3].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        im.controls[4].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    }
    else
    {
        menu_handler.setCheckState(handle, interactive_markers::MenuHandler::CHECKED);
        im.controls[0].markers[0].color.b = 1;
        im.controls[0].markers[0].color.a = 0.5;
        im.controls[1].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        im.controls[2].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        im.controls[3].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        im.controls[4].interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    }
    server->insert(im);
    server->applyChanges();
    menu_handler.reApply(*server);
    server->applyChanges();
}

void moveCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::cout << "requesting path following\n";
    std_msgs::String stage;
    stage.data = "followPath";
    stagePub.publish(stage);
}

interactive_markers::MenuHandler::EntryHandle h_mode_last;

void init_menu()
{
    interactive_markers::MenuHandler::EntryHandle check_handle = menu_handler.insert("Edit Position", &modCb);
    menu_handler.setCheckState(check_handle, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler.insert("Move here", &moveCb);
}

void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    feedback->client_id;
}

class ValveMarker
{
public:
    ValveMarker(geometry_msgs::Pose &pose)
    {
        // Eigen::Quaterniond q;
        // q.FromTwoVectors(pose,dir);
        std::cout << "creando marcador\n";

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.pose.position = pose.position;
        int_marker.scale = 0.2;
        int_marker.name = "valveMarker" + std::to_string(server->size());
        // int_marker.description = "valveMarker" + std::to_string(server->size());

        // valve model
        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        control.always_visible = true;
        // control.name = "menu";
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = ("package://hrov_martech2023/data/valve.dae");
        marker.pose.position.z = -0.1;
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.7;
        marker.color.r = 1;
        marker.color.g = 0.796;
        marker.color.b = 0.0;
        marker.color.a = 1;
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);

        control = visualization_msgs::InteractiveMarkerControl();
        control.orientation.w = 1;
        control.name = "move_x";
        // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        int_marker.controls.push_back(control);

        control.orientation.w = 0.7071;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 0.7071;
        control.name = "move_y";
        // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        int_marker.controls.push_back(control);

        control.orientation.w = 0.7071;
        control.orientation.x = 0;
        control.orientation.y = 0.7071;
        control.orientation.z = 0;
        control.name = "rotate_z";
        // control.markers.push_back(marker);
        // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
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

    std::cout << "yeah intervention\n";
    ros::init(argc, argv, "interactive_valve");
    ros::NodeHandle nh;

    stagePub = nh.advertise<std_msgs::String>("/stage", 10);

    // create an interactive marker server on the topic namespace pose_markers
    server.reset(new interactive_markers::InteractiveMarkerServer("intervention", "valve"));
    init_menu();

    geometry_msgs::PoseConstPtr cam_pose = ros::topic::waitForMessage<geometry_msgs::Pose>("/rviz/current_camera_pose");

    Eigen::Isometry3d camtf;
    tf2::fromMsg(*cam_pose, camtf);

    Eigen::Isometry3d target;
    Eigen::Vector3d v = {0, 0, -1};
    target.translation() = v + target.translation();

    // Eigen::Isometry3d res;
    // res = camtf * target;

    // Eigen::IOFormat fmt(3, 0, ", ", ";\n", "[", "]", "[", "]");
    // std::cout << camtf.matrix().format(fmt) << "\n\n";
    // std::cout << target.matrix().format(fmt) << "\n\n";
    // std::cout << res.matrix().format(fmt) << "\n\n";

    geometry_msgs::Pose valvepos;
    valvepos = tf2::toMsg(camtf * target);
    ValveMarker valve(valvepos);

    while (ros::ok())
    {

        // ros::spinOnce();
        // ros::Duration(1).sleep();
        ros::spin();
    }
    server.reset();

    return 0;
}
