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
#include <Eigen/Eigenvalues>

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

void modCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

    ROS_INFO_STREAM("Goal Selected");
    ROS_INFO_STREAM(feedback->marker_name);
    ROS_INFO_STREAM(feedback->menu_entry_id);

    visualization_msgs::InteractiveMarker im;
    for (size_t i = 0; i < server->size(); i++)
    {
        server->get("goalMarker" + std::to_string(i), im);
        for (size_t m = 0; m < im.controls[1].markers.size(); m++)
        {
            im.controls[1].markers[m].color.a = 0;
        }
        im.controls[0].markers[0].color.b = 0;
        im.controls[2].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        im.controls[3].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        im.controls[4].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        im.controls[5].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        server->insert(im);
    }
    server->applyChanges();

    //

    server->get(feedback->marker_name, im);
    for (size_t m = 0; m < im.controls[1].markers.size(); m++)
    {
        im.controls[1].markers[m].color.a = 0.5;
    }
    im.controls[0].markers[0].color.b = 1;
    im.controls[2].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    im.controls[3].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    im.controls[4].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    im.controls[5].interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    server->insert(im);
    server->applyChanges();
}

void reqCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
}

void moveCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
}

void init_menu()
{
    menu_handler.insert("Select as goal", &modCb);
    menu_handler.insert("Request Path", &reqCb);
    menu_handler.insert("Move here", &moveCb);
}

void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    // ROS_INFO_STREAM(feedback->marker_name << " is now at: "
    //                                       << feedback->pose.position.x << ", " << feedback->pose.position.y
    //                                       << ", " << feedback->pose.position.z);
    // ROS_INFO_STREAM(feedback->marker_name << " with orientation: "
    //                                       << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y
    //                                       << ", " << feedback->pose.orientation.z << ", " << feedback->pose.orientation.w);
    feedback->client_id;
}

class GoalMarker
{
public:
    // GoalMarker(std::array<int, 3> pose, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server)
    // GoalMarker(Eigen::Vector3d pose, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server)
    GoalMarker(Eigen::Vector3d pose, Eigen::Vector3d dir)
    {

        // Eigen::Quaterniond q;
        // q.FromTwoVectors(pose,dir);

        std::cout << "creando marcador\n";

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world_ned";
        // int_marker.header.stamp = ros::Time::now(); the time stamp is the devil, breaks the pose frame feedback
        int_marker.pose.position.x = pose.x();
        int_marker.pose.position.y = pose.y();
        int_marker.pose.position.z = pose.z();
        int_marker.pose.orientation.w = 1;
        int_marker.scale = 1;
        int_marker.name = "goalMarker" + std::to_string(server->size());
        // int_marker.description = "goalMarker" + std::to_string(server->size());
        // pose inicial del marcador

        // main sphere
        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        control.always_visible = true;
        // control.name = "menu";

        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.r = 1;
        marker.color.g = 0.796;
        marker.color.b = 0.0;
        marker.color.a = 1;
        control.markers.push_back(marker);
        int_marker.controls.push_back(control);

        // lil sphere
        control = visualization_msgs::InteractiveMarkerControl();
        // control.interaction_mode = InteractiveMarkerControl.NONE
        control.always_visible = true;

        marker = visualization_msgs::Marker();
        marker.type = visualization_msgs::Marker::SPHERE;
        // marker.pose.position.x = trans.transform.translation.y
        // marker.pose.position.z = trans.transform.translation.z
        marker.pose.position.x = -0.539 - 3;
        marker.pose.position.z = -0.75;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.color.a = 1;
        control.markers.push_back(marker);

        // girona ghost
        marker = visualization_msgs::Marker();
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = ("package://girona1000_description/resources/meshes/girona1000.dae");
        marker.pose.position.x = -0.539 + 0.7 - 3;
        marker.pose.position.z = -0.75 + 0.4;
        marker.pose.orientation.y = -0.707;
        marker.pose.orientation.z = 0.707;
        marker.scale.x, marker.scale.y, marker.scale.z = 1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.color.a = 0;
        control.markers.push_back(marker);

        // fov center
        marker = visualization_msgs::Marker();
        marker.type = visualization_msgs::Marker::ARROW;
        // marker.pose.position.x = -0.539 + 0.7 - 3;
        // marker.pose.orientation.w = 1;
        geometry_msgs::Point arrow;
        marker.points.push_back(arrow);
        arrow.x = dir.x();
        arrow.y = dir.y();
        arrow.z = 0;
        marker.points.push_back(arrow);
        // marker.scale.x = 3;
        // marker.scale.y = 0.05;
        // marker.scale.z = 0.05;
        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1;
        marker.color.a = 1;
        control.markers.push_back(marker);
        // // fov rigth
        // marker = visualization_msgs::Marker();
        // marker.type = visualization_msgs::Marker::ARROW;
        // marker.pose.position.x = -0.539 + 0.7 - 3;
        // marker.pose.orientation.z = 0.3826834;
        // marker.pose.orientation.w = 0.9238795;
        // marker.scale.x = 4.24;
        // marker.scale.y = 0.05;
        // marker.scale.z = 0.05;
        // marker.color.g = 1;
        // marker.color.a = 0;
        // control.markers.push_back(marker);
        // // fov left
        // marker.pose.position.x = -0.539 + 0.7 - 3;
        // marker.pose.orientation.z = -0.3826834;
        // control.markers.push_back(marker);

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

    std::cout << "yeah clustering\n";
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;

    // create an interactive marker server on the topic namespace pose_markers
    server.reset(new interactive_markers::InteractiveMarkerServer("menu"));
    init_menu();

    // GoalMarker marker0({0, 0, 0}, server);
    // GoalMarker marker1({1, 1, 0}, server);

    hrov_martech2023::PointArrayConstPtr points_msg = ros::topic::waitForMessage<hrov_martech2023::PointArray>("/octo/unk_cells");
    std::vector<Eigen::Vector3d> pts;
    for (geometry_msgs::Point pt : points_msg->puntos)
    {
        pts.push_back({pt.x, pt.y, pt.z});
    }

    // shared_ptr<GaussianMixturesModel<3>> gmm = bayesian_gmm_constructor(pts, 10);
    shared_ptr<GaussianMixturesModel<3>> gmm = bayesian_gmm_constructor(pts, 10, true);
    gmm->balance_covariances(0.1);
    gmm->plot_components(1, pts, true);

    Eigen::EigenSolver<Eigen::Matrix3d> eigsolver;
    int i = 0;
    int minIndex = 0;
    for (Eigen::Vector3d pt : *gmm->means())
    {
        std::cout << pt << "\n";
        std::cout << "weights: " << gmm->weights()->at(i) << "\n";
        // std::cout << "weights:" << gmm->covariances()->at(i).eigenvalues() << "\n";
        eigsolver.compute(gmm->covariances()->at(i), true);
        std::cout << "eigen values:\n"
                  << eigsolver.eigenvalues() << "\n";
        // why not easier, in python is like, half a line of code
        std::cout << "min eigen value:" << eigsolver.eigenvalues().real().minCoeff(&minIndex) << "\n";
        std::cout << "min eigen value index:" << minIndex << "\n";

        std::cout << "eigen vectors:" << eigsolver.eigenvectors().real().col(minIndex) << "\n";
        GoalMarker marker(pt, eigsolver.eigenvectors().real().col(minIndex));
        std::cout << "----------------\n";

        i++;
    }

    while (ros::ok())
    {
        // pub.publish(marr);
        // ros::spinOnce();
        // ros::Duration(1).sleep();
        ros::spin();
    }
    server.reset();

    return 0;
}
