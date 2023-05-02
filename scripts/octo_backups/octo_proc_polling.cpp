#include <ros/ros.h>
#include <cmath>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <hrov_martech2023/PointArray.h>

int main(int argc, char **argv)
{
    std::cout << "yeah octostuff\n";
    ros::init(argc, argv, "octo_proc");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/frontiers", 10);
    ros::Publisher unk_pub = nh.advertise<hrov_martech2023::PointArray>("/octo/unk_cells", 10);

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_full");
    // octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");

    // despues me preocupo por los memory leaks del pointer este cando reciba desde una subscripcion
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    // octomap::AbstractOcTree *abs_tree = octomap_msgs::binaryMsgToMap(*mapa_msg);
    // std::unique_ptr<octomap::AbstractOcTree> abs_tree(octomap_msgs::msgToMap(*mapa_msg));

    std::cout << "resolution " << abs_tree->getResolution() << "\n";
    std::cout << "type " << abs_tree->getTreeType() << "\n";

    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abs_tree));
    std::cout << "resolution " << tree->getResolution() << "\n";
    std::cout << "NumNodes " << tree->calcNumNodes() << "\n";
    std::cout << "Size=Nodes " << tree->size() << "\n";
    std::cout << "NumLeafs " << tree->getNumLeafNodes() << "\n";
    std::cout << "TreeDepth " << tree->getTreeDepth() << "\n";

    // no puedo iterar sobre nodos que no existen?, no, pero puedo checkear alrededor de los que si existem
    // check adjacent cubes to see if this cube is in a frontier or it isnt
    // mandar un raycast
    // contar los hijos de cada nodo, menos de 8 es porque hay unknowns(depende si es free o ocupied?)
    // https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d/blob/master/include/exploration.h

    // crear los markers
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    // mejor hacer un cube list
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.659;
    marker.color.g = 0.659;
    marker.color.b = 0.659;
    marker.color.a = 1;
    marker.pose.orientation.w = 1;

    marker.ns = "cubos";
    marker.header.stamp = ros::Time::now();

    int i = 0;
    int depth = 15;
    // distancia adicional segun el depth al que se trabaje
    float dist = tree->getResolution() * pow(2, 16 - depth);
    marker.scale.x = dist / 1.3;
    marker.scale.y = dist / 1.3;
    marker.scale.z = dist / 1.3;
    octomap::point3d_collection pt_vec;
    octomap::point3d fr(dist, 0, 0);
    pt_vec.push_back(fr);
    fr = octomap::point3d(-dist, 0, 0);
    pt_vec.push_back(fr);
    fr = octomap::point3d(0, dist, 0);
    pt_vec.push_back(fr);
    fr = octomap::point3d(0, -dist, 0);
    pt_vec.push_back(fr);
    fr = octomap::point3d(0, 0, dist);
    pt_vec.push_back(fr);
    fr = octomap::point3d(0, 0, -dist);
    pt_vec.push_back(fr);

    // for publishing to the clustering algorithm
    hrov_martech2023::PointArray pt_array;
    for (octomap::OcTree::leaf_iterator n = tree->begin_leafs(depth), end = tree->end_leafs(); n != end; ++n)
    {
        if (!tree->isNodeOccupied(*n))
        {
            for (octomap::point3d adj_pt : pt_vec)
            {
                // octomap::point3d pt = n.getCoordinate() + pt_vec.at(0);
                octomap::point3d pt = n.getCoordinate() + adj_pt;
                // if (tree->search(pt, depth) == NULL)
                if (tree->search(pt, depth) == NULL)
                {
                    std::cout << "Node center: " << n.getCoordinate() << "\n";
                    std::cout << "al lado en x Unknown\n";

                    geometry_msgs::Point pt_msgs;
                    pt_msgs.x = pt.x();
                    pt_msgs.y = pt.y();
                    pt_msgs.z = pt.z();
                    pt_array.puntos.push_back(pt_msgs);
                    marker.points.push_back(pt_msgs);

                    std::cout << "-------------\n";
                }
            }
        }
    }

    if (pt_array.puntos.empty())
    {
        std::cout << "no hay nodos sin explorar, finalizando programa\n";
        return 1;
    }

    marker_pub.publish(marker);

    while (ros::ok())
    {
        unk_pub.publish(pt_array);
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
}