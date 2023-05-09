#include <ros/ros.h>
#include <cmath>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <hrov_martech2023/PointArray.h>

#define DEPTH 15

ros::Publisher marker_pub;
ros::Publisher unk_pub;
visualization_msgs::Marker marker;
float dist;

octomap::point3d_collection dir_vec;
octomap::point3d dir;

void getUnks(std::shared_ptr<octomap::OcTree> tree)
{
    std::cout << "processing " << tree->getResolution() << "\n";
    std::cout << "Size " << tree->size() << "\n";
    // distancia adicional segun el depth al que se trabaje

    marker.header.stamp = ros::Time::now();
    marker.points.clear();

    // for publishing to the clustering algorithm
    hrov_martech2023::PointArray pt_array;
    octomap::OcTreeNode *dummyNode;
    std::vector<octomap::OcTreeNode *> candidatos;
    for (octomap::OcTree::leaf_iterator n = tree->begin_leafs(DEPTH), end = tree->end_leafs(); n != end; ++n)
    {
        if (tree->isNodeOccupied(*n))
        {
            for (octomap::point3d elem : dir_vec)
            {
                octomap::point3d pt = n.getCoordinate() + elem;
                dummyNode = tree->search(pt, DEPTH);
                if (dummyNode != NULL && !tree->isNodeOccupied(dummyNode))
                {
                    for (octomap::point3d elem2 : dir_vec)
                    {
                        octomap::point3d pt2 = pt + elem2;
                        if (tree->search(pt2, DEPTH) == NULL)
                        {
                            std::cout << "Node center: " << pt2 << "\n";
                            std::cout << "al lado en x Unknown\n";
                            geometry_msgs::Point pt_msgs;
                            pt_msgs.x = pt2.x();
                            pt_msgs.y = pt2.y();
                            pt_msgs.z = pt2.z();
                            pt_array.puntos.push_back(pt_msgs);
                            marker.points.push_back(pt_msgs);

                            std::cout << "-------------\n";
                        }
                    }
                }
            }
        }
    }

    if (pt_array.puntos.empty())
    {
        std::cout << "no hay nodos sin explorar, finalizando programa\n";
        return;
    }

    marker_pub.publish(marker);
    unk_pub.publish(pt_array);
}

void octoCallback(const octomap_msgs::OctomapConstPtr &msg)
{

    std::shared_ptr<octomap::OcTree> tree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    getUnks(tree);
}

int main(int argc, char **argv)
{
    std::cout << "yeah octostuff\n";
    ros::init(argc, argv, "octo_proc");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("/frontiers", 10);
    unk_pub = nh.advertise<hrov_martech2023::PointArray>("/octo/unk_cells", 10);

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);

    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abs_tree));

    dist = tree->getResolution() * pow(2, 16 - DEPTH);

    // crear el marker
    marker.header.frame_id = "world";
    marker.ns = "cubos";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 0.659;
    marker.color.g = 0.659;
    marker.color.b = 0.659;
    marker.color.a = 1;
    marker.pose.orientation.w = 1;
    marker.scale.x = dist / 1.3;
    marker.scale.y = dist / 1.3;
    marker.scale.z = dist / 1.3;

    octomap::point3d dir(dist, 0, 0);
    dir_vec.push_back(dir);
    dir = octomap::point3d(-dist, 0, 0);
    dir_vec.push_back(dir);
    dir = octomap::point3d(0, dist, 0);
    dir_vec.push_back(dir);
    dir = octomap::point3d(0, -dist, 0);
    dir_vec.push_back(dir);
    dir = octomap::point3d(0, 0, dist);
    dir_vec.push_back(dir);
    dir = octomap::point3d(0, 0, -dist);
    dir_vec.push_back(dir);

    ros::Subscriber sub = nh.subscribe("/octomap_binary", 10, octoCallback);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        // ros::spin();
    }
}